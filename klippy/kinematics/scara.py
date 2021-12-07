# Serial Scara Arm
#
# Copyright (C) 2021 Koal Designs <koaldesigns@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper, mathutil, chelper

class ScaraKinematics:
    def __init__(self, toolhead, config):
        # Setup arm rails and z rail
        rail_proximal = stepper.PrinterRail(config.getsection('stepper_proximal'),
                                             units_in_radians=True)
        self.proximal_length = rail_proximal.getfloat('proximal_length', above=0.)
        rail_proximal.setup_itersolve(
            'scara_stepper_alloc', 'p',
            self.proximal_length, self.distal_length, self.crosstalk, self.arm_mode)

        rail_distal = stepper.PrinterRail(config.getsection('stepper_distal'),
                                             units_in_radians=True)
        self.distal_length = rail_distal.getfloat('distal_length', above=0.)
        rail_distal.setup_itersolve(
            'scara_stepper_alloc', 'd',
            self.proximal_length, self.distal_length, self.crosstalk, self.arm_mode)

        rail_z = stepper.LookupMultiRail(config.getsection('stepper_z'))
        rail_z.setup_itersolve('cartesian_stepper_alloc', 'z')

        self.rails = [rail_proximal, rail_distal, rail_z]

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)

        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.limit_z = (1.0, -1.0)
        self.XY_homed = False

        # Homing trickery to fake cartesian kinematics
        self.printer = config.get_printer()
        ffi_main, ffi_lib = chelper.get_ffi()
        self.cartesian_kinematics_P = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc('x'), ffi_lib.free)
        self.cartesian_kinematics_D = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc('y'), ffi_lib.free)

        logging.info('SCARA Configs: P %.2f,   D %.2f',
                     self.proximal_length, self.distal_length)

        # Read config
        self.crosstalk = config.getfloat('xy_crosstalk_factor')
        self.theta_limits = config.getfloatlist('theta_limits')
        self.psi_limits = config.getfloatlist('psi_limits')
        self.min_radius = config.getfloat('minimum_radius')
        self.arm_mode = config.getbool('arm_mode')
        if self.min_radius == None:
            self.min_radius = self.proximal_length - self.distal_length
            if self.min_radius <= 0:
                self.min_radius = 0
        self.continuous_rotation = [(self.theta_limits[1] - self.theta_limits[0] > 2*math.pi),
                                        (self.psi_limits[1] - self.psi_limits[0] > 2*math.pi)]
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def angle_to_pos(self, theta, psi):
        x_pos = (math.acos(theta) * self.proximal_length) + (math.acos(psi + theta) * self.distal_length)
        y_pos = (math.asin(theta) * self.proximal_length) + (math.asin(psi + theta) * self.distal_length)
        return [x_pos, y_pos]

    def calc_position(self, stepper_positions):
        # Converts stepper positioning to cartesian coordinates
        theta = stepper_positions[self.rails[0].get_name()]
        psi = stepper_positions[self.rails[1].get_name()] * (self.crosstalk[0] * theta)
        x_pos = (math.acos(theta) * self.proximal_length) + (math.acos(psi + theta) * self.distal_length)
        y_pos = (math.asin(theta) * self.proximal_length) + (math.asin(psi + theta) * self.distal_length)
        z_pos = stepper_positions[self.rails[2].get_name()]
        return [x_pos, y_pos, z_pos]

    def set_position(self, newpos, homing_axes):
        logging.info('Running set_position: %s,   %s', newpos, homing_axes)
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
        if 2 in homing_axes:
            self.limit_z = self.rails[2].get_range()
            logging.info('Set Z Limits: %f,   %f', self.limit_z[0], self.limit_z[1])
        if 0 in homing_axes and 1 in homing_axes:
            self.XY_homed = True
            logging.info('Set X, Y to: %f,   %f', newpos[0], newpos[1])

    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limit_z = (1.0, -1.0)

    def home(self, homing_state):
        # Always home X/Y = Prox/Dist together
        # How to specify rotational homing which is not the theta/psi angle 0?
        homing_axes = homing_state.get_axes()
        logging.info('Homing: %s', homing_axes)
        home_xy = 0 in homing_axes or 1 in homing_axes
        home_z = 2 in homing_axes
        if home_xy:
            # Home distal and proximal at the same time
            self.XY_homed = False
            homing_state.set_axes([0, 1])
            joints = self.rails[:2]
            p_endstop = joints[0].get_homing_info().position_endstop
            p_min, p_max = joints[0].get_range()
            d_endstop = joints[1].get_homing_info().position_endstop
            d_min, d_max = joints[1].get_range()

            # Switch to linear kinematics
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.flush_step_generation()
            joint_steppers = self.get_steppers()[:2]
            linear_kinematics = [self.cartesian_kinematics_D, self.cartesian_kinematics_P]
            prev_sks = [stepper.set_stepper_kinematics(kinematic)
                            for stepper, kinematic in zip(joint_steppers, linear_kinematics)]
            try:
                homepos = [p_endstop, d_endstop, None, None]
                forcepos = [None, None, None, None]
                p_hi = joints[0].get_homing_info()
                d_hi = joints[1].get_homing_info()
                if p_hi.positive_dir:
                    forcepos[0] = p_max
                else:
                    forcepos[0] = p_min
                if d_hi.positive_dir:
                    forcepos[1] = d_max
                else:
                    forcepos[1] = d_min
                homing_state.home_rails(joints, forcepos, homepos)
                for stepper, prev_sk in zip(joint_steppers, prev_sks):
                    stepper.set_stepper_kinematics(prev_sk)
                [x,y] = self.angle_to_pos(
                    joints[0].get_homing_info().position_endstop,
                    joints[1].get_homing_info().position_endstop)
                toolhead.set_position([x, y, 0, 0], (0, 1))
                toolhead.flush_step_generation()
                logging.info('P and D are homed')

            except Exception as e:
                for stepper, prev_sk in zip(joint_steppers, prev_sks):
                    stepper.set_stepper_kinemations(prev_sk)
                toolhead.flush_stp_generation()
                raise
        if home_z:
            rail = self.rails[2]
            z_min, z_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, hi.position_endstop]
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[2] -= 1.5 * (hi.position_endstop - z_min)
            else:
                forcepos[2] += 1.5 * (z_max - hi.position_endstop)
            homing_state.home_rails([rail], forcepos, homepos)
            logging.info('Z is homed')

    def _motor_off(self, print_time):
        self.XY_homed = False

    def check_theta_psi(self, x_pos, y_pos):
        # Checks to see if the theta or psi angle is out of the limits
        x = x_pos
        y = y_pos
        cosPsi = (x**2 + y**2 - self.proximal_length**2 - self.distal_length**2) / (2 * self.proximal_length * self.distal_length)
        square = 1.0 - cosPsi**2
        psi = self.psi = math.acos(cosPsi)
        sinPsi = math.sqrt(square)
        sk_1 = self.proximal_length + self.distal_length * cosPsi
        sk_2 = self.distal_length * sinPsi

        # Try current arm mode, then the other (right=true, left=false)
        while True:
            if self.arm_mode:
                if self.continuous_rotation[1] | (psi >= self.psi_limits[0] & psi <= self.psi_limits[1]):
                    theta = self.theta = math.atan2(sk_1 * y - sk_2 * x, sk_1 * x + sk_2 * y)
                    if self.continuous_rotation[0] | (theta >= self.theta_limits[0] & theta <= self.theta_limits[1]):
                        return True
            else:
                if self.continuous_rotation[1] | (-psi >= self.psi_limits[0] & -psi <= self.psi_limits[1]):
                    theta = self.theta = math.atan2(sk_1 * y + sk_2 * x, sk_1 * x - sk_2 * y)
                    if self.continuous_rotation[0] | (theta >= self.theta_limits[0] & theta <= self.theta_limits[1]):
                        psi = self.psi = -psi
                        return True
            return False

    def check_move(self, move):
        end_pos = move.end_pos
        # PD Moves
        if move.axes_d[0] or move.axes[1]:
            x_pos, y_pos = end_pos[:2]
            if not self.XY_homed:
                raise move.move_error('Must home XY before moving')
                
            # Make sure location is in a reachable location
            max_radius = self.proximal_length + self.distal_length
            pos_radius = math.sqrt(x_pos**2 + y_pos**2)
            if pos_radius < self.min_radius or pos_radius > max_radius:
                raise move.move_error('XY position is outside of reachable area')
            if not self.check_theta_psi(x_pos, y_pos):
                raise move.move_error('Movement is outside of theta or psi limits')

        if move.axes_d[2]:
            if end_pos[2] < self.limit_z[0] or end_pos[2] > self.limit_z[1]:
                if self.limit_z[0] > self.limit_z[1]:
                    raise move.move_error("Must home Z axis before moving")
                raise move.move_error()

            # Move with Z - update velocity and accel for slower Z axis
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(
                self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    def get_status(self, eventtime):
        xy_home = "XY" if self.XY_homed else ''
        z_home = "Z" if self.limit_z[0] <= self.limit_z[1] else ''
        return {
            'homed_axes': xy_home + z_home,}

def load_kinematics(toolhead, config):
    return ScaraKinematics(toolhead, config)
