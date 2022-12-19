// Serial Scara Arm
//
// Copyright (C) 2021 Koal Designs <koaldesigns@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct scara_stepper {
    struct stepper_kinematics sk;
    double proximal_length;
    double distal_length;
    double crosstalk;
    int arm_mode;
    int arm_type;
};

static double 
square(double a)
{
    double a2 = a*a;
    return a2;
}

// Calculate arm poitions
static double
scara_pos_to_angle(double dx, double dy, double proximal_length, double distal_length,
                    double crosstalk, int arm_mode, int arm_type)
{
    const double cosPsi = (square(dx) + square(dy) - square(proximal_length) - square(distal_length)) / (2 * proximal_length * distal_length);
    double square1 = 1.0 - square(cosPsi);
    double psi = acos(cosPsi);
    double theta = 0;
    const double sinPsi = sqrt(square1);
    const double sk_1 = proximal_length + distal_length * cosPsi;
    const double sk_2 = distal_length * sinPsi;

    if (arm_mode)
    {       
        theta = atan2(sk_1 * dy - sk_2 * dx, sk_1 * dx + sk_2 * dy);
    }
    else
    {
        theta = atan2(sk_1 * dy + sk_2 * dx, sk_1 * dx - sk_2 * dy);
        psi = -psi;
    }
    double d_angle = theta;
    double p_angle = psi - (crosstalk * theta);
    if (arm_type)
    {
        return d_angle;
    }
    else
    {
        return p_angle;
    }
}

static double
scara_proximal_stepper_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    struct scara_stepper *fs = container_of(sk, struct scara_stepper, sk);
    double angle = scara_pos_to_angle(c.x, c.y, fs->proximal_length, fs->distal_length,
                                            fs->crosstalk, fs->arm_mode, 0);
    return angle;                                   
}

static double
scara_distal_stepper_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    struct scara_stepper *fs = container_of(sk, struct scara_stepper, sk);
    double angle = scara_pos_to_angle(c.x, c.y, fs->proximal_length, fs->distal_length,
                                            fs->crosstalk, fs->arm_mode, 1);
    return angle;                                   
}

struct stepper_kinematics * __visible
scara_stepper_alloc(char arm,
                    double proximal_length, double distal_length,
                    double crosstalk, int arm_mode)
{
    struct scara_stepper *fs = malloc(sizeof(*fs));
    memset(fs, 0, sizeof(*fs));
    fs->proximal_length = proximal_length;
    fs->distal_length = distal_length;
    fs->crosstalk = crosstalk;
    fs->arm_mode = arm_mode;
    if (arm == 'p')
    {
        fs->sk.calc_position_cb = scara_proximal_stepper_position;
    } 
    else if (arm == 'd')
    {
        fs->sk.calc_position_cb = scara_distal_stepper_position;
    }
    fs->sk.active_flags = AF_X | AF_Y;
    return &fs->sk;
}
