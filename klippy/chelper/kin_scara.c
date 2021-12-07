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
    bool arm_mode;
};

// Calculate arm poitions
static double
scara_pos_to_angle(double dx, double dy, double proximal_length, double distal_length,
                    double crosstalk, bool arm_mode, bool arm_type)
{
    const float cosPsi = (fsquare(dx) + fsquare(dy) - fsquare(proximal_length) - fsquare(distal_length)) / (2 * proximal_length * distal_length);
    float square = 1.0 - fsquare(cosPsi);
    float psi = acosf(cosPsi);
    float theta = 0;
    const float sinPsi = sqrtf(square);
    const float sk_1 = proximal_length + distal_length * cosPsi;
    const float sk_2 = distal_length * sinPsi;

    if (arm_mode)
    {       
        theta = atan2f(sk_1 * dy - sk_2 * dx, sk_1 * dx + sk_2 * dy);
    }
    else
    {
        theta = atan2f(sk_1 * dy + sk_2 * dx, sk_1 * dx - sk_2 * dy);
        psi = -psi;
    }
    float x_angle = theta;
    float y_angle = psi - (crosstalk * theta);
    if (arm_type)
    {
        return x_angle;
    }
    else
    {
        return y_angle;
    }
}

static double
scara_stepper_calc_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    double motor_pos = scara_pos_to_angle(c.x, c.y, fs->proximal_length, fs->distal_length,
                                            fs->crosstalk, fs->arm_mode, fs->arm_type);
    return motor_pos;                                   
}

struct stepper_kinematics * __visible
scara_stepper_alloc(char arm,
                    double proximal_length, double distal_length,
                    double crosstalk, double arm_mode)
{
    struct scara_stepper *f = malloc(sizeof(*fs));
    memset(fs, 0, sizeof(*fss));
    fs->sk.calc_position_cb = scara_stepper_calc_position;
    fs->proximal_length = proximal_length;
    fs->distal_length = distal_length;
    fs->crosstalk = crosstalk;
    fs->arm_mode = arm_mode;
    if (arm == 'p')
    {
        fs->arm_type = bool true
    } 
    else if (arm == 'd')
    {
        fs->arm_type = bool false
    }
    fs->sk.active_flags = AF_X | AF_Y;
    return &fs->sk;
}
