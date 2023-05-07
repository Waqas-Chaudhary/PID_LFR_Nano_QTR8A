#ifndef MOTOR_DRIVER_HPP_
#define MOTOR_DRIVER_HPP_

#include <Arduino.h>

typedef struct
{
    int left;
    int right;
    int max;
    int turn;
} motor_speeds_t;

void motor_driver_init(void);

void motor_forward(int left_pwm_speed, int right_pwm_speed);

void motor_backward(int left_pwm_speed, int right_pwm_speed);

void motor_right_sharp(int left_pwm_speed, int right_pwm_speed);

void motor_left_sharp(int left_pwm_speed, int right_pwm_speed);

void motor_stop(void);

#endif // MOTOR_DRIVER_HPP_