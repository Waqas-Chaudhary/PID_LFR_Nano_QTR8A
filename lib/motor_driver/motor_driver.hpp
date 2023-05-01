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

void frd_PWM(int L, int R);

void bck(int L, int R);

void sharpR(int L, int R);

void sharpL(int L, int R);

void stop();

#endif // MOTOR_DRIVER_HPP_