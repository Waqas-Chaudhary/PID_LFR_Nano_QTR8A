#include "motor_driver.hpp"

// Motor Driver Pins
#define IN1 10
#define IN2 11
#define IN3 6
#define IN4 9

void motor_driver_init(void)
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void motor_forward(int left_pwm_speed, int right_pwm_speed)
{
    analogWrite(IN1, left_pwm_speed);
    analogWrite(IN2, 0);
    analogWrite(IN3, right_pwm_speed);
    analogWrite(IN4, 0);
}

void motor_backward(int left_pwm_speed, int right_pwm_speed)
{
    analogWrite(IN1, 0);
    analogWrite(IN2, left_pwm_speed);
    analogWrite(IN3, 0);
    analogWrite(IN4, right_pwm_speed);
}

void motor_right_sharp(int left_pwm_speed, int right_pwm_speed)
{
    analogWrite(IN1, left_pwm_speed);
    analogWrite(IN2, 0);
    analogWrite(IN3, 0);
    analogWrite(IN4, right_pwm_speed);
}

void motor_left_sharp(int left_pwm_speed, int right_pwm_speed)
{
    analogWrite(IN1, 0);
    analogWrite(IN2, left_pwm_speed);
    analogWrite(IN3, right_pwm_speed);
    analogWrite(IN4, 0);
}

void motor_stop(void)
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}