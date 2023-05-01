#include "motor_driver.hpp"


// MX1508 Motor Driver Pins
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

void frd_PWM(int L, int R)
{
    analogWrite(IN1, L);
    analogWrite(IN2, 0);
    analogWrite(IN3, R);
    analogWrite(IN4, 0);
}

void bck(int L, int R)
{
    analogWrite(IN1, 0);
    analogWrite(IN2, L);
    analogWrite(IN3, 0);
    analogWrite(IN4, R);
}

void sharpR(int L, int R)
{
    analogWrite(IN1, L);
    analogWrite(IN2, 0);
    analogWrite(IN3, 0);
    analogWrite(IN4, R);
}

void sharpL(int L, int R)
{
    analogWrite(IN1, 0);
    analogWrite(IN2, L);
    analogWrite(IN3, R);
    analogWrite(IN4, 0);
}

void stop()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}