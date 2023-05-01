#include <Arduino.h>
#include "ir_sensor.hpp"
#include "motor_driver.hpp"

// PID related
      int    L_Error = 0;          // last error, var to store previous error values.
const int    Goal    = 3500;       // 3500 is value to keep the robot at centre w.r.t line.
const double Kp      = 0.012 * 2;
const double Kd      = 0.01 * 10;

void setup()
{
    Serial.begin(9600);
    motor_driver_init();
    ir_sensors_init();
}

void PID(uint16_t Position, motor_speeds_t * motor_speeds)
{
    int Error = 0;
    int Adj   = 0;

    Position = ir_sensors_read_line();
    Serial.println(Position);

    Error = Goal - Position;
    
    // Calculating adjustment to pwm for left & right motors
    Adj = Kp * Error + Kd * (Error - L_Error);
    L_Error = Error;
    motor_speeds->left  = motor_speeds->max - Adj;
    motor_speeds->right = motor_speeds->max + Adj;

    if (motor_speeds->left > motor_speeds->max)
    {
        motor_speeds->left = motor_speeds->max;
    }
    else if (motor_speeds->left < 0)
    {
        motor_speeds->left = 0;
    }

    if (motor_speeds->right > motor_speeds->max)
    {
        motor_speeds->right = motor_speeds->max;
    }
    else if (motor_speeds->right < 0)
    {
        motor_speeds->right = 0;
    }
}

// check_side() is used here to get back on line if line is lost completely.
void check_side(uint16_t Position, motor_speeds_t * motor_speeds)
{
    if (Position == 0)
    {
        do
        {
            sharpL(motor_speeds->max, motor_speeds->turn);
            Position = ir_sensors_read_line();
            Serial.println(Position);
            if ((Position > 3200) && (Position < 3800))
            {
                break;
            }
        } while (1);
    }
    else if (Position == 7000)
    {
        do
        {
            sharpR(motor_speeds->turn, motor_speeds->turn);
            Position = ir_sensors_read_line();
            Serial.println(Position);
            if ((Position > 3200) && (Position < 3800))
            {
                break;
            }
        } while (1);
    }
}

void loop()
{
    static motor_speeds_t motor_speeds;
    static uint16_t Position;

    PID(Position, &motor_speeds);
    frd_PWM(motor_speeds.left, motor_speeds.right);
    check_side(Position, &motor_speeds);
}