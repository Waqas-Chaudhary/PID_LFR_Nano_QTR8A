#include <Arduino.h>
#include "ir_sensor.hpp"
#include "motor_driver.hpp"

// PID related
#define GOAL (int)    3500         // 3500 is value to keep the robot at centre w.r.t line.
#define Kp   (double) 0.012 * 2.0
#define Kd   (double) 0.01 * 10.0

void setup()
{
    Serial.begin(9600);
    motor_driver_init();
    ir_sensors_init();
}

void PID(uint16_t position, motor_speeds_t * speeds)
{
    int error      = 0;
    int pid_output = 0;
    static int previous_error; // last error, var to store previous error values.

    position = ir_sensors_read_line();
    Serial.println(position);

    // PID calculations
    error = GOAL - position;
    pid_output = Kp * error + Kd * (error - previous_error);
    previous_error = error;

    // adjusting motor speeds
    speeds->left  = speeds->max - pid_output;
    speeds->right = speeds->max + pid_output;

    // ceiling and floor conditions for left motor speed
    speeds->left = (speeds->left > speeds->max) ? speeds->max : speeds->left;
    speeds->left = (speeds->left < 0) ? 0 : speeds->left;

    // ceiling and floor conditions for right motor speed
    speeds->right = (speeds->right > speeds->max) ? speeds->max : speeds->right;
    speeds->right = (speeds->right < 0) ? 0 : speeds->right;
}

/**
 * @brief This function is used here to get back on line if line is 
 * lost completely.
 * 
 * @param position position of robot
 * @param motor_speeds motor speeds of robot
 */
void check_side(uint16_t position, motor_speeds_t * motor_speeds)
{
    if (position == 0)
    {
        do
        {
            motor_left_sharp(motor_speeds->max, motor_speeds->turn);
            position = ir_sensors_read_line();
            Serial.println(position);
            if ((position > 3200) && (position < 3800))
            {
                break;
            }
        } while (1);
    }
    else if (position == 7000)
    {
        do
        {
            motor_right_sharp(motor_speeds->turn, motor_speeds->turn);
            position = ir_sensors_read_line();
            Serial.println(position);
            if ((position > 3200) && (position < 3800))
            {
                break;
            }
        } while (1);
    }
}

void loop()
{
    motor_speeds_t motor_speeds = 
    {
        .left  = 0,
        .right = 0,
        .max   = 65,
        .turn  = 65,
    };

    static uint16_t position;

    PID(position, &motor_speeds);
    motor_forward(motor_speeds.left, motor_speeds.right);
    check_side(position, &motor_speeds);
}