#include <Arduino.h>
#include "ir_sensor.hpp"
#include "motor_driver.hpp"

// PID related
#define GOAL (int)    3500         // 3500 is value to keep the robot at centre w.r.t line.
#define Kp   (double) 0.012 * 2.0
#define Kd   (double) 0.01 * 10.0

#define LINE_EXTREME_LIMIT_LEFT   0    // position of line when it is at extreme left
#define LINE_EXTREME_LIMIT_RIGHT  7000 // position of line when it is at extreme right
#define LINE_CENTER_LIMIT_LEFT    3200 // acceptable center line position on the left side
#define LINE_CENTER_LIMIT_RIGHT   3800 // acceptable center line position on the right side
#define CHECK_SIDE_MAX_ITERATIONS 2000 // number of iterations after which the check_side function gives up

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
 * @brief This function is used to get back on line if line is lost
 * completely. 0 is one extreme end of the line and 7000 is the other
 * extreme end of the line.
 * 
 * @param position position of robot
 * @param motor_speeds motor speeds of robot
 */
// This function checks if the position is at the extreme limits (left or right)
// and adjusts the motor speeds to bring the position back to the center limits.
void check_side(uint16_t position, const motor_speeds_t * motor_speeds)
{
    // Function pointer to select the appropriate motor function
    void (*motor_function)(int, int) = NULL;

    if (position == LINE_EXTREME_LIMIT_LEFT)
    {
        motor_function = motor_left_sharp;
    }
    else if (position == LINE_EXTREME_LIMIT_RIGHT)
    {
        motor_function = motor_right_sharp;
    }
    else
    {
        motor_function = NULL;
    }

    // If a motor function is selected, adjust the motor speeds to bring the position back to the center limits.
    if (motor_function != NULL)
    {
        while (CHECK_SIDE_MAX_ITERATIONS)
        {
            motor_function(motor_speeds->max, motor_speeds->turn);
            position = ir_sensors_read_line();
            Serial.println(position);

            if ((position > LINE_CENTER_LIMIT_LEFT) && (position < LINE_CENTER_LIMIT_RIGHT))
            {
                break;
            }
        }
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