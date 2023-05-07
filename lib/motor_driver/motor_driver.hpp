#ifndef MOTOR_DRIVER_HPP_
#define MOTOR_DRIVER_HPP_

#include <Arduino.h>

/**
 * @struct motor_speeds_t
 * 
 * @brief A structure to hold motor speeds and speed limits.
 * this makes it easy to carry the group of related variables
 * across the program.
 */
typedef struct
{
    int left;  // Left motor speed value
    int right; // Right motor speed value
    int turn;  // Turn speed value for both motors
    int max;   // Maximum speed limit for motors
    int min;   // Minimum speed limit for motors
} motor_speeds_t;

/**
 * @brief Initializes the motor driver pins as output pins.
 * This function sets the motor driver pins IN1, IN2, IN3, and IN4 as output
 * pins using the pinMode function.
 */
void motor_driver_init(void);

/**
 * @brief Ensures motor speeds stay within limits.
 * This function checks and corrects the motor speeds stored in the provided
 * motor_speeds_t structure to ensure they are within the defined limits.
 * 
 * @param speeds A pointer to a motor_speeds_t structure containing motor speeds and limits.
 */
void speed_limit_check(motor_speeds_t * speeds);

void motor_forward(int left_pwm_speed, int right_pwm_speed);

void motor_backward(int left_pwm_speed, int right_pwm_speed);

void motor_right_sharp(int left_pwm_speed, int right_pwm_speed);

void motor_left_sharp(int left_pwm_speed, int right_pwm_speed);

void motor_stop(void);

#endif // MOTOR_DRIVER_HPP_