/****************************************************************************************
* File: differential_paired_stepper_motors.h
*
* Description: Header file for differential_paired_stepper_motors.cpp
*
* Created: 1/13/2014, by Kyle McGahee
****************************************************************************************/

#ifndef DIFFERENTIAL_PAIRED_STEPPER_MOTORS_INCLUDED_H
#define DIFFERENTIAL_PAIRED_STEPPER_MOTORS_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "stepper_motor.h"

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: DifferentialPairedStepperMotors
*
* Description: Put class description here.
******************************************************************************/
class DifferentialPairedStepperMotors
{
public: // methods

    // Constructor
    DifferentialPairedStepperMotors
        (
            bool    acceleration_enabled,      // If true then motors will accelerate.
            float   turn_speed,                // Degrees per second
            float   wheel_diameter,            // Centimeters
            float   wheel_2_wheel_distance,    // AKA wheel base.  In centimeters.
            float   full_steps_per_revolution, // How many full steps needed to rotate wheel once.
            uint8_t pulses_per_step            // How many steps per full steps. Ie 2 for half stepping.
        );

    // Must be called before using. Initializes GPIO pins to default state and sets up
    // timer interrupts for stepping/accelerating.
    void Initialize(void);

    // Clears motor control fields and re-init both motors.
    void ReInitialize(void);

    // Set both motors to drive at specified speed and resets current step count.
    // If speed is negative then motors will drive in reverse.
    // If acceleration is enabled then the new speed will be accelerated to from the
    // current speed, otherwise the motors will attempt to automatically jump to the new
    // speed, which can sometime stall the motors.
    void Drive
        (
            float new_speed // cm / sec
        );

    // Set right motor to drive at speed.  If speed is negative then motor will drive in reverse.
    // Note: Current steps are NOT reset when calling this function.
    void DriveRight
        (
            float new_speed // cm / sec
        );

    // Set left motor to drive at speed.  If speed is negative then motor will drive in reverse.
    // Note: Current steps are NOT reset when calling this function.
    void DriveLeft
        (
            float new_speed // cm / sec
        );

    // Turn the robot the correct 'angle' in degrees in the direction specified using a
    // zero point turn approach.
    void Turn
        (
            motor_direction_t new_direction, // Which direction to turn.
            float             turn_angle     // Degrees
        );

    // Commands both motors to stop and will wait until they are.
    void Stop(void);

    // Returns true only if both motors are stopped.
    bool inline Stopped(void);

    // Getters
    float get_right_motor_current_distance(void);
    float get_left_motor_current_distance(void);
    float get_right_motor_total_distance(void);
    float get_left_motor_total_distance(void);
    float get_wheel_base_distance(void) { return wheel_2_wheel_distance; }

private: // methods

    // Enable/disable acceleration mode.
    void SetAccelerationMode
        (
            bool new_enabled_status // If true then acceleration mode will be enabled.
        );

private: // fields

    StepperMotor * right_motor;
    StepperMotor * left_motor;

    // If true then motors will accelerate to target speed.
    bool acceleration_enabled;

    float wheel_diameter;      // cm
    float wheel_circumference; // cm

    // Distance between center of two wheels.  The 'Wheel Base' of the robot
    float wheel_2_wheel_distance;      // cm
    float wheel_2_wheel_circumference; // cm

    // Number of steps need for the wheel to make one full rotation.
    float full_steps_per_revolution;

    // Number of pulses that make up one full step
    // Ie 2 for half-stepping, 4 for quarter stepping, etc
    uint8_t pulses_per_step;

    float full_steps_per_cm;
    float cm_per_full_step;

    // The amount of time to take to accelerate from current speed to target speed.
    float acceleration_time_constant; // seconds

    // How many steps/sec to increment every acceleration ISR.
    float velocity_update_inc;

    // How many centimeters (arclength) robot will travel for each degree rotated.
    float cm_per_degree_zero_point;

    // Speed needed for turns in units of full steps / second.
    uint32_t turn_speed_full_steps_per_sec;

}; // DifferentialPairedStepperMotors

#endif // DIFFERENTIAL_PAIRED_STEPPER_MOTORS_INCLUDED_H
