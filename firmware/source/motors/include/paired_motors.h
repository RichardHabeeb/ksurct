/****************************************************************************************
* File: paired_motors.h
*
* Description: Header file for paired_motors.cpp
*
* Created: 04/20/2014, by Kyle McGahee
****************************************************************************************/

#ifndef PAIRED_MOTORS_INCLUDED_H
#define PAIRED_MOTORS_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "motor_interface.h"
#include "stm32f4xx.h"

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

// Defines turn directions.
typedef enum
{
    turn_right,
    turn_left
} motor_turn_t;

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: PairedMotors
*
* Description: Put class description here.
******************************************************************************/
class PairedMotors
{
public: // methods

    // Constructor
    PairedMotors
        (
            IMotor & right_motor,           // From driver perspective.
            IMotor & left_motor,            // From driver perspective.
            float    wheel_2_wheel_distance // AKA wheel base.  In centimeters.
        );

    // Initializes both motors.
    void Initialize(void);

    // Set both motors to drive at specified velocity.
    // If speed is negative then motors will drive in reverse.
    void Drive
        (
            float new_velocity // cm / sec
        );

    // Turn the motor pair the correct 'angle' in degrees in the direction specified using a
    // zero point turn approach.
    void ZeroPointTurn
        (
            motor_turn_t  new_direction,  // Which direction to turn.
            float         turn_angle,     // How far to turn in degrees
            float         turn_speed      // How fast to turn in degrees per second.
        );

    // Commands both motors to stop and will wait until they are.
    void Stop(void);

    // Returns true only if both motors are stopped.
    bool inline Stopped(void);

    float get_wheel_base_distance(void) { return wheel_2_wheel_distance; }

    // Resets current distance measurement for both motors.
    void reset_current_distance(void);

    // Provide properties to access each motor directly.
    // Right/left designation is from driver perspective.
    IMotor & get_right_motor(void) const { return *right_motor; }
    IMotor & get_left_motor(void)  const { return *left_motor; }

private: // methods

private: // fields

    IMotor * right_motor;
    IMotor * left_motor;

    float wheel_2_wheel_distance; // AKA wheel base.  In centimeters.
    float cm_per_degree_zero_point;

}; // PairedMotors

#endif // PAIRED_MOTORS_INCLUDED_H
