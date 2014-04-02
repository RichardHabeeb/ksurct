/****************************************************************************************
* File: paired_dc_brushed_motors.h
*
* Description: Header file for paired_dc_brushed_motors.cpp
*
* Created: 03/31/2014, by Kyle McGahee
****************************************************************************************/

#ifndef PAIRED_DC_BRUSHED_MOTORS_INCLUDED_H
#define PAIRED_DC_BRUSHED_MOTORS_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "dc_brushed_motor.h"
#include "paired_motors_interface.h"
#include "stm32f4xx.h"

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
* Class: PairedDCBrushedMotors
*
* Description: Put class description here.
******************************************************************************/
class PairedDCBrushedMotors : public IPairedMotors
{
public: // methods

    // Constructor
    PairedDCBrushedMotors
        (
            float wheel_diameter,                // Centimeters
            float encoder_counts_per_revolution, // How many encoder counts make up one wheel rotation.
            float wheel_2_wheel_distance,        // AKA wheel base.  In centimeters.
            float angular_rate_per_duty          // Degrees / sec for a given % duty cycle.
        );

    // Must be called before using.
    void Initialize(void);

    // Clears motor control fields and re-init both motors.
    void ReInitialize(void);

    // Set both motors to drive at specified speed and resets current encoder count.
    // If speed is negative then motors will drive in reverse.
    void Drive
        (
            float new_speed // cm / sec
        );

    // Set right motor to drive at speed.  If speed is negative then motor will drive in reverse.
    // Note: Current encoder count is NOT reset when calling this function.
    void DriveRight
        (
            float new_speed // cm / sec
        );

    // Set left motor to drive at speed.  If speed is negative then motor will drive in reverse.
    // Note: Current encoder count is NOT reset when calling this function.
    void DriveLeft
        (
            float new_speed // cm / sec
        );

    // Turn the robot the correct 'angle' in degrees in the direction specified using a
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

    // Getters (all distances in centimeters)
    float get_right_motor_current_distance(void);
    float get_left_motor_current_distance(void);
    float get_right_motor_total_distance(void);
    float get_left_motor_total_distance(void);
    float get_wheel_base_distance(void) { return wheel_2_wheel_distance; }

private: // methods

    void InitializeEncoders(void);

    // Returns duty cycle corresponding to specified linear speed in cm/sec.
    float ConvertSpeedToDutyCycle
        (
            float speed // cm / sec
        );

private: // fields

    DCBrushedMotor * right_motor;
    DCBrushedMotor * left_motor;

    float wheel_diameter;         // Centimeters
    float wheel_circumference;    // Centimeters
    float wheel_2_wheel_distance; // AKA wheel base.  In centimeters.
    float angular_rate_per_duty;  // Degrees / sec for a given % duty cycle.

    float encoder_counts_per_revolution; // How many encoder counts make up one wheel rotation.

    float cm_per_encoder_count; // How many centimeters are travelled each count of the encoder.

    float cm_per_degree_zero_point; // How many centimeters to drive to turn 1 degree in a zero point turn.

}; // PairedDCBrushedMotors

#endif // PAIRED_DC_BRUSHED_MOTORS_INCLUDED_H
