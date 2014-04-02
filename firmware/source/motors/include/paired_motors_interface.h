/****************************************************************************************
* File: paired_motors_interface.h
*
* Description: Interface to control two differentially paired motors.
*
* Created: 4/01/2014, by Kyle McGahee
****************************************************************************************/

#ifndef PAIRED_MOTORS_INTERFACE_INCLUDED_H
#define PAIRED_MOTORS_INTERFACE_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

// Defines relative movement of robot in a longitudinal direction.
typedef enum
{
    drive_backward,
    drive_forward
} motor_direction_t;

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
* Class: IPairedMotors
*
* Description: Interface to control two differentially paired motors.
******************************************************************************/
class IPairedMotors
{
public: // methods

    // Must be called before using.
    virtual void Initialize(void) = 0;

    // Clears motor control fields and re-init both motors.
    virtual void ReInitialize(void) = 0;

    // Set both motors to drive at specified speed and current distance travelled.
    // If speed is negative then motors will drive in reverse.
    virtual void Drive
        (
            float new_speed // cm / sec
        ) = 0;

    // Set right motor to drive at speed.  If speed is negative then motor will drive in reverse.
    // Note: Current distance travelled is NOT reset when calling this function.
    virtual void DriveRight
        (
            float new_speed // cm / sec
        ) = 0;

    // Set left motor to drive at speed.  If speed is negative then motor will drive in reverse.
    // Note: Current distance travelled is NOT reset when calling this function.
    virtual void DriveLeft
        (
            float new_speed // cm / sec
        ) = 0;

    // Turn the robot the correct 'angle' in degrees in the direction specified using a
    // zero point turn approach.
    virtual void ZeroPointTurn
        (
            motor_turn_t  new_direction,  // Which direction to turn.
            float         turn_angle,     // How far to turn in degrees
            float         turn_speed      // How fast to turn in degrees per second.
        ) = 0;

    // Commands both motors to stop and will wait until they are.
    virtual void Stop(void) = 0;

    // Returns true only if both motors are stopped.
    virtual bool inline Stopped(void) = 0;

    // Getters (all distances in centimeters)
    virtual float get_right_motor_current_distance(void) = 0;
    virtual float get_left_motor_current_distance(void) = 0;
    virtual float get_right_motor_total_distance(void) = 0;
    virtual float get_left_motor_total_distance(void) = 0;
    virtual float get_wheel_base_distance(void) = 0;

}; // IPairedMotors

#endif // PAIRED_MOTORS_INTERFACE_INCLUDED_H
