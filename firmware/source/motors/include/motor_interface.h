/****************************************************************************************
* File: motor_interface.h
*
* Description: Interface to control a motor.
*
* Created: 4/20/2014, by Kyle McGahee
****************************************************************************************/

#ifndef MOTOR_INTERFACE_INCLUDED_H
#define MOTOR_INTERFACE_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

// Defines relative movement of motor in a longitudinal direction.
typedef enum
{
    drive_backward,
    drive_forward
} motor_direction_t;

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: IMotor
*
* Description: Interface to control a motor.
******************************************************************************/
class IMotor
{
public: // methods

    // Must be called before using.
    virtual void Initialize(void) = 0;

    // Clears all stateful information and returns to it's original 'initialized' state.
    virtual void ReInitialize(void) = 0;

    // Set motor to drive at specified forward velocity. If velocity is negative then motor will drive in reverse.
    virtual void Drive
        (
            float new_velocity // cm / sec
        ) = 0;

    // Change current commanded velocity by the given amount. Can be either positive or negative.
    virtual void AddVelocity
        (
            float velocity_change // cm / sec
        ) = 0;

    // Sets direction to either forward or backwards without changing desired motor speed.
    virtual void SetDirection
        (
            motor_direction_t new_direction
        ) = 0;

    // Commands motor to stop and will wait until it is.
    virtual void Stop(void) = 0;

    // Returns true only if motor is stopped.
    virtual bool IsStopped(void) = 0;

    // Getters (all distances in centimeters)
    virtual float get_commanded_velocity(void) = 0;
    virtual float get_current_distance(void) = 0;
    virtual float get_total_distance(void) = 0;

    // Trivial setters
    virtual void reset_current_distance(void) = 0;
    virtual void reset_total_distance(void) = 0;

}; // IMotor

#endif // MOTOR_INTERFACE_INCLUDED_H
