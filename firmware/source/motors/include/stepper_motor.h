/****************************************************************************************
* File: stepper_motor.h
*
* Description: Header file for stepper_motor.cpp
*
* Created: 1/13/2014, by Kyle McGahee
****************************************************************************************/

#ifndef STEPPER_MOTOR_INCLUDED_H
#define STEPPER_MOTOR_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "gpio.h"
#include "stm32f4xx.h"

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

// Motor direction type.
typedef int8_t motor_direction_t;
enum
{
    drive_backward,
    drive_forward
};

// Motor turn direction type.
typedef int8_t motor_turn_t;
enum
{
    turn_right,
    turn_left
};

// Acceleration function pointer type
typedef void (*accelerator_t)(int32_t acceleration);

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

class StepperMotor
{
public: // methods

    // Constructor
    StepperMotor
        (
            OutputPin &   step_pin,
            OutputPin &   direction_pin,
            uint8_t       pulses_per_step,            // Ie 2 for half stepping, 16 for 1/16th stepping, etc.
            bool          acceleration_enabled,       // Whether or not to use acceleration logic.
            float         acceleration_time_constant, // How long to accelerate to target speed. (seconds)
            float         velocity_update_inc,        // How many full steps to update by when accelerating.
            accelerator_t accel_function_pointer      // Acceleration callback function used to change ISR frequency.
        );

    // Clears any stateful fields such as total steps, etc and stops motor.
    void ReInitialize(void);

    // Drives stepper motor in either forward or reverse direction at specified speed.
    // Provided speed should always be positive.
    void Drive
        (
            motor_direction_t new_direction, // Either forwards or backwards.
            uint32_t          new_speed      // Full steps per second (should always be positive)
        );

    // Takes one step (or microstep). Always updates current steps and if configured will
    // also update total steps.
    void Step(void);

    // Commands motor to come to a complete stop.  If acceleration mode is enabled then
    // will deaccelerate.
    void Stop(void);

    // Returns true if motor is completely stopped.
    bool IsStopped(void);

    // Updates current speed for motor to reflect target speed. Then if there is some
    // non-zero speed step the motor.  The new frequency needed to realize the new
    // current speed is returned.
    uint32_t UpdateSpeed(void);

    // Sets direction to either forward or backwards without changing motor speed.
    inline void SetDirection
        (
            motor_direction_t new_direction
        );

    // Getters
    uint32_t get_current_full_steps(void) const { return (uint32_t)((float)current_steps / (float)pulses_per_step); }

    uint64_t get_total_full_steps(void) const { return (uint32_t)((float)total_steps / (float)pulses_per_step); }

    uint32_t get_current_speed(void) const { return current_speed; }

    // Setters
    void set_acceleration(bool enabled) { acceleration_enabled = enabled; }

    void set_update_total_steps(bool update) { update_total_steps = update; }

    void reset_current_steps(void) { current_steps = 0; }

private: // methods

    // Updates the speed the motor is trying to reach.  If acceleration is enabled in the
    // motor control then this speed will be instantly realized next interrupt service routine.
    void SetTargetSpeed
        (
            uint32_t new_speed // Full steps per second.
        );

    // When acceleration is enabled this helper method is used to update the current speed
    // by some incremental value.
    inline void UpdateCurrentSpeedWithAcceleration(void);

    // Returns true if we go outside bounds of target speed.
    inline bool OvershotTargetSpeed(void);

private: // fields

    // Control pins provided by client.
    OutputPin & step_pin;
    OutputPin & direction_pin;

    // Used as a callback to set acceleration ISR frequency.
    accelerator_t accel_function_pointer;

    // Number of pulses that make up one full step ie 2 for half-stepping, 4 for quarter stepping, etc
    uint8_t pulses_per_step;

    // If true then will acceleration (and deaccelerate) to reach target speed.
    bool acceleration_enabled;

    // If true then will update total steps every time step() is called.
    bool update_total_steps;

    // The amount of time to take to accelerate from current speed to target speed (or de-accelerate)
    float acceleration_time_constant; // seconds

    // How precise the updating of velocity will be when accelerating.
    // The higher the number the less will interrupt but will be choppier.
    float velocity_update_inc;

    uint32_t current_speed; // Full steps per second
    uint32_t target_speed;  // Full steps per second
    float    acceleration;  // Full steps per seconds squared
    uint64_t total_steps;   // Number of steps since initialization (NOT necessarily full steps)
    uint32_t current_steps; // Number of steps since last speed change (NOT necessarily full steps)

}; // StepperMotor

#endif // STEPPER_MOTOR_INCLUDED_H
