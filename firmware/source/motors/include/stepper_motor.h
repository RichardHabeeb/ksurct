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
#include "motor_interface.h"
#include "stm32f4xx.h"

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

// Acceleration function pointer type
typedef void (*accelerator_t)(int32_t acceleration);

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: StepperMotor
*
* Description: Represent a single stepper motor without encoder feedback.
******************************************************************************/
class StepperMotor : public IMotor
{
public: // methods

    // Constructor
    StepperMotor
        (
            OutputPin     & step_pin,
            OutputPin     & direction_pin,
            OutputPin     & micro_select_1_pin,         // Controls microstepping mode.
            OutputPin     & micro_select_2_pin,         // Controls microstepping mode.
            OutputPin     & micro_select_3_pin,         // Controls microstepping mode.
            float           wheel_diameter,             // In centimeters.
            float           full_steps_per_revolution,  // How many full steps needed to rotate wheel once.
            uint8_t         pulses_per_step,            // Ie 2 for half stepping, 16 for 1/16th stepping, etc.
            bool            acceleration_enabled,       // Whether or not to use acceleration logic.
            float           acceleration_time_constant, // How long to accelerate to target speed. (seconds)
            float           velocity_update_inc,        // How many full steps to update by when accelerating.
            accelerator_t   accel_function_pointer      // Acceleration callback function used to change ISR frequency.
        );

    // Must be called before using.
    void Initialize(void);

    // Clears all stateful information and returns to it's original 'initialized' state.
    void ReInitialize(void);

    // Set motor to drive at specified forward velocity. If velocity is negative then motor will drive in reverse.
    void Drive
        (
            float new_velocity // cm / sec
        );

    // Change current commanded velocity by the given amount. Can be either positive or negative.
    void AddVelocity
        (
            float velocity_change // cm / sec
        );

    // Sets direction to either forward or backwards without changing desired motor speed.
    void SetDirection
        (
            motor_direction_t new_direction
        );

    // Commands motor to come to a complete stop.  If acceleration mode is enabled then
    // will deaccelerate.
    void Stop(void);

    // Returns true if motor is completely stopped.
    bool IsStopped(void);

    // Takes one step (or microstep). Always updates current steps and if configured will
    // also update total steps.
    // **Should only be called by stepping ISR.
    void Step(void);

    // Updates current speed for motor to reflect target speed. Then if there is some
    // non-zero speed step the motor.  The new frequency needed to realize the new
    // current speed is returned.
    uint32_t UpdateSpeed(void);

    // Getters (all distances in centimeters)
    float get_commanded_velocity(void);
    float get_current_distance(void)   { return current_steps * cm_per_step; }
    float get_total_distance(void)     { return total_steps * cm_per_step; }

    // Trivial setters
    void reset_current_distance(void) { current_steps = 0; }
    void reset_total_distance(void)   { total_steps = 0; }

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

    // Motor control pins.
    OutputPin & step_pin;
    OutputPin & direction_pin;
    OutputPin & micro_select_1_pin;
    OutputPin & micro_select_2_pin;
    OutputPin & micro_select_3_pin;

    // Used as a callback to set acceleration ISR frequency.
    accelerator_t accel_function_pointer;

    // Number of pulses that make up one full step ie 2 for half-stepping, 4 for quarter stepping, etc
    uint8_t pulses_per_step;

    // If true then will acceleration (and deaccelerate) to reach target speed.
    bool acceleration_enabled;

    // If true then will update total steps every time step() is called.
    bool update_total_steps;

    float wheel_diameter;      // cm
    float wheel_circumference; // cm

    // Number of steps need for the wheel to make one full rotation.
    float full_steps_per_revolution;

    // Conversion constants.
    float full_steps_per_cm;
    float cm_per_full_step;
    float steps_per_cm;
    float cm_per_step;

    // The amount of time to take to accelerate from current speed to target speed (or de-accelerate)
    float acceleration_time_constant; // seconds

    // How precise the updating of velocity will be when accelerating.
    // The higher the number the less will interrupt but will be choppier.
    float velocity_update_inc;

    uint32_t current_speed; // Full steps per second
    uint32_t target_speed;  // Full steps per second
    float    acceleration;  // Full steps per seconds squared
    int64_t  total_steps;   // Number of steps since initialization (NOT necessarily full steps)
    int32_t  current_steps; // Number of steps since last speed change (NOT necessarily full steps)

    motor_direction_t current_direction; // Direction motor is currently spinning.

}; // StepperMotor

#endif // STEPPER_MOTOR_INCLUDED_H
