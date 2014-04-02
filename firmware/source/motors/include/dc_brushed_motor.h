/****************************************************************************************
* File: dc_brushed_motor.h
*
* Description: Header file for dc_brushed_motor.cpp
*
* Created: 4/3/2014, by Kyle McGahee
****************************************************************************************/

#ifndef DC_BRUSHED_MOTOR_INCLUDED_H
#define DC_BRUSHED_MOTOR_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <stdint.h>

#include "gpio.h"
#include "paired_motors_interface.h"
#include "output_pwm_timer.h"

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

class DCBrushedMotor
{
public: // methods

    // Constructor
    DCBrushedMotor
        (
            OutputPWMTimer & pwm_timer,
            pwm_channel_t    pwm_channel,
            OutputPin      & direction_pin
        );

    // Clears any stateful fields such as all encoder counts, etc and stops motor.
    void ReInitialize(void);

    // Drives motor in either forward or reverse direction at the specified PWM value.
    // Provided speed should always be positive.
    void Drive
        (
            motor_direction_t new_direction,    // Either forwards or backwards.
            float             new_pwm_duty_cyle // Should be from 0 (off) to 100 (full speed)
        );

    // Sets direction to either forward or backwards without changing motor speed.
    inline void SetDirection
        (
            motor_direction_t new_direction
        );

    // Commands motor to come to a complete stop.
    void Stop(void);

    // Returns true if motor is completely stopped.
    bool IsStopped(void);

    // Increase encoder counts by 1. Should be called by ISR.
    void IncrementEncoderTicks(void);

    // Getters
    uint32_t get_current_encoder_counts(void) const { return current_encoder_counts; }

    uint64_t get_total_encoder_counts(void) const { return total_encoder_counts; }

    // Setters
    void set_update_total_encoder_counts(bool update) { update_total_encoder_counts = update; }

    void reset_current_encoder_counts(void) { current_encoder_counts = 0; }

private: // methods

    inline void SetDutyCycle
        (
            float new_duty_cycle // From 0 (off) to 100 (full speed)
        );

private: // fields

    OutputPin & direction_pin;

    OutputPWMTimer & pwm_timer;
    pwm_channel_t    pwm_channel;

    // Current output duty % of PWM signal from 0-100.
    float current_pwm_duty_cyle;

    // If true then will update total encoder counts every time IC interrupt is called.
    bool update_total_encoder_counts;

    uint64_t total_encoder_counts;   // Number of encoder counts since initialization.
    uint32_t current_encoder_counts; // Number of encoder counts since last counter reset.

}; // DCBrushedMotor

#endif // DC_BRUSHED_MOTOR_INCLUDED_H
