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
#include "motor_interface.h"
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

/******************************************************************************
* Class: DCBrushedMotor
*
* Description: Represent a single dc brushed motor with encoder feedback.
******************************************************************************/
class DCBrushedMotor : public IMotor
{
public: // methods

    // Constructor
    DCBrushedMotor
        (
            AlternateFunctionPin & pwm_pin,
            OutputPin            & direction_pin,
            OutputPWMTimer       & pwm_timer,
            pwm_channel_t          pwm_channel,
            float                  wheel_diameter,                // Centimeters
            float                  encoder_counts_per_revolution, // How many encoder counts make up one wheel rotation.
            float                  angular_rate_per_duty          // Degrees / sec for a given % duty cycle.
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

    // Commands motor to come to a complete stop.
    void Stop(void);

    // Returns true if motor is completely stopped.
    bool IsStopped(void);

    // Increase encoder counts by 1. Should only be called by ISR.
    void IncrementEncoderTicks(void);

    // Getters (all distances in centimeters)
    float get_commanded_velocity(void);
    float get_current_distance(void) { return current_encoder_counts * cm_per_encoder_tick; }
    float get_total_distance(void) { return total_encoder_counts * cm_per_encoder_tick; }

    // Trivial setters
    void reset_current_distance(void) { current_encoder_counts = 0; }
    void reset_total_distance(void) { total_encoder_counts = 0; }

private: // methods

    inline void SetSpeed
        (
            float new_speed // Should be positive (cm / sec)
        );

    float ConvertSpeedToDutyCycle
        (
            float speed // Must be positive. (cm / sec)
        );

private: // fields

    AlternateFunctionPin & pwm_pin;

    OutputPin & direction_pin;

    OutputPWMTimer & pwm_timer;
    pwm_channel_t    pwm_channel;

    float wheel_diameter;      // Centimeters
    float wheel_circumference; // Centimeters

    float angular_rate_per_duty; // Degrees / sec for a given % duty cycle.

    float encoder_counts_per_revolution; // How many encoder counts make up one wheel rotation.

    float cm_per_encoder_tick; // How many centimeters are travelled each count of the encoder.

    float current_speed; // Current commanded speed in cm/sec.

    bool update_total_encoder_counts; // If true then will update total encoder counts every time IC interrupt is called.

    uint64_t total_encoder_counts;   // Number of encoder counts since initialization.
    uint32_t current_encoder_counts; // Number of encoder counts since last counter reset.

    motor_direction_t current_direction; // Direction motor is currently spinning.

}; // DCBrushedMotor

#endif // DC_BRUSHED_MOTOR_INCLUDED_H
