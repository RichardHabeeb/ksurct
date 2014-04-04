/****************************************************************************************
* File: output_pwm_timer.h - Output Pulse Width Modulation Timer
*
* Description: Header file for output_pwm_timer.cpp
*
* Created: 03/24/2014, by Kyle McGahee
****************************************************************************************/

#ifndef OUTPUT_PWM_TIMER_INCLUDED_H
#define OUTPUT_PWM_TIMER_INCLUDED_H

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

// Timer channel select type.
typedef uint8_t pwm_channel_t;
enum
{
    pwm_channel_1,
    pwm_channel_2,
    pwm_channel_3,
    pwm_channel_4,
    number_pwm_channels
};

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: OutputPWMTimer
*
* Description: Allows channels of a timer to be setup to output a PWM signal.
******************************************************************************/
class OutputPWMTimer
{
public: // methods

    // Constructor
    OutputPWMTimer
        (
            TIM_TypeDef * timer,                  // TIMx associated with channel.  Will be initialized inside class.
            uint32_t      timer_clock,            // RCC_APB1Periph_TIMx
            uint32_t      timer_base_frequency,   // Base frequency of timer clock.
            uint32_t      timer_scaled_frequency, // Scaled down frequency of timer clock.
            uint32_t      output_frequency        // Desired frequency of PWM signal.
        );

    // Sets up timer channel to run at timer output frequency.  It will initially have a
    // 0% duty cycle.  Must be called before using a channel.
    void InitChannel
        (
            pwm_channel_t          channel,  // Channel to initialize.
            AlternateFunctionPin & pwm_pin   // Pin already setup with desired pin remapping.
        );

    // Set what percentage of the pulse is high.
    void SetChannelDutyCycle
        (
            pwm_channel_t channel,       // Channel to update.
            float         new_duty_cycle // Range 0 to 1 (equates to 0% - 100%)
        );

    // Set how long (in microseconds) the pulse is high.
    void SetChannelPulseDuration
        (
            pwm_channel_t channel,            // Channel to update.
            float         new_pulse_duration  // In microseconds.
        );

private: // methods

    // Prescales timer down and sets output frequency of pwm signal.
    void SetupTimer(void);

private: // fields

    TIM_TypeDef * timer;
    uint32_t      timer_clock;
    uint32_t      timer_base_frequency;
    uint32_t      timer_scaled_frequency;
    uint32_t      output_frequency;
    bool          timer_initialized;

}; // OutputPWMTimer

#endif  // OUTPUT_PWM_TIMER_INCLUDED_H
