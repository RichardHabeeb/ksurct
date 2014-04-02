/****************************************************************************************
* File: output_pwm_timer.cpp - Output Pulse Width Modulation Timer
*
* Description: Allows channels of a timer to be setup to output a PWM signal.
*
* Created: 03/24/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>

#include "gpio.h"
#include "output_pwm_timer.h"
#include "stm32f4xx.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                   MEMORY CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                    CLASS METHODS
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: OutputPWMTimer
*
* Description: Constructor
*****************************************************************************/
OutputPWMTimer::OutputPWMTimer
    (
        TIM_TypeDef * timer,                  // TIMx associated with channel.  Will be initialized inside class.
        uint32_t      timer_clock,            // RCC_APB1Periph_TIMx
        uint32_t      timer_base_frequency,   // Base frequency of timer clock.
        uint32_t      timer_scaled_frequency, // Scaled down frequency of timer clock.
        uint32_t      output_frequency        // Desired frequency of OutputPWMTimer signal.
    )
{
    this->timer = timer;
    this->timer_clock = timer_clock;
    this->timer_base_frequency = timer_base_frequency;
    this->timer_scaled_frequency = timer_scaled_frequency;
    this->output_frequency = output_frequency;

    timer_initialized = false;

} // OutputPWMTimer::OutputPWMTimer() - Constructor

/*****************************************************************************
* Function: InitChannel
*
* Description: Sets up timer channel to run at timer output frequency.  It will initially
*              have a 0% duty cycle.  Must be called before using a channel.
*****************************************************************************/
void OutputPWMTimer::InitChannel
    (
        pwm_channel_t          channel,  // Channel to initialize.
        AlternateFunctionPin & pwm_pin   // Pin already setup with desired pin remapping.
    )
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    // Map channel to specified pin.
    pwm_pin.Init();

    if (!timer_initialized)
    {
        SetupTimer();
        timer_initialized = true;
    }

    // Setup general channel settings.
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    // Setup specific channel settings.
    switch (channel)
    {
        case pwm_channel_1:
            TIM_OC1Init(this->timer, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(this->timer, TIM_OCPreload_Enable);
            break;
        case pwm_channel_2:
            TIM_OC2Init(this->timer, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(this->timer, TIM_OCPreload_Enable);
            break;
        case pwm_channel_3:
            TIM_OC3Init(this->timer, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(this->timer, TIM_OCPreload_Enable);
            break;
        case pwm_channel_4:
            TIM_OC4Init(this->timer, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(this->timer, TIM_OCPreload_Enable);
            break;
    }

    SetChannelPulseDuration(channel, 0);

    TIM_ARRPreloadConfig(this->timer, ENABLE);

    TIM_Cmd(this->timer, ENABLE);

} // OutputPWMTimer::InitChannel()

/*****************************************************************************
* Function: SetChannelDutyCycle
*
* Description: Set what percentage of the pulse is high.
*****************************************************************************/
void OutputPWMTimer::SetChannelDutyCycle
    (
        pwm_channel_t channel,
        float         new_duty_cycle
    )
{
    uint32_t capture_compare_value = (uint32_t)(new_duty_cycle * timer->ARR);

    switch (channel)
    {
        case pwm_channel_1:
            timer->CCR1 = capture_compare_value;
            break;
        case pwm_channel_2:
            timer->CCR2 = capture_compare_value;
            break;
        case pwm_channel_3:
            timer->CCR3 = capture_compare_value;
            break;
        case pwm_channel_4:
            timer->CCR4 = capture_compare_value;
            break;
    }

} // OutputPWMTimer::SetChannelDutyCycle()

/*****************************************************************************
* Function: SetChannelPulseDuration
*
* Description: Set how long (in microseconds) the pulse is high.
*****************************************************************************/
void OutputPWMTimer::SetChannelPulseDuration
    (
        pwm_channel_t channel,            // Channel to update.
        float         new_pulse_duration  // In microseconds.
    )
{
    uint32_t capture_compare_value = (uint32_t)(new_pulse_duration * 1e-6 * timer_scaled_frequency);

    switch (channel)
    {
        case pwm_channel_1:
            timer->CCR1 = capture_compare_value;
            break;
        case pwm_channel_2:
            timer->CCR2 = capture_compare_value;
            break;
        case pwm_channel_3:
            timer->CCR3 = capture_compare_value;
            break;
        case pwm_channel_4:
            timer->CCR4 = capture_compare_value;
            break;
    }

} // OutputPWMTimer::SetChannelPulseDuration()

/*****************************************************************************
* Function: SetupTimer
*
* Description: Prescales timer down and sets output frequency of pwm signal.
*****************************************************************************/
void OutputPWMTimer::SetupTimer(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // Clock enable
    RCC_APB1PeriphClockCmd(this->timer_clock, ENABLE);

    // Compute the prescaler value
    uint16_t prescaler = (uint16_t)(this->timer_base_frequency / this->timer_scaled_frequency) - 1;

    // Timebase configuration
    TIM_TimeBaseStructure.TIM_Period = (this->timer_scaled_frequency / this->output_frequency) - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(this->timer, &TIM_TimeBaseStructure);

    // Prescaler configuration
    TIM_PrescalerConfig(this->timer, prescaler, TIM_PSCReloadMode_Immediate);

} // OutputPWMTimer::SetupTimer()
