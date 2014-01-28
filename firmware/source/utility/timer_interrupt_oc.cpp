/****************************************************************************************
* File: timer_interrupt_oc.cpp - Timer Interrupt Output Compare
*
* Description:
*
* Created: 01/17/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>

#include "stm32f4xx.h"
#include "timer_interrupt_oc.h"

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
*                                    STATIC FIELDS
*--------------------------------------------------------------------------------------*/

oc_channel_info_t TimerInterruptOC::tim3[number_oc_channels];

/*---------------------------------------------------------------------------------------
*                                    CLASS METHODS
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: TimerInterruptOC - Constructor
*
* Description:
*****************************************************************************/
TimerInterruptOC::TimerInterruptOC
    (
        TIM_TypeDef * timer, // TIMx associated with channel.  Will be initialized inside class.
        IRQn          timer_irq_number, // Needed to setup timer interrupt.
        uint32_t      timer_clock,  // RCC_APB1Periph_TIMx
        uint32_t      timer_clock_freq, // Base frequency of timer clock.
        uint32_t      timer_clock_counter // Scaled down frequency of timer clock.
    )
{
    this->timer = timer;
    this->timer_irq_number = timer_irq_number;
    this->timer_clock = timer_clock;
    this->timer_clock_freq = timer_clock_freq;
    this->timer_clock_counter = timer_clock_counter;

    timer_initialized = false;

} // TimerInterruptOC::TimerInterruptOC() - Constructor


/*****************************************************************************
* Function: InitChannel
*
* Description:
*****************************************************************************/
void TimerInterruptOC::InitChannel
    (
        oc_channel_t     channel,
        float            frequency,
        timer_callback_t callback
    )
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    if (!timer_initialized)
    {
        SetupTimer();
        timer_initialized = true;
    }

    uint16_t capture_compare_value = 0;

    if (frequency > 0.0f)
    {
        capture_compare_value = (uint16_t) (this->timer_clock_counter / frequency);
    }

    // Setup general channel settings.
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = capture_compare_value;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    // Setup specific channel settings.
    switch (channel)
    {
        case oc_channel_1:
            TIM_OC1Init(this->timer, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(this->timer, TIM_OCPreload_Disable);
            break;
        case oc_channel_2:
            TIM_OC2Init(this->timer, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(this->timer, TIM_OCPreload_Disable);
            break;
        case oc_channel_3:
            TIM_OC3Init(this->timer, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(this->timer, TIM_OCPreload_Disable);
            break;
        case oc_channel_4:
            TIM_OC4Init(this->timer, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(this->timer, TIM_OCPreload_Disable);
            break;
    }

    // Right now just supports timer 3.  Might make more generic if I have time.
    if (timer == TIM3)
    {
        tim3[channel].callback = callback;
    }

    SetChannelFrequency(channel, frequency);

    TIM_Cmd(this->timer, ENABLE);

} // TimerInterruptOC::InitChannel()

/*****************************************************************************
* Function: SetChannelFrequency
*
* Description:
*****************************************************************************/
void TimerInterruptOC::SetChannelFrequency
    (
        oc_channel_t channel,
        float        new_frequency
    )
{
    if (new_frequency <= 0.0f)
    {
        SetChannelState(channel, DISABLE);
        TimerInterruptOC::tim3[channel].frequency = 0;
        TimerInterruptOC::tim3[channel].capture_compare_value = 0;
        return;
    }

    uint16_t capture_compare_value = (uint16_t) (this->timer_clock_counter / new_frequency);

    switch (channel)
    {
        case oc_channel_1:
            timer->CCR1 = timer->CNT + capture_compare_value;
            break;
        case oc_channel_2:
            timer->CCR2 = timer->CNT + capture_compare_value;
            break;
        case oc_channel_3:
            timer->CCR3 = timer->CNT + capture_compare_value;
            break;
        case oc_channel_4:
            timer->CCR4 = timer->CNT + capture_compare_value;
            break;
    }

    // Right now just supports timer 3.  Might make more generic if I have time.
    TimerInterruptOC::tim3[channel].frequency = (uint32_t)new_frequency;
    TimerInterruptOC::tim3[channel].capture_compare_value = capture_compare_value;

    // In case it was disabled.
    SetChannelState(channel, ENABLE);

} // TimerInterruptOC::SetChannelFrequency()

/*****************************************************************************
* Function: SetupTimer
*
* Description:
*****************************************************************************/
void TimerInterruptOC::SetupTimer(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Clock enable
    RCC_APB1PeriphClockCmd(this->timer_clock, ENABLE);

    // Enable timer global interrupt
    NVIC_InitStructure.NVIC_IRQChannel = this->timer_irq_number;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Compute the prescaler value
    uint16_t prescaler = (uint16_t) (this->timer_clock_freq / this->timer_clock_counter) - 1;

    // Timebase configuration
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(this->timer, &TIM_TimeBaseStructure);

    // Prescaler configuration
    TIM_PrescalerConfig(this->timer, prescaler, TIM_PSCReloadMode_Immediate);

} // TimerInterruptOC::SetupTimer()

/*****************************************************************************
* Function: SetChannelState
*
* Description:
*****************************************************************************/
void TimerInterruptOC::SetChannelState
    (
        oc_channel_t    channel,
        FunctionalState state
    )
{
    switch (channel)
    {
        case oc_channel_1:
            TIM_ITConfig(this->timer, TIM_IT_CC1, state);
            break;
        case oc_channel_2:
            TIM_ITConfig(this->timer, TIM_IT_CC2, state);
            break;
        case oc_channel_3:
            TIM_ITConfig(this->timer, TIM_IT_CC3, state);
            break;
        case oc_channel_4:
            TIM_ITConfig(this->timer, TIM_IT_CC4, state);
            break;
    }

} // TimerInterruptOC::SetChannelState()

/*****************************************************************************
* Function: TIM3_IRQHandler
*
* Description:
*****************************************************************************/
extern "C" void TIM3_IRQHandler(void)
{
    uint32_t capture = 0;

    oc_channel_info_t * channel = NULL;

    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {
        channel = &TimerInterruptOC::tim3[oc_channel_1];
        TIM3->SR = (uint16_t)~TIM_IT_CC1; // Clear the interrupt pending bit
        capture = TIM3->CCR1; // Get the capture 1 register value
        TIM3->CCR1 = capture + channel->capture_compare_value; // Set the capture compare 1 register value
        channel->callback();
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
    {
        channel = &TimerInterruptOC::tim3[oc_channel_2];
        TIM3->SR = (uint16_t)~TIM_IT_CC2; // Clear the interrupt pending bit
        capture = TIM3->CCR2; // Get the capture 2 register value
        TIM3->CCR2 = capture + channel->capture_compare_value; // Set the capture compare 2 register value
        channel->callback();
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
    {
        channel = &TimerInterruptOC::tim3[oc_channel_3];
        TIM3->SR = (uint16_t)~TIM_IT_CC3; // Clear the interrupt pending bit
        capture = TIM3->CCR3; // Get the capture 3 register value
        TIM3->CCR3 = capture + channel->capture_compare_value; // Set the capture compare 3 register value
        channel->callback();
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)
    {
        channel = &TimerInterruptOC::tim3[oc_channel_4];
        TIM3->SR = (uint16_t)~TIM_IT_CC4; // Clear the interrupt pending bit
        capture = TIM3->CCR4; // Get the capture 4 register value
        TIM3->CCR4 = capture + channel->capture_compare_value; // Set the capture compare 4 register value
        channel->callback();
    }

} // TIM3_IRQHandler()
