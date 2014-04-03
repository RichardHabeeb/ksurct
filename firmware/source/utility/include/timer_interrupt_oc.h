/****************************************************************************************
* File: timer_interrupt_oc.h
*
* Description:
*
* Created: 01/17/2014, by Kyle McGahee
****************************************************************************************/

#ifndef TIMER_INTERRUPT_OC_INCLUDED_H
#define TIMER_INTERRUPT_OC_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "stm32f4xx.h"

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

// Output compare channel select type.
typedef uint8_t oc_channel_t;
enum
{
    oc_channel_1,
    oc_channel_2,
    oc_channel_3,
    oc_channel_4,
    number_oc_channels
};

// Callback interrupt function pointer type.
typedef void (*timer_callback_t)(void);

typedef struct
{
    uint32_t frequency;
    timer_callback_t callback;
    uint16_t capture_compare_value;
} oc_channel_info_t;

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: TimerInterruptOC
*
* Description: Put class description here.
******************************************************************************/
class TimerInterruptOC
{
public: // methods

    // Constructor
    TimerInterruptOC
        (
            TIM_TypeDef * timer,              // TIMx associated with channel.  Will be initialized inside class.
            IRQn          timer_irq_number,   // Needed to setup timer interrupt.
            uint32_t      timer_clock,        // RCC_APB1Periph_TIMx
            uint32_t      timer_clock_freq,   // Base frequency of timer clock.
            uint32_t      timer_clock_counter // Scaled down frequency of timer clock.
        );

    void InitChannel
        (
            oc_channel_t     channel,
            float            frequency,
            timer_callback_t callback
        );

    void SetChannelFrequency
        (
            oc_channel_t channel,
            float        new_frequency
        );

public: // fields

    static oc_channel_info_t tim3[number_oc_channels];
    static oc_channel_info_t tim4[number_oc_channels];

private: // methods

    void SetupTimer(void);

    void SetChannelState
    (
        oc_channel_t    channel,
        FunctionalState state
    );

private: // fields

    TIM_TypeDef * timer;
    IRQn          timer_irq_number;
    uint32_t      timer_clock;
    uint32_t      timer_clock_freq;
    uint32_t      timer_clock_counter;
    bool          timer_initialized;

}; // TimerInterruptOC

#endif  // TIMER_INTERRUPT_OC_INCLUDED_H
