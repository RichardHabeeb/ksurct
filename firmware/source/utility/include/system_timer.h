/****************************************************************************************
* File: system_timer.h
*
* Description: Header file for system_timer.cpp
*
* Created: 02/23/2014, by Kyle McGahee
****************************************************************************************/

#ifndef SYSTEM_TIMER_INCLUDED_H
#define SYSTEM_TIMER_INCLUDED_H

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

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: SystemTimer
*
* Description: Allows precise timing using the core CMSIS system timer.  Right
*              now the number of ticks it can count is only a 32-bit integer
*              because I'm not sure if making it 64-bit would introduce a
*              race condition due to non-atomic operation when incrementing.
*              For a frequency of 1MHz (ie micro-second precision) this will
*              reset the time after 70 minutes.
******************************************************************************/
class SystemTimer
{
public: // methods

    // Resets time to 0 seconds and sets up sys-tick interrupt to run at desired frequency.
    // See class description for caution when selecting frequency.
    bool Initialize
        (
            float frequency // Desired frequency of timer. Must be from 1 to SystemCoreClock.
        );

    // Should only be called by sys-tick interrupt.
    inline void increment_ticks(void) { ++ticks; }

    inline double get_time(void) { return (double)ticks / (double)frequency; }

private: // fields

    // Number of times timer counter has ticked.
    // See class comment for reasoning on why it's only 32 bits.
    uint32_t ticks;

    // Frequency of systick timer.
    float frequency;

}; // SystemTimer

#endif  // SYSTEM_TIMER_INCLUDED_H
