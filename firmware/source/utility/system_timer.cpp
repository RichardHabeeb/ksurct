/****************************************************************************************
* File: system_timer.cpp
*
* Description: Allows precise timing using the core CMSIS system timer.  Right
*              now the number of ticks it can count is only a 32-bit integer
*              because I'm not sure if making it 64-bit would introduce a
*              race condition due to non-atomic operation when incrementing.
*              For a frequency of 1MHz (ie micro-second precision) this will
*              reset the time after 70 minutes.
*
* Created: 02/23/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>

#include "stm32f4xx.h"
#include "system_timer.h"

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

extern SystemTimer system_timer;

/*****************************************************************************
* Function: Initialize
*
* Description:  Resets time to 0 seconds and sets up sys-tick interrupt to
*               run at desired frequency.
*****************************************************************************/
bool SystemTimer::Initialize
    (
        float frequency // Desired frequency of timer. Must be from 1 to SystemCoreClock.
    )
{
    if (frequency < 1.f || frequency > SystemCoreClock)
    {
        return false;
    }

    this->frequency = frequency;

    this->ticks = 0;

    SystemCoreClockUpdate();

    uint32_t ticks_between_interrupts = (uint32_t)(SystemCoreClock / frequency);
    SysTick_Config(ticks_between_interrupts);

    return true;

} // SystemTimer::Initialize()

/*****************************************************************************
* Function: SysTick_Handler
*
* Description: Interrupt that occurs every tick of system timer.
*****************************************************************************/
extern "C" void SysTick_Handler(void)
{
    system_timer.increment_ticks();

} // SysTick_Handler()
