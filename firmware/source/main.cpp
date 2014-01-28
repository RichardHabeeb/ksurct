/****************************************************************************************
* File: main.cpp
*
* Description: TODO
*
* Created: 12/2/2013, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <cstdio>

#include "stm32f4xx.h"
#include "differential_paired_stepper_motors.h"
#include "gpio.h"
#include "micromouse.h"
#include "pid.h"
#include "simplefloodfill.h"
#include "system_timer.h"
#include "timer_interrupt_oc.h"
#include "weightedpathfinding.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
                                    MEMORY CONSTANTS
---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      VARIABLES
*--------------------------------------------------------------------------------------*/

SystemTimer system_timer;

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: main
*
* Description: TODO
*****************************************************************************/
int main(void)
{
    static Maze maze(16, 16, 18.0f);

    static SimpleFloodFill simple_flood_fill(&maze);

    static DifferentialPairedStepperMotors motors(true, // Acceleration enabled
                                                  180,  // Turn speed (degrees per second)
                                                  6,    // Wheel diameter (centimeters)
                                                  11,   // Wheel base (centimeters)
                                                  200,  // Full steps per revolution
                                                  16);  // Pulses per full steps (for microstepping)

    // TODO: Figure out gains / limits
    static PID centering_controller(1,    // Proportional Gain
                                    0,    // Integral Gain
                                    1,    // Derivative Gain
                                    10,   // Upper Output Saturation Limit
                                    -10,  // Lower Output Saturation Limit
                                    5,    // Upper Error Integral Saturation Limit
                                    -5);  // Lower Error Integral Saturation Limit

    static wall_threshold_t wall_thresholds = { 12.f, 0.f, 12.f };

    Micromouse micromouse(maze, simple_flood_fill, motors, centering_controller, wall_thresholds, 18.f);

    motors.Initialize();

    system_timer.Initialize(1e6);

    while (true)
    {
        if (!micromouse.SolveMaze())
        {
            // assert
        }
    }

    // return 0; KLM: Remove warning: statement is unreachable.

} // main()
