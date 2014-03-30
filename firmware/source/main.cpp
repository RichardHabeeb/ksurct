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
#include "simulated_ir_sensors.h"
#include "system_timer.h"
#include "test_maze_creator.h"
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
    uint32_t number_of_rows    = 16;
    uint32_t number_of_columns = 16;
    float    cell_length       = 18.0f;

    //Maze * maze = new Maze(number_of_rows, number_of_columns, cell_length);
    Maze * maze = TestMazeCreator().CreateMaze(number_of_rows, number_of_columns, cell_length);
    maze->set_starting_cell(0, 0);
    maze->set_goal_cell(number_of_rows/2, number_of_columns/2);

    if (maze == NULL) { return 1; }

    static SimpleFloodFill simple_flood_fill(maze);

    static DifferentialPairedStepperMotors motors(false, // Acceleration enabled
                                                  180,  // Turn speed (degrees per second)
                                                  6,    // Wheel diameter (centimeters)
                                                  11,   // Wheel base (centimeters)
                                                  200,  // Full steps per revolution
                                                  16);  // Pulses per full steps (for microstepping)

    // TODO: Figure out gains / limits
    static PID centering_controller(0,    // Proportional Gain
                                    0,    // Integral Gain
                                    0,    // Derivative Gain
                                    10,   // Upper Output Saturation Limit
                                    -10,  // Lower Output Saturation Limit
                                    5,    // Upper Error Integral Saturation Limit
                                    -5);  // Lower Error Integral Saturation Limit


    wall_threshold_t wall_thresholds;
    wall_thresholds.front = (maze->get_cell_length() / 2.f) + 5.f;
    wall_thresholds.side  = (maze->get_cell_length() / 2.f) + 5.f;

    static SimulatedIRSensors ir_sensors;

    static Micromouse micromouse(*maze, simple_flood_fill, ir_sensors, motors, centering_controller, wall_thresholds, 18.f);

    // Just do this if using simulated ir sensors so it knows what distances to return.
    ir_sensors.set_maze(*maze);
    ir_sensors.set_micromouse(micromouse);

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
