/****************************************************************************************
* File: config_distance_sensors.cpp
*
* Description: Creates correct distance sensors to use based on configuration settings.
*
* Created: 04/20/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <stddef.h>

#include "config_settings.h"
#include "distance_sensors_interface.h"
#include "ir_sensors.h"
#include "private_system_config.h"
#include "simulated_ir_sensors.h"
#include "test_maze_creator.h"
#include "util_assert.h"

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
                                    MEMORY CONSTANTS
---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: configure_distance_sensors
*
* Description: Configures correct distance sensors to use based on configuration settings.
*****************************************************************************/
IDistanceSensors * configure_distance_sensors(void)
{
    IDistanceSensors * distance_sensors = NULL;
    IRsensor_offset_t offsets;

    if (USE_SIMULATED_SENSORS)
    {
        distance_sensors = new SimulatedIRSensors();
    }
    else
    {
        if (SENSOR_SIDE_TYPE == TWO_SIDED_SENSOR_TYPE)
        {
            distance_sensors = new IRSensors(POWERLION, offsets);
        }
        else if (SENSOR_SIDE_TYPE == ONE_SIDED_SENSOR_TYPE)
        {
            distance_sensors = new IRSensors(BABY_KITTEN, offsets);
        }
    }

    assert(distance_sensors != NULL, ASSERT_STOP);

    return distance_sensors;

} // configure_distance_sensors()

/*****************************************************************************
* Function: post_setup_simulated_sensors
*
* Description: Performs setup of simulated sensors after the necessary components
*              have been created.
*****************************************************************************/
void post_setup_simulated_sensors
    (
        IDistanceSensors * distance_sensors, // Already instantiated simulated distance sensors.
        Maze             * maze,             // Already configured maze (simulated or real)
        Micromouse       * micromouse        // Already configured micromouse
    )
{
    // Just do this if using simulated ir sensors so it knows what distances to return.
    if (USE_SIMULATED_SENSORS)
    {
        // If using simulated sensors then have to pass in a simulated maze.  If config
        // settings specify not to use a simulated maze then need to create one.
        Maze * simulated_maze = maze;
        if (!USE_TEST_MAZE)
        {
            simulated_maze = TestMazeCreator().CreateMaze(maze->get_number_rows(),
                                                          maze->get_number_columns(),
                                                          maze->get_cell_length());
        }
        ((SimulatedIRSensors*)(distance_sensors))->set_maze(*simulated_maze);
        ((SimulatedIRSensors*)(distance_sensors))->set_micromouse(*micromouse);
    }

} // post_setup_simulated_sensors()
