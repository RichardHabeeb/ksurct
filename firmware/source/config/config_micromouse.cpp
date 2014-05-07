/****************************************************************************************
* File: config_micromouse.cpp
*
* Description: Creates correct micromouse to use based on configuration settings.
*
* Created: 04/20/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <stddef.h>

#include "config_settings.h"
#include "distance_sensors_interface.h"
#include "gpio.h"
#include "maze.h"
#include "micromouse.h"
#include "motor_interface.h"
#include "paired_motors.h"
#include "pid.h"
#include "private_system_config.h"
#include "simulated_ir_sensors.h"
#include "stm32f4xx.h"
#include "timer_interrupt_oc.h"

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
* Function: configure_micromouse
*
* Description: Configures correct micromouse to use based on configuration settings.
*****************************************************************************/
Micromouse * configure_micromouse(void)
{
    configure_leds();

    Maze * maze = configure_maze();

    IPathFinder * path_finder = configure_path_finder(maze);

    IDistanceSensors * distance_sensors = configure_distance_sensors();

    IMotor * right_motor = NULL;
    IMotor * left_motor  = NULL;
    configure_motors(&right_motor, &left_motor);
    PairedMotors * paired_motors = new PairedMotors(*right_motor, *left_motor, WHEEL_BASE);

    wall_threshold_t wall_thresholds;
    wall_thresholds.front     = (maze->get_cell_length() / 2.f) + WALL_TRESHOLD_OFFSET_FRONT;
    wall_thresholds.side      = (maze->get_cell_length() / 2.f) + WALL_TRESHOLD_OFFSET_SIDE;
    // TODO: Think through what the diagonal distance needs to be.  Whether or not we
    // want to try to correct the actual distance in mapping curve or handle it here.
    wall_thresholds.diagonal  = (maze->get_cell_length()) + WALL_TRESHOLD_OFFSET_DIAGONAL;

    // Stored offsets from sensor to center of robot.
    sensor_offset_t sensor_offsets = { SENSOR_OFFSET_SIDE, SENSOR_OFFSET_FRONT, SENSOR_OFFSET_DIAGONAL };

    distance_sensors->Initialize();

    paired_motors->Initialize();

    // TODO: Figure out gains / limits
    static PID centering_pid(CENTERING_PID_KP,
                             CENTERING_PID_KI,
                             CENTERING_PID_KD,
                             CENTERING_PID_SAT_HIGH,
                             CENTERING_PID_SAT_LOW,
                             CENTERING_PID_INT_SAT_HIGH,
                             CENTERING_PID_INT_SAT_LOW);

    Micromouse * micromouse = new Micromouse(*maze,
                                             *path_finder,
                                             *distance_sensors,
                                             *paired_motors,
                                             centering_pid,
                                             wall_thresholds,
                                             sensor_offsets,
                                             TRAVELLING_SPEED,
                                             TURNING_SPEED);

    if (USE_SIMULATED_SENSORS)
    {
        post_setup_simulated_sensors(distance_sensors, maze, micromouse);
    }

    return micromouse;

} // configure_micromouse()
