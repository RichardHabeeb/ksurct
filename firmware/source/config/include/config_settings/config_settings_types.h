/****************************************************************************************
* File: config_settings_types.h
*
* Description: Defines common configuration types (ie motor type, etc)
*
* Created: 04/20/2014, by Kyle McGahee
****************************************************************************************/

#ifndef CONFIG_SETTINGS_TYPES_INCLUDED_H
#define CONFIG_SETTINGS_TYPES_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                         TYPES
*--------------------------------------------------------------------------------------*/

// What type of motors to use.
typedef enum
{
    DC_BRUSHED_MOTOR_TYPE,
    STEPPER_MOTOR_TYPE,
} motor_type_t;

// Whether or not the micromouse has sensors on both the front and back or just the front.
typedef enum
{
    ONE_SIDED_SENSOR_TYPE,
    TWO_SIDED_SENSOR_TYPE,
} sensor_side_t;

// Default maze solving type.
typedef enum
{
    SIMPLE_FLOOD_FILL_SOLVER,
    WEIGHTED_PATH_SOLVER
} maze_solver_type_t;

#endif // CONFIG_SETTINGS_TYPES_INCLUDED_H
