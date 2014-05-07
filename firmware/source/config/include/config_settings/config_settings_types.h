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

// Which robot is being used.  I was hoping to avoid this, but it seems to make the code
// more readable and less easy to break.
typedef enum
{
    BABY_KITTEN,
    POWER_LION,
} robot_identifier_t;

// What type of motors to use.
typedef enum
{
    DC_BRUSHED_MOTOR_TYPE,
    STEPPER_MOTOR_TYPE,
} motor_type_t;

// Default maze solving type.
typedef enum
{
    SIMPLE_FLOOD_FILL_SOLVER,
    WEIGHTED_PATH_SOLVER
} maze_solver_type_t;

#endif // CONFIG_SETTINGS_TYPES_INCLUDED_H
