/****************************************************************************************
* File: private_system_config.h
*
* Description: Provides access to all module configuration functions.  Should only be
*              used by public system configuration setup procedure.
*
* Created: 04/20/2014, by Kyle McGahee
****************************************************************************************/

#ifndef PRIVATE_SYSTEM_CONFIG_INCLUDED_H
#define PRIVATE_SYSTEM_CONFIG_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      FUNCTIONS
*--------------------------------------------------------------------------------------*/

// Forward class declarations.
class Maze;
class Micromouse;
class IPathFinder;
class IMotor;
class IDistanceSensors;

Maze * configure_maze(void);

IPathFinder * configure_path_finder
    (
        Maze * maze
    );

IDistanceSensors * configure_distance_sensors(void);

void configure_motors
    (
        IMotor ** right_motor,
        IMotor ** left_motor
    );

void post_setup_simulated_sensors
    (
        IDistanceSensors * distance_sensors, // Already instantiated simulated distance sensors.
        Maze             * maze,             // Already configured maze (simulated or real)
        Micromouse       * micromouse        // Already configured micromouse
    );

#endif // PRIVATE_SYSTEM_CONFIG_INCLUDED_H
