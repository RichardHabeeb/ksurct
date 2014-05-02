/****************************************************************************************
* File: config_settings.h
*
* Description: Provides a single header that defines build specific settings.
*
* Created: 04/20/2014, by Kyle McGahee
****************************************************************************************/

#ifndef CONFIG_SETTINGS_INCLUDED_H
#define CONFIG_SETTINGS_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                      INCLUDES
*--------------------------------------------------------------------------------------*/

#include "config_settings_types.h"

/*---------------------------------------------------------------------------------------
*                        MICROMOUSE CONFIGURATION SETTINGS FILE
*--------------------------------------------------------------------------------------*/

// To change micromouse this is the only line you should have to change.
//#include "power_lion_settings.h"
#include "baby_kitten_settings.h"

// Define any constants that the micromouse specific header file didn't need to define.
// These are only defined for the sake of building all the files and should not be used
// at runtime.
#include "default_constant_groups.h"

/*---------------------------------------------------------------------------------------
*                                  MAZE SETTINGS
*--------------------------------------------------------------------------------------*/

#define MAZE_NUMBER_OF_ROWS      (16)
#define MAZE_NUMBER_OF_COLUMNS   (16)
#define MAZE_CELL_LENGTH         (18.f) // centimeters

/*---------------------------------------------------------------------------------------
*                                  TEST SETTINGS
*--------------------------------------------------------------------------------------*/

// If using simulated sensors they will always need to use a test maze (that's how they're simulated)
// so if 'use simulated sensors' is true and 'use test maze' is false then will pass a blank 'real'
// maze to micromouse but sensors will still be mapping the 'test' maze walls. (THIS IS BORKED)
#define USE_SIMULATED_SENSORS   (true)
#define USE_TEST_MAZE           (true)

#endif // CONFIG_SETTINGS_INCLUDED_H
