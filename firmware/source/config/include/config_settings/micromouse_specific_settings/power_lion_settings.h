/****************************************************************************************
* File: power_lion_settings.h
*
* Description: Defines vehicle specific settings for power lion micromouse.
*
* Created: 04/20/2014, by Kyle McGahee
****************************************************************************************/

#ifndef POWER_LION_SETTINGS_INCLUDED_H
#define POWER_LION_SETTINGS_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                    INCLUDES
*--------------------------------------------------------------------------------------*/

#include "config_settings_types.h"

/*---------------------------------------------------------------------------------------
*                                   SETTINGS
*--------------------------------------------------------------------------------------*/

#define  ROBOT                 (POWER_LION)
#define  MOTOR_TYPE            (DC_BRUSHED_MOTOR_TYPE)
#define  MAZE_SOLVER           (SIMPLE_FLOOD_FILL_SOLVER)
#define  WHEEL_BASE            (7.f)  // centimeters
#define  WHEEL_DIAMETER        (3.f)  // centimeters
#define  TRAVELLING_SPEED      (18.f) // centimeters / second
#define  TURNING_SPEED         (30.f) // degrees / second

/*---------------------------------------------------------------------------------------
*                                 SENSOR OFFSETS
*--------------------------------------------------------------------------------------*/

// All offsets are in centimeters.
// TODO: Measure these.
#define  SENSOR_OFFSET_FRONT     (5.f)
#define  SENSOR_OFFSET_SIDE      (5.f)
#define  SENSOR_OFFSET_DIAGONAL  (5.f)

/*---------------------------------------------------------------------------------------
*                             WALL THRESHOLD OFFSETS
*--------------------------------------------------------------------------------------*/

// All offsets are in centimeters.
#define  WALL_TRESHOLD_OFFSET_FRONT     (5.f)
#define  WALL_TRESHOLD_OFFSET_SIDE      (5.f)
#define  WALL_TRESHOLD_OFFSET_DIAGONAL  (5.f)

/*---------------------------------------------------------------------------------------
*                              COVERED THRESHHOLD
*--------------------------------------------------------------------------------------*/
#define  SENSOR_COVERED_THRESHOLD       (3.0f)

/*---------------------------------------------------------------------------------------
*                             CENTERING PID PARAMETERS
*--------------------------------------------------------------------------------------*/

#define CENTERING_PID_KP              (0.0f)
#define CENTERING_PID_KI              (0.0f)
#define CENTERING_PID_KD              (0.0f)
#define CENTERING_PID_SAT_HIGH        (100.f)
#define CENTERING_PID_SAT_LOW         (-100.f)
#define CENTERING_PID_INT_SAT_HIGH    (100.f)
#define CENTERING_PID_INT_SAT_LOW     (-100.f)

/*---------------------------------------------------------------------------------------
*                           SPECIFIC MOTOR/WHEEL SETTINGS
*--------------------------------------------------------------------------------------*/

#define DC_BRUSHED_MOTOR_CONSTANTS_GROUP
#define ENCODER_TICKS_PER_REVOLUTION (1000) // How many encoder ticks make up one wheel rotation.
#define ANGULAR_RATE_PER_DUTY        (18.f) // Degrees / sec for a given % duty cycle.
#define VELOCITY_CONTROLLER_RATE     (30.f) // How many times a second to try to control motor velocity.

#endif // POWER_LION_SETTINGS_INCLUDED_H
