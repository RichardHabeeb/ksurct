/****************************************************************************************
* File: baby_kitten_settings.h
*
* Description: Defines vehicle specific settings for power lion micromouse.
*
* Created: 04/20/2014, by Kyle McGahee
****************************************************************************************/

#ifndef BABY_KITTEN_SETTINGS_INCLUDED_H
#define BABY_KITTEN_SETTINGS_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                    INCLUDES
*--------------------------------------------------------------------------------------*/

#include "config_settings_types.h"

/*---------------------------------------------------------------------------------------
*                                   SETTINGS
*--------------------------------------------------------------------------------------*/

#define  ROBOT                 (BABY_KITTEN)
#define  MOTOR_TYPE            (STEPPER_MOTOR_TYPE)
#define  MAZE_SOLVER           (SIMPLE_FLOOD_FILL_SOLVER)
#define  WHEEL_BASE            (9.27f) // centimeters
#define  WHEEL_DIAMETER        (3.1f)   // centimeters
#define  TRAVELLING_SPEED      (5.f)   // centimeters / second
#define  TURNING_SPEED         (5.f)   // degrees / second

/*---------------------------------------------------------------------------------------
*                                 SENSOR OFFSETS
*--------------------------------------------------------------------------------------*/

// All offsets are in centimeters.
// TODO: Measure these.
#define  SENSOR_OFFSET_FRONT     (5.6f)
#define  SENSOR_OFFSET_SIDE      (4.75f)
#define  SENSOR_OFFSET_DIAGONAL  (5.f)

/*---------------------------------------------------------------------------------------
*                              WALL THRESHOLD OFFSETS
*--------------------------------------------------------------------------------------*/

// All offsets are in centimeters.
#define  WALL_TRESHOLD_OFFSET_FRONT     (5.f)
#define  WALL_TRESHOLD_OFFSET_SIDE      (5.f)
#define  WALL_TRESHOLD_OFFSET_DIAGONAL  (5.f)

/*---------------------------------------------------------------------------------------
*                              CENTERING PID PARAMETERS
*--------------------------------------------------------------------------------------*/

#define CENTERING_PID_KP              (1.5f)
#define CENTERING_PID_KI              (0.0f)
#define CENTERING_PID_KD              (.015f)
#define CENTERING_PID_SAT_HIGH        (100.f)
#define CENTERING_PID_SAT_LOW         (-100.f)
#define CENTERING_PID_INT_SAT_HIGH    (100.f)
#define CENTERING_PID_INT_SAT_LOW     (-100.f)

/*---------------------------------------------------------------------------------------
*                           SPECIFIC MOTOR/WHEEL SETTINGS
*--------------------------------------------------------------------------------------*/

#define  STEPPER_MOTOR_CONSTANTS_GROUP
#define  FULL_STEPS_PER_REVOLUTION    (200.f)  // How many full steps needed to rotate wheel once.
#define  PULSES_PER_STEP              (16)     // Ie 2 for half stepping, 16 for 1/16th stepping, etc.
#define  ACCELERATION_ENABLED         (true)   // Whether or not to use acceleration logic.
#define  ACCELERATION_TIME_CONSTANT   (.5f)    // How long to accelerate to target speed. (seconds)
#define  ACCELERATION_REF_SPEED       (20.f)   // What speed to reach after the acceleration time constant (cm/sec)
#define  VELOCITY_UPDATE_INCREMENT    (1.f)    // How many full steps to update by when accelerating.

#endif // BABY_KITTEN_SETTINGS_INCLUDED_H
