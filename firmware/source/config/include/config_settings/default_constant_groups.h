/****************************************************************************************
* File: default_constant_groups.h
*
* Description: Checks every constants group and if a group hasn't been defined in the
*              current micromouse specific header then this file will define the constants in
*              the group to all zeros.  These constants should never be used at runtime and
*              this file only exists to because the unused files need to be built and require
*              their specific constants.
*
* Note: All constant groups that exist in any micromouse specific header must also be
*       conditionally defined here as well.
*
* Created: 04/20/2014, by Kyle McGahee
****************************************************************************************/

#ifndef DEFAULT_CONSTANT_GROUPS_INCLUDED_H
#define DEFAULT_CONSTANT_GROUPS_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                      INCLUDES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                    DEFAULT DEFINITIONS FOR UNUSED CONSTANT GROUPS
*--------------------------------------------------------------------------------------*/

/***************************************************************************************/
#ifndef DC_BRUSHED_MOTOR_CONSTANTS_GROUP

#define ENCODER_TICKS_PER_REVOLUTION (0)
#define ANGULAR_RATE_PER_DUTY        (0)
#define VELOCITY_CONTROLLER_RATE     (0)

#endif // DC_BRUSHED_MOTOR_CONSTANTS_GROUP

/***************************************************************************************/
#ifndef STEPPER_MOTOR_CONSTANTS_GROUP

#define  FULL_STEPS_PER_REVOLUTION    (0)
#define  PULSES_PER_STEP              (0)
#define  ACCELERATION_ENABLED         (false)
#define  ACCELERATION_TIME_CONSTANT   (0)
#define  ACCELERATION_REF_SPEED       (0)
#define  VELOCITY_UPDATE_INCREMENT    (0)

#endif // STEPPER_MOTOR_CONSTANTS_GROUP

#endif // DEFAULT_CONSTANT_GROUPS_INCLUDED_H
