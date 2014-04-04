/****************************************************************************************
* File: motor_callbacks.h
*
* Description:
*
* Created: 4/20/2014, by Kyle McGahee
****************************************************************************************/

#ifndef MOTOR_CALLBACKS_INCLUDED_H
#define MOTOR_CALLBACKS_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      PROCEDURES
*--------------------------------------------------------------------------------------*/

// Forward class declarations
class DCBrushedMotor;
class VelocityController;
class TimerInterruptOC;
class StepperMotor;

void InitializeDCBrushedMotorCallbacks
    (
        DCBrushedMotor     * right_brushed_motor,
        DCBrushedMotor     * left_brushed_motor,
        VelocityController * right_brushed_motor_velocity_controller,
        VelocityController * left_brushed_motor_velocity_controller,
        TimerInterruptOC   * velocity_control_timer,
        float                velocity_control_rate // Hz
    );

void InitializeStepperMotorCallbacks
    (
        StepperMotor      * right_stepper_motor,
        StepperMotor      * left_stepper_motor,
        TimerInterruptOC  * stepping_timer
    );

extern "C" void SetRightAcceleration
    (
        int32_t acceleration
    );

extern "C" void SetLeftAcceleration
    (
        int32_t acceleration
    );

#endif // MOTOR_CALLBACKS_INCLUDED_H
