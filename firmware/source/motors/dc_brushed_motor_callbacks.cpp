/****************************************************************************************
* File: dc_brushed_motor_callbacks.cpp
*
* Description: Include file description here.
*
* Created: 4/20/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "dc_brushed_motor.h"
#include "timer_interrupt_oc.h"
#include "stm32f4xx.h"
#include "velocity_controller.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                   MEMORY CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      VARIABLES
*--------------------------------------------------------------------------------------*/

static DCBrushedMotor     * right_motor;
static DCBrushedMotor     * left_motor;
static VelocityController * right_motor_velocity_controller;
static VelocityController * left_motor_velocity_controller;
static float                velocity_control_period;

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

// Forward declarations
void ControlMotorVelocities(void);

/******************************************************************************
* Name: InitializeDCBrushedMotorCallbacks
*
* Description:
******************************************************************************/
void InitializeDCBrushedMotorCallbacks
    (
        DCBrushedMotor     * right_brushed_motor,
        DCBrushedMotor     * left_brushed_motor,
        VelocityController * right_brushed_motor_velocity_controller,
        VelocityController * left_brushed_motor_velocity_controller,
        TimerInterruptOC   * velocity_control_timer,
        float                velocity_control_rate // Hz
    )
{
    right_motor = right_brushed_motor;
    left_motor  = left_brushed_motor;

    right_motor_velocity_controller = right_brushed_motor_velocity_controller;
    left_motor_velocity_controller  = left_brushed_motor_velocity_controller;

    velocity_control_period = 1.f / velocity_control_rate;

    velocity_control_timer->InitChannel(oc_channel_1, velocity_control_rate, ControlMotorVelocities);

} // InitializeDCBrushedMotorCallbacks()

/******************************************************************************
* Name: ControlMotorVelocities
*
* Description: Tries to update both motors to match their desired frequencies.
******************************************************************************/
void ControlMotorVelocities(void)
{
    right_motor_velocity_controller->Run(velocity_control_period);
    left_motor_velocity_controller->Run(velocity_control_period);

} // ControlMotorVelocities()

/******************************************************************************
* Name: EXTI0_IRQHandler
*
* Description: External line interrupt for right motor encoder.
******************************************************************************/
extern "C" void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) == SET)
    {
        right_motor->IncrementEncoderTicks();

        EXTI_ClearITPendingBit(EXTI_Line0);
    }

} // EXTI0_IRQHandler()

/******************************************************************************
* Name: EXTI2_IRQHandler
*
* Description: External line interrupt for left motor encoder.
******************************************************************************/
extern "C" void EXTI2_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) == SET)
    {
        left_motor->IncrementEncoderTicks();

        EXTI_ClearITPendingBit(EXTI_Line2);
    }

} // EXTI2_IRQHandler()
