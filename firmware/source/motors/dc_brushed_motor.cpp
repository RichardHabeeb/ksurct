/****************************************************************************************
* File: dc_brushed_motor.cpp
*
* Description:
*
* Created: 4/3/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "gpio.h"
#include "dc_brushed_motor.h"
#include "util_math.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                    MEMORY CONSTANTS
---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      VARIABLES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Name: DCBrushedMotor
*
* Description: Constructor
******************************************************************************/
DCBrushedMotor::DCBrushedMotor
    (
        OutputPWMTimer & pwm_timer,
        pwm_channel_t    pwm_channel,
        OutputPin      & direction_pin
    ) :
    pwm_timer(pwm_timer),
    direction_pin(direction_pin)
{
    this->pwm_channel = pwm_channel;
    this->update_total_encoder_counts = true;
    this->total_encoder_counts = 0;
    this->current_encoder_counts = 0;

} // DCBrushedMotor()

/******************************************************************************
* Name: ReInitialize
*
* Description: Clears any stateful fields such as total encoder counts, etc. and stops
*              motor.
******************************************************************************/
void DCBrushedMotor::ReInitialize(void)
{
    Stop();
    current_encoder_counts = 0;
    total_encoder_counts = 0;

} // ReInitialize()

/******************************************************************************
* Name: Drive
*
* Description: Drives motor in either forward or reverse direction at the specified PWM value.
*              Provided speed should always be positive.
******************************************************************************/
void DCBrushedMotor::Drive
    (
        motor_direction_t new_direction, // Either forwards or backwards.
        float             new_pwm_duty_cyle  // Should be from 0 (off) to 100 (full speed)
    )
{
    SetDirection(new_direction);

    SetDutyCycle(new_pwm_duty_cyle);

} // Drive()

/******************************************************************************
* Name: Stop
*
* Description: Commands motor to come to a complete stop.
******************************************************************************/
void DCBrushedMotor::Stop(void)
{
    SetDutyCycle(0);

} // Stop()

/******************************************************************************
* Name: IsStopped
*
* Description: Returns whether the motor is currently stopped.
******************************************************************************/
bool DCBrushedMotor::IsStopped(void)
{
    // TODO add some additional logic using feedback from encoders.
    return (current_pwm_duty_cyle == 0);

} // IsStopped()

/******************************************************************************
* Name: SetDirection
*
* Description: Sets direction to either forward or backwards without changing motor speed.
******************************************************************************/
inline void DCBrushedMotor::SetDirection
    (
        motor_direction_t new_direction
    )
{
    if (new_direction == drive_forward)
    {
        direction_pin.WriteLow();
    }
    else
    {
        direction_pin.WriteHigh();
    }

} // SetDirection()

/******************************************************************************
* Name: SetDutyCycle
*
* Description:
******************************************************************************/
inline void DCBrushedMotor::SetDutyCycle
    (
        float new_duty_cycle // From 0 (off) to 100 (full speed)
    )
{
    new_duty_cycle = CapBounds(new_duty_cycle, 0.f, 100.f);

    // Need to divide duty cycle by 100 to get into 0-1 range.
    pwm_timer.SetChannelDutyCycle(pwm_channel, new_duty_cycle / 100.f);
    current_pwm_duty_cyle = new_duty_cycle;

} // SetDutyCycle()

/******************************************************************************
* Name: IncrementEncoderTicks
*
* Description: Increase encoder counts by 1. Should be called by ISR.
******************************************************************************/
void DCBrushedMotor::IncrementEncoderTicks(void)
{
    current_encoder_counts++;

    if (update_total_encoder_counts)
    {
        total_encoder_counts++;
    }

} // IncrementEncoderTicks()

