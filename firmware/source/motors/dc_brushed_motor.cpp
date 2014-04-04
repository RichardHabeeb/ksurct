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

#include <cmath>

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
        AlternateFunctionPin & pwm_pin,
        OutputPin            & direction_pin,
        OutputPWMTimer       & pwm_timer,
        pwm_channel_t          pwm_channel,
        float                  wheel_diameter,                // Centimeters
        float                  encoder_counts_per_revolution, // How many encoder counts make up one wheel rotation.
        float                  angular_rate_per_duty          // Degrees / sec for a given % duty cycle.
    ) :
    pwm_pin(pwm_pin),
    direction_pin(direction_pin),
    pwm_timer(pwm_timer)
{
    this->pwm_channel = pwm_channel;

    this->wheel_diameter = wheel_diameter;
    this->wheel_circumference = wheel_diameter * PI;

    this->angular_rate_per_duty = angular_rate_per_duty;

    this->encoder_counts_per_revolution = encoder_counts_per_revolution;

    this->cm_per_encoder_tick = wheel_circumference / encoder_counts_per_revolution;

    this->update_total_encoder_counts = true;
    this->total_encoder_counts = 0;
    this->current_encoder_counts = 0;
    this->current_speed = 0.f;

} // DCBrushedMotor()

/******************************************************************************
* Name: Initialize
*
* Description:
******************************************************************************/
void DCBrushedMotor::Initialize(void)
{
    pwm_timer.InitChannel(pwm_channel, pwm_pin);

    ReInitialize();

} // Initialize()

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
* Description: Set motor to drive at specified forward velocity. If velocity is negative
*              then motor will drive in reverse.
******************************************************************************/
void DCBrushedMotor::Drive
    (
        float new_velocity // cm / sec
    )
{
    motor_direction_t new_direction = (new_velocity >= 0.f) ? drive_forward : drive_backward;

    SetDirection(new_direction);

    SetSpeed(fabs(new_velocity));

} // Drive()

/******************************************************************************
* Name: AddVelocity
*
* Description: Change current commanded velocity by the given amount. Can be either
*              positive or negative.
******************************************************************************/
void DCBrushedMotor::AddVelocity
    (
        float velocity_change // cm / sec
    )
{
    float new_velocity = get_commanded_velocity() + velocity_change;

    Drive(new_velocity);

} // AddVelocity()

/******************************************************************************
* Name: Stop
*
* Description: Commands motor to come to a complete stop.
******************************************************************************/
void DCBrushedMotor::Stop(void)
{
    SetSpeed(0);

} // Stop()

/******************************************************************************
* Name: IsStopped
*
* Description: Returns whether the motor is currently stopped.
******************************************************************************/
bool DCBrushedMotor::IsStopped(void)
{
    // TODO add some additional logic using feedback from encoders.
    return (current_speed == 0);

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

    current_direction = new_direction;

} // SetDirection()

/******************************************************************************
* Name: SetSpeed
*
* Description:
******************************************************************************/
inline void DCBrushedMotor::SetSpeed
    (
        float new_speed // Should be positive (cm / sec)
    )
{
    float new_duty_cycle = ConvertSpeedToDutyCycle(new_speed);

    new_duty_cycle = CapBounds(new_duty_cycle, 0.f, 100.f);

    // Need to divide duty cycle by 100 to get into 0-1 range.
    pwm_timer.SetChannelDutyCycle(pwm_channel, new_duty_cycle / 100.f);

    current_speed = new_speed;

} // SetSpeed()

/******************************************************************************
* Name: IncrementEncoderTicks
*
* Description: Increase encoder counts by 1. Should be called by ISR.
******************************************************************************/
void DCBrushedMotor::IncrementEncoderTicks(void)
{
    if (current_direction == drive_forward)
    {
        current_encoder_counts++;
        total_encoder_counts++;
    }
    else
    {
        current_encoder_counts--;
        total_encoder_counts--;
    }

} // IncrementEncoderTicks()

/******************************************************************************
* Name: ConvertSpeedToDutyCycle
*
* Description: Returns duty cycle corresponding to specified linear speed in cm/sec.
******************************************************************************/
float DCBrushedMotor::ConvertSpeedToDutyCycle
    (
        float speed // Must be positive. (cm / sec)
    )
{
    float angular_rate = AbsoluteValue(speed) / wheel_circumference * 360.f;
    float duty_cycle = angular_rate / angular_rate_per_duty;

    return duty_cycle;

} // ConvertSpeedToDutyCycle()

/******************************************************************************
* Name: get_commanded_velocity
*
* Description:
******************************************************************************/
float DCBrushedMotor::get_commanded_velocity(void)
{
    float sign = (current_direction == drive_forward) ? 1.f : -1.f;

    return current_speed * sign;

} // get_commanded_velocity()
