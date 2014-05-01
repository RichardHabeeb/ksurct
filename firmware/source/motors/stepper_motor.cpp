/****************************************************************************************
* File: stepper_motor.cpp
*
* Description: Provides 'stepping' logic for a single stepper motor using a motor driver
*              with step and direction pins.  These pins need to be provided through
*              the GPIO utility class. This class also provides the means to 'accelerate'
*              a stepper motor through a callback function that then changes the stepping
*              frequency every interrupt.
*
* Created: 1/13/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <cmath>

#include "gpio.h"
#include "stepper_motor.h"
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
* Name: StepperMotor - Constructor
*
* Description:
******************************************************************************/
StepperMotor::StepperMotor
    (
        OutputPin     & step_pin,
        OutputPin     & direction_pin,
        OutputPin     & micro_select_1_pin,         // Controls microstepping mode.
        OutputPin     & micro_select_2_pin,         // Controls microstepping mode.
        OutputPin     & micro_select_3_pin,         // Controls microstepping mode.
        float           wheel_diameter,             // In centimeters.
        float           full_steps_per_revolution,  // How many full steps needed to rotate wheel once.
        uint8_t         pulses_per_step,            // Ie 2 for half stepping, 16 for 1/16th stepping, etc.
        bool            acceleration_enabled,       // Whether or not to use acceleration logic.
        float           acceleration,               // In full steps per seconds^2
        float           velocity_update_inc,        // How many full steps to update by when accelerating.
        accelerator_t   accel_function_pointer      // Acceleration callback function used to change ISR frequency.
    ) :
    direction_pin(direction_pin),
    step_pin(step_pin),
    micro_select_1_pin(micro_select_1_pin),
    micro_select_2_pin(micro_select_2_pin),
    micro_select_3_pin(micro_select_3_pin)
{
    this->accel_function_pointer     = accel_function_pointer;
    this->pulses_per_step            = pulses_per_step;
    this->acceleration_enabled       = acceleration_enabled;
    this->acceleration               = acceleration;
    this->velocity_update_inc        = velocity_update_inc;

    // Number of steps need for the wheel to make one full rotation
    this->full_steps_per_revolution = full_steps_per_revolution;

    // In centimeters
    this->wheel_diameter      = wheel_diameter;
    this->wheel_circumference = wheel_diameter * PI;

    // Not sure why we have to multiply by 2.  Probably losing a factor of 2 somewhere else.
    // Oh well it seems to work for now.
    this->full_steps_per_cm = 2.f * full_steps_per_revolution / wheel_circumference;
    this->cm_per_full_step  = 1.0f / full_steps_per_cm;
    this->steps_per_cm      = full_steps_per_cm * pulses_per_step;
    this->cm_per_step       = 1.0f / steps_per_cm;

    this->update_total_steps = true;
    this->current_speed = 0;
    this->target_speed  = 0;
    this->total_steps   = 0;
    this->current_steps = 0;

} // StepperMotor()

/******************************************************************************
* Name: Initialize
*
* Description:
******************************************************************************/
void StepperMotor::Initialize(void)
{
    step_pin.Init(LOW);

    direction_pin.Init(LOW);

    // Microstepping select pins.
    switch (pulses_per_step)
    {
        case 1:
            micro_select_1_pin.Init(LOW);
            micro_select_2_pin.Init(LOW);
            micro_select_3_pin.Init(LOW);
            break;
        case 2:
            micro_select_1_pin.Init(HIGH);
            micro_select_2_pin.Init(LOW);
            micro_select_3_pin.Init(LOW);
            break;
        case 4:
            micro_select_1_pin.Init(LOW);
            micro_select_2_pin.Init(HIGH);
            micro_select_3_pin.Init(LOW);
            break;
        case 8:
            micro_select_1_pin.Init(HIGH);
            micro_select_2_pin.Init(HIGH);
            micro_select_3_pin.Init(LOW);
            break;
        default: // Intentional fall-through to 16
        case 16:
            micro_select_1_pin.Init(HIGH);
            micro_select_2_pin.Init(HIGH);
            micro_select_3_pin.Init(HIGH);
            break;
    }


    ReInitialize();

} // Initialize()

/******************************************************************************
* Name: ReInitialize
*
* Description: Clears any stateful fields such as total steps, etc. and stops
*              motor.
******************************************************************************/
void StepperMotor::ReInitialize(void)
{
    Stop();
    current_steps = 0;
    total_steps = 0;

} // ReInitialize()

/******************************************************************************
* Name: Drive
*
* Description: Set motor to drive at specified forward velocity. If velocity is negative
*              then motor will drive in reverse.
******************************************************************************/
void StepperMotor::Drive
    (
        float new_velocity // cm / sec
    )
{
    motor_direction_t new_direction = (new_velocity >= 0.f) ? drive_forward : drive_backward;

    SetDirection(new_direction);

    SetTargetSpeed((uint32_t)(full_steps_per_cm * fabs(new_velocity)));

} // Drive()

/******************************************************************************
* Name: AddVelocity
*
* Description: Change current commanded velocity by the given amount. Can be either
*              positive or negative.
******************************************************************************/
void StepperMotor::AddVelocity
    (
        float velocity_change // cm / sec
    )
{
    float new_velocity = get_commanded_velocity() + velocity_change;

    Drive(new_velocity);

} // AddVelocity()

/******************************************************************************
* Name: Step
*
* Description: Takes one step (or microstep). Always updates current steps and
*              if configured will also update total steps.
******************************************************************************/
void StepperMotor::Step(void)
{
    // In order for us to actually take a step we need to make sure we have a nonzero
    // target speed.  It takes a small amount of time for current_speed to get set to zero
    // so if this check isn't here the motors won't stop quite when you tell them to.
    // If acceleration is enabled then we need the ability to de-accelerate down to zero.
    if (target_speed != 0)
    //|| (acceleration_enabled && acceleration < 0)) doesn't work quite right I'm not worried about it though
    {
        if (current_direction == drive_forward)
        {
            current_steps++;
            total_steps++;
        }
        else
        {
            current_steps--;
            total_steps--;
        }

        step_pin.Toggle();
    }

} // Step()

/******************************************************************************
* Name: Stop
*
* Description: Commands motor to come to a complete stop.  If acceleration mode
*              is enabled then will deaccelerate.
******************************************************************************/
void StepperMotor::Stop(void)
{
    SetTargetSpeed(0);

} // Stop()

/******************************************************************************
* Name: IsStopped
*
* Description: Returns whether the motor is currently stopped.
******************************************************************************/
bool StepperMotor::IsStopped(void)
{
    return (current_speed == 0);

} // IsStopped()

/******************************************************************************
* Name: UpdateSpeed
*
* Description: Updates current speed for motor to reflect target speed. Then
*              if there is some non-zero speed step the motor.  The new
*              frequency needed to realize the new current speed is returned.
******************************************************************************/
uint32_t StepperMotor::UpdateSpeed(void)
{
    // Check to see if we need to update our current speed because we're not at the
    // correct speed yet
    if (current_speed != target_speed)
    {
        // Check to see if we want to bother with accelerating, if we don't then just
        // use fixed speed. Also for right now no de-acceleration.
        if (acceleration_enabled && target_speed != 0)
        {
            UpdateCurrentSpeedWithAcceleration();
        }
        else // Use fixed speed method. (ie jump straight to target speed)
        {
            current_speed = target_speed;
        }
    }

    // Return stepping frequency needed to achieve current speed (not necessarily target
    // speed).  Need to multiply by two because only step on a rising edge.
    return (2 * current_speed * pulses_per_step);

} // UpdateSpeed()

/******************************************************************************
* Name: SetDirection
*
* Description: Sets direction to either forward or backwards without changing
*              motor speed.
******************************************************************************/
void StepperMotor::SetDirection
    (
        motor_direction_t new_direction
    )
{
    if (new_direction == drive_forward)
    {
        direction_pin.WriteHigh();
    }
    else
    {
        direction_pin.WriteLow();
    }

    current_direction = new_direction;

} // SetDirection()

/******************************************************************************
* Name: SetTargetSpeed
*
* Description: Updates the speed the motor is trying to reach.  If acceleration
*              is enabled in the motor control then this speed will be instantly
*              realized next interrupt service routine.
******************************************************************************/
void StepperMotor::SetTargetSpeed
    (
        uint32_t new_speed // Full steps per seconds
    )
{
    target_speed = new_speed;

    // Need to update 'accelerator' ISR frequency that is in charge of updating stepping
    // frequency.
    accel_function_pointer((int32_t)(acceleration / velocity_update_inc));

} // SetTargetSpeed()

/******************************************************************************
* Name: UpdateCurrentSpeedWithAcceleration
*
* Description: When acceleration is enabled this helper method is used to update
*              the current speed by some incremental value
******************************************************************************/
inline void StepperMotor::UpdateCurrentSpeedWithAcceleration(void)
{
    current_speed += (acceleration > 0.f ? velocity_update_inc : -velocity_update_inc);

    // Keep speed in range based off whether we're accelerating or deaccelerating.
    if (OvershotTargetSpeed())
    {
        current_speed = target_speed;
    }

} // UpdateCurrentSpeedWithAcceleration()

/******************************************************************************
* Name: OvershotTargetSpeed
*
* Description: Returns true if we go outside bounds of target speed.
******************************************************************************/
inline bool StepperMotor::OvershotTargetSpeed(void)
{
    return ((acceleration > 0.f && current_speed > target_speed)
         || (acceleration < 0.f && current_speed < target_speed));

} // OvershotTargetSpeed()

/******************************************************************************
* Name: get_commanded_velocity
*
* Description:
******************************************************************************/
float StepperMotor::get_commanded_velocity(void)
{
    float sign = (current_direction == drive_forward) ? 1.f : -1.f;

    return target_speed * cm_per_full_step * sign;

} // get_commanded_velocity()
