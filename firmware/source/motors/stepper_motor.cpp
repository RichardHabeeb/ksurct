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

#include "gpio.h"
#include "stepper_motor.h"

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
        OutputPin &   step_pin,
        OutputPin &   direction_pin,
        uint8_t       pulses_per_step,            // Ie 2 for half stepping, 16 for 1/16th stepping, etc.
        bool          acceleration_enabled,       // Whether or not to use acceleration logic.
        float         acceleration_time_constant, // How long to accelerate to target speed. (seconds)
        float         velocity_update_inc,        // How many full steps to update by when accelerating.
        accelerator_t accel_function_pointer      // Acceleration callback function used to change ISR frequency.
    ) :
    direction_pin(direction_pin),
    step_pin(step_pin)
{
    this->accel_function_pointer = accel_function_pointer;

    this->pulses_per_step = pulses_per_step;
    this->acceleration_enabled = acceleration_enabled;
    this->acceleration_time_constant = acceleration_time_constant;
    this->velocity_update_inc = velocity_update_inc;

    this->update_total_steps = true;
    this->current_speed = 0;
    this->target_speed  = 0;
    this->acceleration  = 0;
    this->total_steps   = 0;
    this->current_steps = 0;

} // StepperMotor()

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
* Description: Drives stepper motor in either forward or reverse direction at
*              specified speed.  Provided speed should always be positive.
******************************************************************************/
void StepperMotor::Drive
    (
        motor_direction_t new_direction, // Either forwards or backwards.
        uint32_t          new_speed      // Full steps per second (should always be positive)
    )
{
    SetDirection(new_direction);
    SetTargetSpeed(new_speed);

} // Drive()


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
        current_steps++;
        if (update_total_steps)
        {
            total_steps++;
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
inline void StepperMotor::SetDirection
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

    // Calculate new 'speed change' value that the current speed will be incremented by
    // every ISR if acceleration is enabled.
    acceleration = (target_speed - current_speed) / acceleration_time_constant;

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
