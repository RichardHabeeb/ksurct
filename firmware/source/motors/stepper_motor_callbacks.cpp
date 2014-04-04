/****************************************************************************************
* File: stepper_motor_callbacks.cpp
*
* Description: Include file description here.
*
* Created: 4/20/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <cmath>

#include "stepper_motor.h"
#include "motor_callbacks.h"
#include "timer_interrupt_oc.h"
#include "util_math.h"

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

static StepperMotor      * right_motor;
static StepperMotor      * left_motor;
static TimerInterruptOC  * timer;
static oc_channel_t        right_motor_velocity_timer_channel;
static oc_channel_t        left_motor_velocity_timer_channel;
static oc_channel_t        right_motor_acceleration_timer_channel;
static oc_channel_t        left_motor_acceleration_timer_channel;

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

// Forward declarations
extern "C"
{
void RightMotorStepper(void);
void LeftMotorStepper(void);
void RightMotorAccelerator(void);
void LeftMotorAccelerator(void);
void SetRightAcceleration(int32_t acceleration);
void SetLeftAcceleration(int32_t acceleration);
}

/******************************************************************************
* Name: InitializeStepperMotorCallbacks
*
* Description:
******************************************************************************/
void InitializeStepperMotorCallbacks
    (
        StepperMotor      * right_stepper_motor,
        StepperMotor      * left_stepper_motor,
        TimerInterruptOC  * stepping_timer
    )
{
    right_motor = right_stepper_motor;
    left_motor  = left_stepper_motor;

    timer = stepping_timer;

    right_motor_velocity_timer_channel     = oc_channel_1;
    left_motor_velocity_timer_channel      = oc_channel_2;
    right_motor_acceleration_timer_channel = oc_channel_3;
    left_motor_acceleration_timer_channel  = oc_channel_4;

    // Setup timer interrupts that will step it's motor each interrupt
    timer->InitChannel(right_motor_velocity_timer_channel, 0, RightMotorStepper);
    timer->InitChannel(left_motor_velocity_timer_channel,  0, LeftMotorStepper);

    // Setup timer interrupts that are responsible for changing the frequency
    // of the above stepping timer interrupts.
    timer->InitChannel(right_motor_acceleration_timer_channel, 0, RightMotorAccelerator);
    timer->InitChannel(left_motor_acceleration_timer_channel,  0, LeftMotorAccelerator);

} // InitializeStepperMotorCallbacks()

/******************************************************************************
* Name: SetRightAcceleration
*
* Description: Passed to StepperMotor instance so that stepper motor can handle
*              all the acceleration business and then call this function without
*              worrying about how motor control is accelerating motors.
******************************************************************************/
extern "C" void SetRightAcceleration
    (
        int32_t acceleration
    )
{
    // Update acceleration timer interrupt so it will interrupt at a frequency
    // equal to input parameter 'acceleration'.
    timer->SetChannelFrequency(right_motor_acceleration_timer_channel, AbsoluteValue(acceleration));

} // SetRightAcceleration()

/******************************************************************************
* Name: SetLeftAcceleration
*
* Description: Passed to StepperMotor instance so that stepper motor can handle
*              all the acceleration business and then call this function without
*              worrying about how motor control is accelerating motors.
******************************************************************************/
extern "C" void SetLeftAcceleration
    (
        int32_t acceleration
    )
{
    // Update acceleration timer interrupt so it will interrupt at a frequency
    // equal to input parameter 'acceleration'.
    timer->SetChannelFrequency(left_motor_acceleration_timer_channel, AbsoluteValue(acceleration));

} // SetLeftAcceleration()

/******************************************************************************
* Name: RightMotorStepper
*
* Description: Pulses right motor once to take a step (or microstep)
******************************************************************************/
extern "C" void RightMotorStepper(void)
{
    right_motor->Step();

} // RightMotorStepper

/******************************************************************************
* Name: LeftMotorStepper
*
* Description: Pulses left motor once to take a step (or microstep)
******************************************************************************/
extern "C" void LeftMotorStepper(void)
{
    left_motor->Step();

} // LeftMotorStepper

/******************************************************************************
* Name: RightMotorAccelerator
*
* Description: Updates the right motor stepper interrupt to reflect the new
*              speed of the motor.
******************************************************************************/
extern "C" void RightMotorAccelerator(void)
{
    static uint32_t current_right_frequency = 0;
    uint32_t new_frequency = right_motor->UpdateSpeed();

    if (current_right_frequency != new_frequency)
    {
        // Have some new frequency we want to step the motors at so update timer interrupt.
        timer->SetChannelFrequency(right_motor_velocity_timer_channel, new_frequency);
    }
    else
    {
        // We are now stepping at desired frequency so disable THIS interrupt until we need it again.
        timer->SetChannelFrequency(right_motor_acceleration_timer_channel, 0);
    }

    current_right_frequency = new_frequency;

} // RightMotorAccelerator

/******************************************************************************
* Name: LeftMotorAccelerator
*
* Description: Updates the left motor stepper interrupt to reflect the new
*              speed of the motor.
******************************************************************************/
extern "C" void LeftMotorAccelerator(void)
{
    static uint32_t current_left_frequency = 0;
    uint32_t new_frequency = left_motor->UpdateSpeed();

    if (current_left_frequency != new_frequency)
    {
        // Have some new frequency we want to step the motors at so update timer interrupt.
        timer->SetChannelFrequency(left_motor_velocity_timer_channel, new_frequency);
    }
    else
    {
        // We are now stepping at desired frequency so disable THIS interrupt until we need it again.
        timer->SetChannelFrequency(left_motor_acceleration_timer_channel, 0);
    }

    current_left_frequency = new_frequency;

} // LeftMotorAccelerator()
