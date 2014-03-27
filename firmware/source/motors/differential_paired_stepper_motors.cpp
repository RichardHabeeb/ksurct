/****************************************************************************************
* File: differential_paired_stepper_motors.cpp
*
* Description: Include file description here.
*
* Created: 1/13/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "differential_paired_stepper_motors.h"
#include "gpio.h"
#include "stepper_motor.h"
#include "timer_interrupt_oc.h"
#include "util_math.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                    MEMORY CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      VARIABLES
*--------------------------------------------------------------------------------------*/

// Instantiate output compare timer to step the motors (still need to be initialized)
static TimerInterruptOC timer(TIM3, TIM3_IRQn, RCC_APB1Periph_TIM3, (SystemCoreClock/4), 50000);

// Instantiate GPIO motor pins (still need to be initialized)
static OutputPin right_motor_step_pin(GPIOD, GPIO_Pin_0, RCC_AHB1Periph_GPIOD);
static OutputPin left_motor_step_pin(GPIOD, GPIO_Pin_1, RCC_AHB1Periph_GPIOD);

static OutputPin right_motor_direction_pin(GPIOD, GPIO_Pin_2, RCC_AHB1Periph_GPIOD);
static OutputPin left_motor_direction_pin(GPIOD, GPIO_Pin_3, RCC_AHB1Periph_GPIOD);

static OutputPin right_motor_enable_pin(GPIOD, GPIO_Pin_4, RCC_AHB1Periph_GPIOD);
static OutputPin left_motor_enable_pin(GPIOD, GPIO_Pin_5, RCC_AHB1Periph_GPIOD);

static OutputPin right_motor_reset_pin(GPIOD, GPIO_Pin_6, RCC_AHB1Periph_GPIOD);
static OutputPin left_motor_reset_pin(GPIOD, GPIO_Pin_7, RCC_AHB1Periph_GPIOD);

// Motor select pins (controls microstepping)
static OutputPin right_motor_ms1_pin(GPIOD, GPIO_Pin_8, RCC_AHB1Periph_GPIOD);
static OutputPin right_motor_ms2_pin(GPIOD, GPIO_Pin_9, RCC_AHB1Periph_GPIOD);
static OutputPin right_motor_ms3_pin(GPIOD, GPIO_Pin_10, RCC_AHB1Periph_GPIOD);
static OutputPin left_motor_ms1_pin(GPIOD, GPIO_Pin_11, RCC_AHB1Periph_GPIOD);
static OutputPin left_motor_ms2_pin(GPIOD, GPIO_Pin_12, RCC_AHB1Periph_GPIOD);
static OutputPin left_motor_ms3_pin(GPIOD, GPIO_Pin_13, RCC_AHB1Periph_GPIOD);

// Temporary to access motors from callback.  This shouldn't be here in release.
static StepperMotor * right_motor_ref;
static StepperMotor * left_motor_ref;

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

// Callback forward declarations (defined at end of file)
void RightMotorStepper(void);
void LeftMotorStepper(void);
void RightMotorAccelerator(void);
void LeftMotorAccelerator(void);
void SetRightAcceleration(int32_t acceleration);
void SetLeftAcceleration(int32_t acceleration);

/******************************************************************************
* Name: DifferentialPairedStepperMotors
*
* Description: Constructor
******************************************************************************/
DifferentialPairedStepperMotors::DifferentialPairedStepperMotors
    (
        bool    acceleration_enabled,      // If true then motors will accelerate.
        float   turn_speed,                // Degrees per second
        float   wheel_diameter,            // Centimeters
        float   wheel_2_wheel_distance,    // AKA wheel base.  In centimeters.
        float   full_steps_per_revolution, // How many full steps needed to rotate wheel once.
        uint8_t pulses_per_step            // How many steps per full steps. Ie 2 for half stepping.
    )
{
    this->acceleration_enabled = acceleration_enabled;
    this->wheel_diameter = wheel_diameter; // centimeters
    this->wheel_circumference = wheel_diameter * PI; // centimeters
    this->wheel_2_wheel_distance = wheel_2_wheel_distance; // centimeters
    this->pulses_per_step = pulses_per_step;

    // Like circumference of robot. But diamter is between two wheels instead of outside.
    this->wheel_2_wheel_circumference = wheel_2_wheel_distance * PI; // centimeters

    // Number of steps need for the wheel to make one full rotation
    this->full_steps_per_revolution = full_steps_per_revolution;

    // Speed needed for turns in units of full steps / second.
    this->turn_speed_full_steps_per_sec = (uint32_t)(AbsoluteValue(turn_speed) * full_steps_per_revolution / 360.0f);

    // Full steps needed to travel 1 centimeter forward/backward.
    this->full_steps_per_cm = 2.0f * full_steps_per_revolution / wheel_circumference;
    this->cm_per_full_step = 1.0f / full_steps_per_cm;

    // The amount of time to take to accelerate from current speed to target speed (or de-accelerate)
    this->acceleration_time_constant = 2.0f; // seconds

    // How many steps/sec to advance each acceleration ISR.
    // Pretty much how precise the updating of velocity will be when accelerating.
    this->velocity_update_inc = 1.0f;

    // Only divide by 180 instead of 360 since you have two motors both
    // moving you some distance so need to divide by (360/2)=180
    this->cm_per_degree_zero_point = wheel_2_wheel_circumference / 360.0f;

    // Instantiate these dynamically until I come up with a better way.
    right_motor = new StepperMotor(right_motor_step_pin,
                                   right_motor_direction_pin,
                                   pulses_per_step,
                                   acceleration_enabled,
                                   acceleration_time_constant,
                                   velocity_update_inc,
                                   SetRightAcceleration);

    left_motor = new StepperMotor( left_motor_step_pin,
                                   left_motor_direction_pin,
                                   pulses_per_step,
                                   acceleration_enabled,
                                   acceleration_time_constant,
                                   velocity_update_inc,
                                   SetLeftAcceleration);

    // Temporary to access motors from callback.  This shouldn't be here in release.
    right_motor_ref = right_motor;
    left_motor_ref = left_motor;

} // DifferentialPairedStepperMotors::DifferentialPairedStepperMotors() - Constructor

/******************************************************************************
* Name: Initialize
*
* Description: Initializes GPIO pins to default state and sets up timer interrupts
*              for stepping/accelerating.
******************************************************************************/
void DifferentialPairedStepperMotors::Initialize(void)
{
    right_motor_step_pin.Init(LOW);
    left_motor_step_pin.Init(LOW);

    right_motor_direction_pin.Init(LOW);
    left_motor_direction_pin.Init(LOW);

    // Active low logic so LOW means enabled.
    right_motor_enable_pin.Init(LOW);
    left_motor_enable_pin.Init(LOW);

    // Active low logic so HIGH means not in constant reset.
    right_motor_reset_pin.Init(HIGH);
    left_motor_reset_pin.Init(HIGH);

    // Motor select pins (controls microstepping)
    right_motor_ms1_pin.Init(HIGH);
    right_motor_ms2_pin.Init(HIGH);
    right_motor_ms3_pin.Init(HIGH);

    left_motor_ms1_pin.Init(HIGH);
    left_motor_ms2_pin.Init(HIGH);
    left_motor_ms3_pin.Init(HIGH);

    // Setup timer interrupts that will step it's motor each interrupt
    timer.InitChannel(oc_channel_1, 0, RightMotorStepper);
    timer.InitChannel(oc_channel_2, 0, LeftMotorStepper);

    // Setup timer interrupts that are responsible for changing the frequency
    // of the above stepping timer interrupts.
    timer.InitChannel(oc_channel_3, 0, RightMotorAccelerator);
    timer.InitChannel(oc_channel_4, 0, LeftMotorAccelerator);

} // DifferentialPairedStepperMotors::Initialize()

/******************************************************************************
* Name: ReInitialize
*
* Description: Clears motor control fields and re-init both motors.
******************************************************************************/
void DifferentialPairedStepperMotors::ReInitialize(void)
{
    right_motor->ReInitialize();
    left_motor->ReInitialize();

} // DifferentialPairedStepperMotors::ReInitialize()

/******************************************************************************
* Name: Drive
*
* Description: Set both motors to drive at specified speed.  If speed is negative
*              then motors will drive in reverse.
*
* Notes: If acceleration is enabled then the new speed will be accelerated to
*        from the current speed, otherwise the motors will attempt to automatically
*        jump to the new speed, which can sometime stall the motors.
******************************************************************************/
void DifferentialPairedStepperMotors::Drive
    (
        float new_speed // cm / sec
    )
{
    DriveRight(new_speed);
    DriveLeft(new_speed);

    right_motor->reset_current_steps();
    left_motor->reset_current_steps();

} // DifferentialPairedStepperMotors::Drive()

/******************************************************************************
* Name: DriveRight
*
* Description: Set right motor to drive at specified speed.  If speed is negative
*              then motor will drive in reverse.
*
* Notes: Current steps are NOT reset when calling this function.
******************************************************************************/
void DifferentialPairedStepperMotors::DriveRight
    (
        float new_speed // cm / sec
    )
{
    uint32_t new_right_speed_full_steps_per_sec = (uint32_t)(AbsoluteValue(new_speed) * full_steps_per_cm);

    motor_direction_t new_right_direction = (new_speed >= 0.f ? drive_forward : drive_backward);

    right_motor->Drive(new_right_direction, new_right_speed_full_steps_per_sec);

} // DifferentialPairedStepperMotors::DriveRight()

/******************************************************************************
* Name: DriveLeft
*
* Description: Set left motor to drive at specified speed.  If speed is negative
*              then motor will drive in reverse.
*
* Notes: Current steps are NOT reset when calling this function.
******************************************************************************/
void DifferentialPairedStepperMotors::DriveLeft
    (
        float new_speed // cm / sec
    )
{
    uint32_t new_left_speed_full_steps_per_sec = (uint32_t)(AbsoluteValue(new_speed) * full_steps_per_cm);

    motor_direction_t new_left_direction = (new_speed >= 0.f ? drive_forward : drive_backward);

    left_motor->Drive(new_left_direction, new_left_speed_full_steps_per_sec);

} // DifferentialPairedStepperMotors::DriveLeft()

/******************************************************************************
* Name: Turn
*
* Description: Turn the robot the correct 'angle' in degrees in the direction
*              specified using a zero point turn approach
******************************************************************************/
void DifferentialPairedStepperMotors::Turn
    (
        motor_direction_t new_direction, // Which direction to turn.
        float             turn_angle     // Degrees
    )
{
    // make sure both motors are stopped
    Stop();

    // Zero point turn so don't want to update total steps.
    right_motor->set_update_total_steps(false);
    left_motor->set_update_total_steps(false);

    // determine number of centimeters to wait until turn will finish
    float distance_to_turn = cm_per_degree_zero_point * turn_angle;

    right_motor->reset_current_steps();
    left_motor->reset_current_steps();

    if (new_direction == turn_right)
    {
        left_motor->Drive(drive_forward, turn_speed_full_steps_per_sec);
        right_motor->Drive(drive_backward, turn_speed_full_steps_per_sec);
    }
    else if (new_direction == turn_left)
    {
        right_motor->Drive(drive_forward, turn_speed_full_steps_per_sec);
        left_motor->Drive(drive_backward, turn_speed_full_steps_per_sec);
    }

    // wait for turn to finish
    while (get_right_motor_current_distance() < distance_to_turn);

    Stop();

    // done turning
    right_motor->set_update_total_steps(true);
    left_motor->set_update_total_steps(true);

} // DifferentialPairedStepperMotors::Turn()

/******************************************************************************
* Name: Stop
*
* Description: Tells both motors they need to stops and wait here until they do.
******************************************************************************/
void DifferentialPairedStepperMotors::Stop(void)
{
    right_motor->Stop();
    left_motor->Stop();

    while (!Stopped());

} // DifferentialPairedStepperMotors::Stop()

/******************************************************************************
* Name: Stopped
*
* Description: Returns true only if both motors are stopped
******************************************************************************/
bool inline DifferentialPairedStepperMotors::Stopped(void)
{
    return (right_motor->IsStopped() && left_motor->IsStopped());

} // DifferentialPairedStepperMotors::Stopped()

/******************************************************************************
* Name: get_right_motor_current_distance
*
* Description: Returns the number of centimeters since last speed change.
******************************************************************************/
float DifferentialPairedStepperMotors::get_right_motor_current_distance(void)
{
    return right_motor->get_current_full_steps() * cm_per_full_step;

} // DifferentialPairedStepperMotors::get_right_motor_current_distance()

/******************************************************************************
* Name: get_left_motor_current_distance
*
* Description: Returns the number of centimeters since last speed change.
******************************************************************************/
float DifferentialPairedStepperMotors::get_left_motor_current_distance(void)
{
    return left_motor->get_current_full_steps() * cm_per_full_step;

} // DifferentialPairedStepperMotors::get_left_motor_current_distance()

/******************************************************************************
* Name: get_right_motor_total_distance
*
* Description: Returns the number of centimeters since the beginning of time.
******************************************************************************/
float DifferentialPairedStepperMotors::get_right_motor_total_distance(void)
{
    return right_motor->get_total_full_steps() * cm_per_full_step;

} // DifferentialPairedStepperMotors::get_right_motor_total_distance()

/******************************************************************************
* Name: get_left_motor_total_distance
*
* Description: Returns the number of centimeters since the beginning of time.
******************************************************************************/
float DifferentialPairedStepperMotors::get_left_motor_total_distance(void)
{
    return left_motor->get_total_full_steps() * cm_per_full_step;

} // DifferentialPairedStepperMotors::get_left_motor_total_distance()

/******************************************************************************
* Name: SetAccelerationMode
*
* Description: Enable/disable acceleration mode.
******************************************************************************/
void DifferentialPairedStepperMotors::SetAccelerationMode
    (
        bool new_enabled_status // If true then acceleration mode will be enabled.
    )
{
    acceleration_enabled = new_enabled_status;
    right_motor->set_acceleration(new_enabled_status);
    left_motor->set_acceleration(new_enabled_status);

} // DifferentialPairedStepperMotors::SetAccelerationMode()

/******************************************************************************
* Name: SetRightAcceleration
*
* Description: Passed to StepperMotor instance so that stepper motor can handle
*              all the acceleration business and then call this function without
*              worrying about how motor control is accelerating motors.
******************************************************************************/
void SetRightAcceleration
    (
        int32_t acceleration
    )
{
    // Update acceleration timer interrupt so it will interrupt at a frequency
    // equal to input parameter 'acceleration'.
    timer.SetChannelFrequency(oc_channel_3, AbsoluteValue(acceleration));

} // DifferentialPairedStepperMotors::SetRightAcceleration()

/******************************************************************************
* Name: SetLeftAcceleration
*
* Description: Passed to StepperMotor instance so that stepper motor can handle
*              all the acceleration business and then call this function without
*              worrying about how motor control is accelerating motors.
******************************************************************************/
void SetLeftAcceleration
    (
        int32_t acceleration
    )
{
    // Update acceleration timer interrupt so it will interrupt at a frequency
    // equal to input parameter 'acceleration'.
    timer.SetChannelFrequency(oc_channel_4, AbsoluteValue(acceleration));

} // DifferentialPairedStepperMotors::SetLeftAcceleration()

/******************************************************************************
* Name: RightMotorStepper
*
* Description: Pulses right motor once to take a step (or microstep)
******************************************************************************/
void RightMotorStepper(void)
{
    right_motor_ref->Step();

} // RightMotorStepper

/******************************************************************************
* Name: LeftMotorStepper
*
* Description: Pulses left motor once to take a step (or microstep)
******************************************************************************/
void LeftMotorStepper(void)
{
    left_motor_ref->Step();

} // LeftMotorStepper

/******************************************************************************
* Name: RightMotorAccelerator
*
* Description: Updates the right motor stepper interrupt to reflect the new
* speed of the motor.
******************************************************************************/
void RightMotorAccelerator(void)
{
    static uint32_t current_right_freq = 0;
    uint32_t new_freq = right_motor_ref->UpdateSpeed();

    if (current_right_freq != new_freq)
    {
        // we have some new frequency we want to step the motors at so update timer interrupt
        timer.SetChannelFrequency(oc_channel_1, new_freq);
    }
    else
    {
        // We are now stepping at desired frequency so disable THIS interrupt until we need it again.
        timer.SetChannelFrequency(oc_channel_3, 0);
    }

    current_right_freq = new_freq;

} // RightMotorAccelerator

/******************************************************************************
* Name: LeftMotorAccelerator
*
* Description: Updates the left motor stepper interrupt to reflect the new
* speed of the motor.
******************************************************************************/
void LeftMotorAccelerator(void)
{
    static uint32_t current_left_freq = 0;
    uint32_t new_freq = left_motor_ref->UpdateSpeed();

    if (current_left_freq != new_freq)
    {
        // we have some new frequency we want to step the motors at so update timer interrupt
        timer.SetChannelFrequency(oc_channel_2, new_freq);
    }
    else
    {
        // We are now stepping at desired frequency so disable THIS interrupt until we need it again.
        timer.SetChannelFrequency(oc_channel_4, 0);
    }

    current_left_freq = new_freq;

} // LeftMotorAccelerator()
