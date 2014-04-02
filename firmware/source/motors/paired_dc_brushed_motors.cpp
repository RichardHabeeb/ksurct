/****************************************************************************************
* File: paired_dc_brushed_motors.cpp
*
* Description: Include file description here.
*
* Created: 03/31/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "gpio.h"
#include "paired_dc_brushed_motors.h"
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

const uint32_t output_pwm_frequency = 100;

/*---------------------------------------------------------------------------------------
*                                      VARIABLES
*--------------------------------------------------------------------------------------*/

// If individual motors are instantiated outside of class then we can move this out as well.

//TODO set these to the right pins.
// Pins for settings direction of each motors.
static OutputPin right_motor_direction_pin(GPIOC, GPIO_Pin_15, RCC_AHB1Periph_GPIOC);
static OutputPin left_motor_direction_pin(GPIOC, GPIO_Pin_14, RCC_AHB1Periph_GPIOC);

// A scaled frequency of 1 MHz will give a resolution of 1 micro-second and set the minimum
// PWM output frequency to around 15 Hz (it's a 16-bit timers).
static OutputPWMTimer pwm_timer(TIM3, RCC_APB1Periph_TIM3, SystemCoreClock, 1000000, output_pwm_frequency);

// Temporary to access motors from callback.  This shouldn't be here in release.
static DCBrushedMotor * right_motor_ref;
static DCBrushedMotor * left_motor_ref;

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Name: PairedDCBrushedMotors
*
* Description: Constructor
******************************************************************************/
PairedDCBrushedMotors::PairedDCBrushedMotors
    (
        float wheel_diameter,                // Centimeters
        float encoder_counts_per_revolution, // How many encoder counts make up one wheel rotation.
        float wheel_2_wheel_distance,        // AKA wheel base.  In centimeters.
        float angular_rate_per_duty          // Degrees / sec for a given % duty cycle.
    )
{
    this->wheel_diameter = wheel_diameter;
    this->wheel_2_wheel_distance = wheel_2_wheel_distance;
    this->angular_rate_per_duty = angular_rate_per_duty;
    this->encoder_counts_per_revolution = encoder_counts_per_revolution;

    this->wheel_circumference = wheel_diameter * PI;

    this->cm_per_encoder_count = wheel_circumference / encoder_counts_per_revolution;

    float wheel_2_wheel_circumference = wheel_2_wheel_distance * PI;
    this->cm_per_degree_zero_point = wheel_2_wheel_circumference / 360.f;

    // TODO maybe inject these through constructor instead of instantiating here.
    right_motor = new DCBrushedMotor(pwm_timer, pwm_channel_1, right_motor_direction_pin);
    left_motor  = new DCBrushedMotor(pwm_timer, pwm_channel_2, left_motor_direction_pin);

    // Set reference so we can access instances in encoder interrupts.
    right_motor_ref = right_motor;
    left_motor_ref  = left_motor;

} // PairedDCBrushedMotors::PairedDCBrushedMotors() - Constructor

/******************************************************************************
* Name: Initialize
*
* Description:
******************************************************************************/
void PairedDCBrushedMotors::Initialize(void)
{
    InitializeEncoders();

} // PairedDCBrushedMotors::Initialize()

/******************************************************************************
* Name: InitializeEncoders
*
* Description:
******************************************************************************/
void PairedDCBrushedMotors::InitializeEncoders(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;
    EXTI_InitTypeDef   EXTI_InitStructure;

    // Enable clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // Configure input capture pins
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0; // Port A, Right Motor
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // Port A, Left Motor
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Connect EXTI lines to input capture pins
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0); // Right Motor
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource2); // Left Motor

    // Configure EXTI lines
    EXTI_InitStructure.EXTI_Line = EXTI_Line0; // Right Motor
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line2; // Left Motor
    EXTI_Init(&EXTI_InitStructure);

    // Enable and set EXTI line interrupts to the highest priority
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; // Right Motor
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn; // Left Motor
    NVIC_Init(&NVIC_InitStructure);

} // PairedDCBrushedMotors::InitializeEncoders()

/******************************************************************************
* Name: ReInitialize
*
* Description: Clears motor control fields and re-init both motors.
******************************************************************************/
void PairedDCBrushedMotors::ReInitialize(void)
{
    right_motor->ReInitialize();
    left_motor->ReInitialize();

} // PairedDCBrushedMotors::ReInitialize()

/******************************************************************************
* Name: Drive
*
* Description: Set both motors to drive at specified speed.  If speed is negative
*              then motors will drive in reverse.
******************************************************************************/
void PairedDCBrushedMotors::Drive
    (
        float new_speed // cm / sec
    )
{
    DriveRight(new_speed);
    DriveLeft(new_speed);

    right_motor->reset_current_encoder_counts();
    left_motor->reset_current_encoder_counts();

} // PairedDCBrushedMotors::Drive()

/******************************************************************************
* Name: DriveRight
*
* Description: Set right motor to drive at specified speed.  If speed is negative
*              then motor will drive in reverse.
*
* Notes: Current encoder counts are NOT reset when calling this function.
******************************************************************************/
void PairedDCBrushedMotors::DriveRight
    (
        float new_speed // cm / sec
    )
{
    motor_direction_t drive_direction = (new_speed >= 0.f ? drive_forward : drive_backward);

    right_motor->Drive(drive_direction, ConvertSpeedToDutyCycle(new_speed));

} // PairedDCBrushedMotors::DriveRight()

/******************************************************************************
* Name: DriveLeft
*
* Description: Set left motor to drive at specified speed.  If speed is negative
*              then motor will drive in reverse.
*
* Notes: Current encoder counts are NOT reset when calling this function.
******************************************************************************/
void PairedDCBrushedMotors::DriveLeft
    (
        float new_speed // cm / sec
    )
{
    motor_direction_t drive_direction = (new_speed >= 0.f ? drive_forward : drive_backward);

    left_motor->Drive(drive_direction, ConvertSpeedToDutyCycle(new_speed));

} // PairedDCBrushedMotors::DriveLeft()

/******************************************************************************
* Name: ZeroPointTurn
*
* Description: Turn the robot the correct 'angle' in degrees in the direction
*              specified using a zero point turn approach
******************************************************************************/
void PairedDCBrushedMotors::ZeroPointTurn
    (
        motor_turn_t  new_direction,  // Which direction to turn.
        float         turn_angle,     // How far to turn in degrees
        float         turn_speed      // How fast to turn in degrees per second.
    )
{
    // Make sure both motors are stopped.
    Stop();

    // Zero point turn so don't want to update total encoder counts.
    right_motor->set_update_total_encoder_counts(false);
    left_motor->set_update_total_encoder_counts(false);

    // Determine number of centimeters to wait until turn will finish.
    float distance_to_turn = cm_per_degree_zero_point * turn_angle;

    float turn_duty_cycle = ConvertSpeedToDutyCycle(turn_speed);

    // In order to tell how far we've turned we need to reset current encoder_counts.
    right_motor->reset_current_encoder_counts();
    left_motor->reset_current_encoder_counts();

    if (new_direction == turn_right)
    {
        left_motor->Drive(drive_forward,   turn_duty_cycle);
        right_motor->Drive(drive_backward, turn_duty_cycle);
    }
    else if (new_direction == turn_left)
    {
        right_motor->Drive(drive_forward, turn_duty_cycle);
        left_motor->Drive(drive_backward, turn_duty_cycle);
    }

    // Wait for turn to finish.
    while (get_right_motor_current_distance() < distance_to_turn);

    Stop();

    // Done turning so renable updating total encoder_counts.
    right_motor->set_update_total_encoder_counts(true);
    left_motor->set_update_total_encoder_counts(true);

} // PairedDCBrushedMotors::ZeroPointTurn()

/******************************************************************************
* Name: Stop
*
* Description: Tells both motors they need to stops and wait here until they do.
******************************************************************************/
void PairedDCBrushedMotors::Stop(void)
{
    right_motor->Stop();
    left_motor->Stop();

    while (!Stopped());

} // PairedDCBrushedMotors::Stop()

/******************************************************************************
* Name: Stopped
*
* Description: Returns true only if both motors are stopped
******************************************************************************/
bool inline PairedDCBrushedMotors::Stopped(void)
{
    return (right_motor->IsStopped() && left_motor->IsStopped());

} // PairedDCBrushedMotors::Stopped()

/******************************************************************************
* Name: get_right_motor_current_distance
*
* Description: Returns the number of centimeters since last speed change.
******************************************************************************/
float PairedDCBrushedMotors::get_right_motor_current_distance(void)
{
    return right_motor->get_current_encoder_counts() * cm_per_encoder_count;

} // PairedDCBrushedMotors::get_right_motor_current_distance()

/******************************************************************************
* Name: get_left_motor_current_distance
*
* Description: Returns the number of centimeters since last speed change.
******************************************************************************/
float PairedDCBrushedMotors::get_left_motor_current_distance(void)
{
    return left_motor->get_current_encoder_counts() * cm_per_encoder_count;

} // PairedDCBrushedMotors::get_left_motor_current_distance()

/******************************************************************************
* Name: get_right_motor_total_distance
*
* Description: Returns the number of centimeters since the beginning of time.
******************************************************************************/
float PairedDCBrushedMotors::get_right_motor_total_distance(void)
{
    return right_motor->get_total_encoder_counts() * cm_per_encoder_count;

} // PairedDCBrushedMotors::get_right_motor_total_distance()

/******************************************************************************
* Name: get_left_motor_total_distance
*
* Description: Returns the number of centimeters since the beginning of time.
******************************************************************************/
float PairedDCBrushedMotors::get_left_motor_total_distance(void)
{
    return left_motor->get_total_encoder_counts() * cm_per_encoder_count;

} // PairedDCBrushedMotors::get_left_motor_total_distance()

/******************************************************************************
* Name: ConvertSpeedToDutyCycle
*
* Description: Returns duty cycle corresponding to specified linear speed in cm/sec.
******************************************************************************/
float PairedDCBrushedMotors::ConvertSpeedToDutyCycle
    (
        float speed // cm / sec
    )
{
    float angular_rate = AbsoluteValue(speed) / wheel_circumference * 360.f;
    float duty_cycle = angular_rate / angular_rate_per_duty;

    return duty_cycle;

} // PairedDCBrushedMotors::ConvertSpeedToDutyCycle()

/******************************************************************************
* Name: EXTI0_IRQHandler
*
* Description: External line interrupt for right motor encoder.
******************************************************************************/
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) == SET)
    {
        right_motor_ref->IncrementEncoderTicks();

        EXTI_ClearITPendingBit(EXTI_Line0);
    }

} // EXTI0_IRQHandler()

/******************************************************************************
* Name: EXTI2_IRQHandler
*
* Description: External line interrupt for right motor encoder.
******************************************************************************/
void EXTI2_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) == SET)
    {
        left_motor_ref->IncrementEncoderTicks();

        EXTI_ClearITPendingBit(EXTI_Line2);
    }

} // EXTI2_IRQHandler()