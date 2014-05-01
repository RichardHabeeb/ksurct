/****************************************************************************************
* File: config_motors.cpp
*
* Description: Creates correct motors to use based on configuration settings.
*
* Created: 04/20/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <stddef.h>

#include "config_settings.h"
#include "dc_brushed_motor.h"
#include "motor_interface.h"
#include "paired_motors.h"
#include "pid.h"
#include "private_system_config.h"
#include "motor_callbacks.h"
#include "timer_interrupt_oc.h"
#include "stepper_motor.h"
#include "util_math.h"
#include "velocity_controller.h"

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
                                    MEMORY CONSTANTS
---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

// Forward declarations.
static void configure_stepper_motors(IMotor ** right_motor, IMotor ** left_motor);
static void configure_dc_brushed_motors(IMotor ** right_motor, IMotor ** left_motor);
static void configure_dc_brushed_encoders(void);

/*****************************************************************************
* Function: configure_motors
*
* Description: Configures correct motors to use based on configuration settings.
*****************************************************************************/
void configure_motors
    (
        IMotor ** right_motor,
        IMotor ** left_motor
    )
{
    if (MOTOR_TYPE == DC_BRUSHED_MOTOR_TYPE)
    {
        configure_dc_brushed_motors(right_motor, left_motor);
    }
    else if (MOTOR_TYPE == STEPPER_MOTOR_TYPE)
    {
        configure_stepper_motors(right_motor, left_motor);
    }

} // configure_motors()

/*****************************************************************************
* Function: configure_stepper_motors
*
* Description: Instantiates stepper motors with given settings.
*****************************************************************************/
static void configure_stepper_motors
    (
        IMotor ** right_motor,
        IMotor ** left_motor
    )
{
    // Could wrap these in a struct.
    float   wheel_diameter             = WHEEL_DIAMETER;
    float   full_steps_per_revolution  = FULL_STEPS_PER_REVOLUTION;
    uint8_t pulses_per_step            = PULSES_PER_STEP;
    bool    acceleration_enabled       = ACCELERATION_ENABLED;
    float   acceleration_time_constant = ACCELERATION_TIME_CONSTANT;
    float   acceleration_ref_speed     = ACCELERATION_REF_SPEED;
    float   velocity_update_inc        = VELOCITY_UPDATE_INCREMENT;

    // Calculate acceleration in full steps per seconds^2.
    float acceleration = AbsoluteValue(acceleration_ref_speed) / acceleration_time_constant;
    acceleration *= full_steps_per_revolution / (wheel_diameter * PI);

    // Instantiate output compare timer to step the motors (still need to be initialized)
    static TimerInterruptOC timer(TIM3, TIM3_IRQn, RCC_APB1Periph_TIM3, (SystemCoreClock/4), 50000);

    // Instantiate GPIO motor pins (still need to be initialized)
    static OutputPin right_motor_step_pin(GPIOC, GPIO_Pin_7, RCC_AHB1Periph_GPIOC);
    static OutputPin left_motor_step_pin(GPIOC,  GPIO_Pin_6, RCC_AHB1Periph_GPIOC);

    static OutputPin right_motor_direction_pin(GPIOB, GPIO_Pin_12, RCC_AHB1Periph_GPIOB);
    static OutputPin left_motor_direction_pin(GPIOB,  GPIO_Pin_11, RCC_AHB1Periph_GPIOB);

    // Motor select pins (controls microstepping)
    static OutputPin right_motor_ms1_pin(GPIOB, GPIO_Pin_15, RCC_AHB1Periph_GPIOB);
    static OutputPin right_motor_ms2_pin(GPIOB, GPIO_Pin_14, RCC_AHB1Periph_GPIOB);
    static OutputPin right_motor_ms3_pin(GPIOB, GPIO_Pin_13, RCC_AHB1Periph_GPIOB);
    static OutputPin left_motor_ms1_pin(GPIOB,  GPIO_Pin_1,  RCC_AHB1Periph_GPIOB);
    static OutputPin left_motor_ms2_pin(GPIOB,  GPIO_Pin_2,  RCC_AHB1Periph_GPIOB);
    static OutputPin left_motor_ms3_pin(GPIOB,  GPIO_Pin_10, RCC_AHB1Periph_GPIOB);

    StepperMotor * right_stepper_motor = new StepperMotor(right_motor_step_pin,
                                                          right_motor_direction_pin,
                                                          right_motor_ms1_pin,
                                                          right_motor_ms2_pin,
                                                          right_motor_ms3_pin,
                                                          wheel_diameter,
                                                          full_steps_per_revolution,
                                                          pulses_per_step,
                                                          acceleration_enabled,
                                                          acceleration,
                                                          velocity_update_inc,
                                                          SetRightAcceleration);

    StepperMotor * left_stepper_motor = new StepperMotor(left_motor_step_pin,
                                                         left_motor_direction_pin,
                                                         left_motor_ms1_pin,
                                                         left_motor_ms2_pin,
                                                         left_motor_ms3_pin,
                                                         wheel_diameter,
                                                         full_steps_per_revolution,
                                                         pulses_per_step,
                                                         acceleration_enabled,
                                                         acceleration,
                                                         velocity_update_inc,
                                                         SetLeftAcceleration);

    InitializeStepperMotorCallbacks(right_stepper_motor,
                                    left_stepper_motor,
                                    &timer);

    *right_motor = right_stepper_motor;
    *left_motor  = left_stepper_motor;

} // configure_stepper_motors()

/*****************************************************************************
* Function: configure_dc_brushed_motors
*
* Description: Instantiates DC brushed motors with given settings.
*****************************************************************************/
static void configure_dc_brushed_motors
    (
        IMotor ** right_motor,
        IMotor ** left_motor
    )
{
    // Could wrap these in a struct.
    float wheel_diameter                = WHEEL_DIAMETER;
    float encoder_counts_per_revolution = ENCODER_TICKS_PER_REVOLUTION;
    float angular_rate_per_duty         = ANGULAR_RATE_PER_DUTY;
    float velocity_controller_rate      = VELOCITY_CONTROLLER_RATE;

    // Can't make this frequency too high or the motor controller won't output the right voltage.
    const uint32_t output_pwm_frequency = 200;

    // Pins for settings direction of each motors.
    static OutputPin right_motor_direction_pin(GPIOC, GPIO_Pin_15, RCC_AHB1Periph_GPIOC);
    static OutputPin left_motor_direction_pin(GPIOC , GPIO_Pin_14, RCC_AHB1Periph_GPIOC);

    // A scaled frequency of 1 MHz will give a resolution of 1 micro-second and set the minimum
    // PWM output frequency to around 15 Hz (it's a 16-bit timers).
    static OutputPWMTimer pwm_timer(TIM3, RCC_APB1Periph_TIM3, SystemCoreClock/6, 1000000, output_pwm_frequency);

    static AlternateFunctionPin right_motor_pwm_pin(GPIOC, GPIO_Pin_6, GPIO_PinSource6, GPIO_AF_TIM3, RCC_AHB1Periph_GPIOC);
    static AlternateFunctionPin left_motor_pwm_pin(GPIOC,  GPIO_Pin_7, GPIO_PinSource7, GPIO_AF_TIM3, RCC_AHB1Periph_GPIOC);

    static AlternateFunctionPin right_motor_indicator_pin(GPIOC, GPIO_Pin_8, GPIO_PinSource8, GPIO_AF_TIM3, RCC_AHB1Periph_GPIOC);
    static AlternateFunctionPin left_motor_indicator_pin(GPIOC,  GPIO_Pin_9, GPIO_PinSource8, GPIO_AF_TIM3, RCC_AHB1Periph_GPIOC);

    // For now just initialize the indicator pins at 50% and leave them alone.
    pwm_timer.InitChannel(pwm_channel_3, right_motor_indicator_pin);
    pwm_timer.InitChannel(pwm_channel_4, left_motor_indicator_pin);
    pwm_timer.SetChannelDutyCycle(pwm_channel_3, .5);
    pwm_timer.SetChannelDutyCycle(pwm_channel_4, .5);

    // Must run velocity control at a set frequency so use a timer for callbacks.
    static TimerInterruptOC velocity_control_timer(TIM5, TIM5_IRQn, RCC_APB1Periph_TIM5, (SystemCoreClock/4), 50000);

    DCBrushedMotor * right_brushed_motor = new DCBrushedMotor(right_motor_pwm_pin,
                                                              right_motor_direction_pin,
                                                              pwm_timer,
                                                              pwm_channel_1,
                                                              wheel_diameter,
                                                              encoder_counts_per_revolution,
                                                              angular_rate_per_duty);

    DCBrushedMotor * left_brushed_motor = new DCBrushedMotor(left_motor_pwm_pin,
                                                             left_motor_direction_pin,
                                                             pwm_timer,
                                                             pwm_channel_2,
                                                             wheel_diameter,
                                                             encoder_counts_per_revolution,
                                                             angular_rate_per_duty);

    // Velocity controller paramters.
    float kp = 0.f;
    float ki = 0.f;
    float kd = 0.f;
    float saturation_high = 100.f;
    float saturation_low  = 100.f;
    float integral_saturation_high = 100.f;
    float integral_saturation_low  = 100.f;

    PID * right_motor_velocity_pid = new PID(kp, ki, kd, saturation_high, saturation_low,
                                             integral_saturation_high, integral_saturation_low);

    PID * left_motor_velocity_pid = new PID(kp, ki, kd, saturation_high, saturation_low,
                                            integral_saturation_high, integral_saturation_low);

    VelocityController * right_velocity_controller = new VelocityController(right_brushed_motor,
                                                                            right_motor_velocity_pid);

    VelocityController * left_velocity_controller = new VelocityController(left_brushed_motor,
                                                                           left_motor_velocity_pid);

    configure_dc_brushed_encoders();

    InitializeDCBrushedMotorCallbacks(right_brushed_motor,
                                      left_brushed_motor,
                                      right_velocity_controller,
                                      left_velocity_controller,
                                      &velocity_control_timer,
                                      velocity_controller_rate);

    *right_motor = right_brushed_motor;
    *left_motor  = left_brushed_motor;

} // configure_dc_brushed_motors()

/*****************************************************************************
* Function: configure_dc_brushed_encoders
*
* Description: Sets up external interrupt pins for DC brushed motor encoders.
*****************************************************************************/
static void configure_dc_brushed_encoders(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

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

} // configure_dc_brushed_encoders()
