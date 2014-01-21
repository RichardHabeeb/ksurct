/****************************************************************************************
* File: gpio.cpp - General Purpose Input Output
*
* Description:
*
* Created: 01/17/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>

#include "stm32f4xx.h"
#include "gpio.h"

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
*                                  OUTPUT_PIN METHODS
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: OutputPin - Constructor
*
* Description:
*****************************************************************************/
OutputPin::OutputPin
    (
        GPIO_TypeDef * port,  // GPIOx where x[A:K]
        uint16_t       pin,   // GPIO_Pin_y where y[0:15]
        uint32_t       clock  // RCC_AHB1Periph_GPIOx where x[A:K]
    )
{
    this->port = port;
    this->pin  = pin;
    this->clock = clock;

} // OutputPin::OutputPin() - Constructor


/*****************************************************************************
* Function: Init
*
* Description:
*****************************************************************************/
void OutputPin::Init
    (
        pin_level_t pin_level // Starting level (high/low) of pin.
    )
{
    assert_param(IS_GPIO_ALL_PERIPH(this->port));
    assert_param(IS_GET_GPIO_PIN(this->pin));
    assert_param(IS_RCC_AHB1_CLOCK_PERIPH(this->clock));

    // Peripheral clock enable.
    RCC_AHB1PeriphClockCmd(this->clock, ENABLE);

    // Configure pin in output pushpull mode.
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = this->pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(this->port, &GPIO_InitStructure);

    // Initialize pin level and field.
    if (pin_level == HIGH)
    {
        WriteHigh();
    }
    else
    {
        WriteLow();
    }

} // OutputPin::Init()


/*---------------------------------------------------------------------------------------
*                                  INPUT_PIN METHODS
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: InputPin - Constructor
*
* Description:
*****************************************************************************/
InputPin::InputPin
    (
        GPIO_TypeDef * port,  // GPIOx where x[A:K]
        uint16_t       pin,   // GPIO_Pin_y where y[0:15]
        uint32_t       clock  // RCC_AHB1Periph_GPIOx where x[A:K]
    )
{
    this->port = port;
    this->pin  = pin;
    this->clock = clock;

} // InputPin::InputPin() - Constructor

/*****************************************************************************
* Function: Init
*
* Description:
*****************************************************************************/
void InputPin::Init(void)
{
    assert_param(IS_GPIO_ALL_PERIPH(this->port));
    assert_param(IS_GET_GPIO_PIN(this->pin));
    assert_param(IS_RCC_AHB1_CLOCK_PERIPH(this->clock));

    // Peripheral clock enable.
    RCC_AHB1PeriphClockCmd(this->clock, ENABLE);

    // Configure pin in input pushpull mode.
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = this->pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(this->port, &GPIO_InitStructure);

} // InputPin::Init()


/*---------------------------------------------------------------------------------------
*                          ALTERNATE_FUNCTION_PIN METHODS
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: AlternateFunctionPin - Constructor
*
* Description:
*****************************************************************************/
AlternateFunctionPin::AlternateFunctionPin
    (
        GPIO_TypeDef * port,                // GPIOx where x[A:K]
        uint16_t       pin,                 // GPIO_Pin_y where y[0:15]
        uint16_t       pin_source,          // GPIO_PinSourceX where X[0:15]
        uint8_t        alternate_function,  // GPIO_AF_x where x is some periph
        uint32_t       clock                // RCC_AHB1Periph_GPIOx where x[A:K]
    )
{
    this->port = port;
    this->pin  = pin;
    this->pin_source = pin_source;
    this->alternate_function = alternate_function;
    this->clock = clock;

} // AlternateFunctionPin::AlternateFunctionPin() - Constructor

/*****************************************************************************
* Function: Init
*
* Description:
*****************************************************************************/
void AlternateFunctionPin::Init(void)
{
    assert_param(IS_GPIO_ALL_PERIPH(this->port));
    assert_param(IS_GET_GPIO_PIN(this->pin));
    assert_param(IS_GPIO_PIN_SOURCE(this->pin_source));
    assert_param(IS_GPIO_AF(this->alternate_function));
    assert_param(IS_RCC_AHB1_CLOCK_PERIPH(this->clock));

    // Peripheral clock enable.
    RCC_AHB1PeriphClockCmd(this->clock, ENABLE);

    // Configure pin in alternate function mode.
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = this->pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(this->port, &GPIO_InitStructure);

    GPIO_PinAFConfig(this->port, this->pin_source, this->alternate_function);

} // AlternateFunctionPin::Init()
