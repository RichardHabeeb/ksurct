/****************************************************************************************
* File: gpio.h
*
* Description: General Purpose Input/Output class
*
* Created: 01/17/2014, by Kyle McGahee
****************************************************************************************/

#ifndef GPIO_INCLUDED_H
#define GPIO_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

typedef uint8_t pin_level_t;
enum
{
    LOW,
    HIGH,
};

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: OutputPin
*
* Description: Put class description here.
******************************************************************************/
class OutputPin
{
public: // methods

    // Constructor
    OutputPin
        (
            GPIO_TypeDef * port,  // GPIOx where x[A:K]
            uint16_t       pin,   // GPIO_Pin_y where y[0:15]
            uint32_t       clock  // RCC_AHB1Periph_GPIOx where x[A:K]
        );

    void Init
        (
            pin_level_t pin_level // Starting level (high/low) of pin.
        );

    void WriteHigh(void)
    {
        this->pin_level = HIGH;
        this->port->BSRRL = this->pin; // Write pin high.
    }

    void WriteLow(void)
    {
        this->pin_level = LOW;
        this->port->BSRRH = this->pin; // Write pin low.
    }

    void Toggle(void)
    {
        if (this->pin_level == HIGH)
        {
            WriteLow();
        }
        else
        {
            WriteHigh();
        }
    }

private: // fields

    uint16_t       pin;
    GPIO_TypeDef * port;
    uint32_t       clock;
    pin_level_t    pin_level;

}; // OutputPin

/******************************************************************************
* Class: InputPin
*
* Description: Put class description here.
******************************************************************************/
class InputPin
{
public: // methods

    // Constructor
    InputPin
        (
            GPIO_TypeDef * port,  // GPIOx where x[A:K]
            uint16_t       pin,   // GPIO_Pin_y where y[0:15]
            uint32_t       clock  // RCC_AHB1Periph_GPIOx where x[A:K]
        );

    void Init(void);

    pin_level_t Read(void)
    {
        if ((this->port->IDR & this->pin) != (uint32_t)Bit_RESET)
        {
            return HIGH;
        }
        else
        {
            return LOW;
        }
    }

private: // fields

    uint16_t       pin;
    GPIO_TypeDef * port;
    uint32_t       clock;

}; // InputPin


/******************************************************************************
* Class: AlternateFunctionPin
*
* Description: Put class description here.
******************************************************************************/
class AlternateFunctionPin
{
public: // methods

    // Constructor
    AlternateFunctionPin
        (
            GPIO_TypeDef * port,                // GPIOx where x[A:K]
            uint16_t       pin,                 // GPIO_Pin_y where y[0:15]
            uint16_t       pin_source,          // GPIO_PinSourceX where X[0:15]
            uint8_t        alternate_function,  // GPIO_AF_x where x is some periph
            uint32_t       clock                // RCC_AHB1Periph_GPIOx where x[A:K]
        );

    void Init(void);

private: // fields

    uint16_t       pin;
    uint16_t       pin_source;
    GPIO_TypeDef * port;
    uint32_t       clock;
    uint8_t        alternate_function;

}; // AlternateFunctionPin

#endif  // GPIO_INCLUDED_H
