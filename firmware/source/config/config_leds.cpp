/****************************************************************************************
* File: config_leds.cpp
*
* Description: Sets up correct GPIO pins for output LEDs.  Any timer LEDs are setup outside
*              this file. To use LEDs just reference the correct header.
*
* Created: 05/07/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <stddef.h>

#include "config_settings.h"
#include "gpio.h"
#include "led_references.h"
#include "private_system_config.h"
#include "util_assert.h"

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
                                    LED REFERENCES
---------------------------------------------------------------------------------------*/

OutputPin * front_directional_led = NULL;
OutputPin * right_directional_led = NULL;
OutputPin * back_directional_led  = NULL;
OutputPin * left_directional_led  = NULL;

OutputPin * indicator_1_led = NULL;
OutputPin * indicator_2_led = NULL;
OutputPin * indicator_3_led = NULL;
OutputPin * indicator_4_led = NULL;

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: configure_leds
*
* Description: Sets up correct GPIO pins for output LEDs.  Any timer LEDs are setup elsewhere.
*              To use LEDs just reference the correct header.
*****************************************************************************/
void configure_leds(void)
{
    if (ROBOT == BABY_KITTEN)
    {
        front_directional_led = new OutputPin(GPIOC, GPIO_Pin_1, RCC_AHB1Periph_GPIOC);
        right_directional_led = new OutputPin(GPIOC, GPIO_Pin_2, RCC_AHB1Periph_GPIOC);
        back_directional_led  = new OutputPin(GPIOC, GPIO_Pin_3, RCC_AHB1Periph_GPIOC);
        left_directional_led  = new OutputPin(GPIOC, GPIO_Pin_0, RCC_AHB1Periph_GPIOC);

        indicator_1_led = new OutputPin(GPIOC, GPIO_Pin_10, RCC_AHB1Periph_GPIOC);
        indicator_2_led = new OutputPin(GPIOC, GPIO_Pin_11, RCC_AHB1Periph_GPIOC);
        indicator_3_led = new OutputPin(GPIOC, GPIO_Pin_12, RCC_AHB1Periph_GPIOC);
        indicator_4_led = new OutputPin(GPIOD, GPIO_Pin_2,  RCC_AHB1Periph_GPIOD);

        // Baby kitten has its power led hooked up to an IO pin so need to turn it on.
        static OutputPin power_led(GPIOC, GPIO_Pin_13, RCC_AHB1Periph_GPIOC);
        power_led.Init(HIGH);
    }
    else if (ROBOT == POWER_LION)
    {
        front_directional_led = new OutputPin(GPIOH, GPIO_Pin_0, RCC_AHB1Periph_GPIOH);
        right_directional_led = new OutputPin(GPIOH, GPIO_Pin_1, RCC_AHB1Periph_GPIOH);
        back_directional_led  = new OutputPin(GPIOC, GPIO_Pin_0, RCC_AHB1Periph_GPIOC);
        left_directional_led  = new OutputPin(GPIOC, GPIO_Pin_1, RCC_AHB1Periph_GPIOC);

        indicator_1_led = new OutputPin(GPIOC, GPIO_Pin_10, RCC_AHB1Periph_GPIOC);
        indicator_2_led = new OutputPin(GPIOC, GPIO_Pin_11, RCC_AHB1Periph_GPIOC);
        indicator_3_led = new OutputPin(GPIOC, GPIO_Pin_12, RCC_AHB1Periph_GPIOC);
        indicator_4_led = new OutputPin(GPIOD, GPIO_Pin_2,  RCC_AHB1Periph_GPIOD);
    }
    else
    {
        assert_always_msg(ASSERT_STOP, "Invalid robot selected.");
    }

    front_directional_led->Init(LOW);
    right_directional_led->Init(LOW);
    back_directional_led->Init(LOW);
    left_directional_led->Init(LOW);

    indicator_1_led->Init(LOW);
    indicator_2_led->Init(LOW);
    indicator_3_led->Init(LOW);
    indicator_4_led->Init(LOW);

} // configure_leds()
