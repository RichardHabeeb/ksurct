/****************************************************************************************
* File: led_references.h
*
* Description: Provides access to directional and indicator LEDs.
*
* Created: 05/07/2014, by Kyle McGahee
****************************************************************************************/

#ifndef LED_REFERENCES_INCLUDED_H
#define LED_REFERENCES_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "gpio.h"

/*---------------------------------------------------------------------------------------
*                                    LED REFERENCES
*--------------------------------------------------------------------------------------*/

extern OutputPin * front_directional_led;
extern OutputPin * right_directional_led;
extern OutputPin * back_directional_led;
extern OutputPin * left_directional_led;

extern OutputPin * indicator_1_led;
extern OutputPin * indicator_2_led;
extern OutputPin * indicator_3_led;
extern OutputPin * indicator_4_led;

#endif // LED_REFERENCES_INCLUDED_H
