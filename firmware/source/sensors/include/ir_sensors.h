/****************************************************************************************
* File: ir_sensor.h
*
* Description: Header file for ir_sensor.cpp
*
* Created: 3/29/2014, by Matthew Olson
****************************************************************************************/

#ifndef IR_SENSOR_INCLUDED_H
#define IR_SENSOR_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "distance_sensors_interface.h"
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

// Pin Definitions
#define IR_EMITER_GPIO                  GPIOC
#define IR_EMITER_PINS                  GPIO_Pin_0
#define IR_EMITER_AHBPERIPH_GPIO        RCC_AHB1Periph_GPIOC

#define IR_COLLECTOR_GPIO               GPIOA
#define IR_COLLECTOR_PINS               GPIO_Pin_1
#define IR_COLLECTOR_AHBPERIPH_GPIO     RCC_AHB1Periph_GPIOA

// If ADC changes, change DMA stream/channel
#define IR_ADC_APB                      RCC_APB2Periph_ADC1
#define IR_ADC                          ADC1

// Set up these channels to match order of sensors enum
#define IR_CHANNELS		        { ADC_Channel_1,            \
                                  ADC_Channel_2,            \
                                  ADC_Channel_3             }

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: ir_sensor
*
* Description: Controller for an inferred distance sensor.
******************************************************************************/
class ir_sensor : public IDistanceSensors
{
public: // methods

    ir_sensor();

    // Returns the distance measured by the sensor indicated in centimeters
    float ReadDistance
        (
            sensor_id_t  //Sensor to be read
        );

    // Starts one read for all of the sensors, every odd read will be a measurement of
    // ambiant light and every even read will be an actual read of distance
    void StartRead
        (
            void
        );

    // Saves one read for all of the sensors, every odd read will be a measurement of
    // ambiant light and every even read will be an actual read of distance
    void AcumulateData(void);

    // Initializes the hardware for ADC, DMA and the interupts
    void Init( void );

    // Initializes the DMA, this is set as public so that if a transoer error
    // occurs it can be reinitialized on the fly
    void InitDMA( void );

public: // fields

private: // methods

    // Initializes ADC hardware
    void InitADC( void );

    // Initializes Interrupts for ADC and DMA
    void InitInterrupts( void );

private: // fields

    bool reading_dist;
    uint16_t ambiant_light[number_of_sensors];
    uint16_t raw_readings[number_of_sensors];
    float rolling_average[number_of_sensors];
    uint32_t emiter_pins;
    GPIO_TypeDef* emiter_gpio;

}; // ir_sensor

//TODO: Temp make callback public so it can be reference untill we get timer_interrupt_oc support
void TimmerCallback( void );

#endif // IR_INCLUDED_H
