/****************************************************************************************
* File: ir_sensors.h
*
* Description: Header file for ir_sensors.cpp
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

// Pin Definitions for front Power Lion
#define IR_EMITER_GPIO                  GPIOB
#define IR_EMITER_PINS                ( GPIO_Pin_11     \
                                      | GPIO_Pin_12     \
                                      | GPIO_Pin_13     \
                                      | GPIO_Pin_14     \
                                      | GPIO_Pin_15 )

#define IR_EMITER_AHBPERIPH_GPIO        RCC_AHB1Periph_GPIOB


#define IR_COLLECTOR2_GPIO               GPIOC
#define IR_COLLECTOR2_PINS               GPIO_Pin_4
#define IR_COLLECTOR2_AHBPERIPH_GPIO     RCC_AHB1Periph_GPIOC

#define IR_COLLECTOR1_GPIO               GPIOA
#define IR_COLLECTOR1_PINS             ( GPIO_Pin_7     \
                                       | GPIO_Pin_6     \
                                       | GPIO_Pin_5     \
                                       | GPIO_Pin_4 )
#define IR_COLLECTOR1_AHBPERIPH_GPIO     RCC_AHB1Periph_GPIOA


// If ADC changes, change DMA stream/channel
#define IR_ADC_APB                      RCC_APB2Periph_ADC1
#define IR_ADC                          ADC1

// Set up these channels to match order of sensors enum
#define IR_CHANNELS		        { ADC_Channel_14,       \
                                  ADC_Channel_6,        \
                                  ADC_Channel_4 }
//MTO Only reading sensors for left front and right
//  save setup for all sensors
                              // {ADC_Channel_14,        \
                                  ADC_Channel_7,         \
                                  ADC_Channel_6,         \
                                  ADC_Channel_5,         \
                                  ADC_Channel_4 }

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: IRSensors
*
* Description: Controller for an inferred distance sensor.
******************************************************************************/
class IRSensors : public IDistanceSensors
{
public: // methods

    IRSensors(void);

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

}; // IRSensors

#endif // IR_INCLUDED_H
