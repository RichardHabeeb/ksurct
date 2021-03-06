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

#include "config_settings.h"
#include "distance_sensors_interface.h"
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

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

    IRSensors
        (
            robot_identifier_t // The identity of the robot for pin definitions
        );

    // Returns the distance measured by the sensor indicated in centimeters
    float ReadDistance
        (
            sensor_id_t  // Sensor to be read
        );

    // Starts one read for all of the sensors, every odd read will be a measurement of
    // ambiant light and every even read will be an actual read of distance
    void StartRead(void);

    // Saves one read for all of the sensors, every odd read will be a measurement of
    // ambiant light and every even read will be an actual read of distance
    void AcumulateData(void);

    // Initializes the hardware for ADC, DMA and the interupts
    void Initialize(void);

    // Calculates difference between measured distance and the specified known distance
    // that sensor should be at.  This error offset will be calculated and added in for
    // every future reading of the sensor.
    void CalibrateSensor( sensor_id_t sensor_id, float known_distance );

    // Returns whether or not the sensors are calibrating
    bool isCalibrating(void);

    // Initializes the DMA, this is set as public so that if a transoer error
    // occurs it can be reinitialized on the fly
    void InitDMA(void);

public: // fields

private: // methods

    // Initializes ADC hardware
    void InitADC( void );

    // Initializes Interrupts for ADC and DMA
    void InitInterrupts( void );

    // Returns distance in centimeters corresponding to specified ADC value.
    // No calibration offsets are added in.
    inline float ConvertToDistance( float adc_value );

private: // fields

    bool reading_dist;
    bool calibrating;
    bool calibration_data_ready;
    uint16_t ambiant_light[number_of_sensors];
    uint16_t raw_readings[number_of_sensors];
    float rolling_average[number_of_sensors];
    float calibration_offsets[number_of_sensors];
    uint32_t emiter_pins;
    GPIO_TypeDef* emiter_gpio;
    ADC_TypeDef * ADCx;
    robot_identifier_t this_robot;

}; // IRSensors

#endif // IR_INCLUDED_H
