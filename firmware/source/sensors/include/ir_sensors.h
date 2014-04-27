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

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/
// Identifies the robot
typedef enum
{
    BABY_KITTEN,
    POWERLION
} robot_identifier_t;

// Contains the offsets of the types of sensors to the center of the robot
// this is used for the constructor
typedef struct
{
    float side_offset;          // Offset from the front of the side sensor
    float diagnal_offset;       // Offset from the front of the diagnal sensor
    float front_offset;         // Offset from the front of the front sensor
} IRsensor_offset_t;

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
            robot_identifier_t, // The identity of the robot for pin definitions
            IRsensor_offset_t   // The offset of each sensor to the center of the robot
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

    // Returns the average read distance measurement of the right sensor in centimeters.
    float Calibrate(void);

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

private: // fields
    bool reading_dist;
    bool calibrating;
    bool calibration_data_ready;
    uint16_t ambiant_light[number_of_sensors];
    uint16_t raw_readings[number_of_sensors];
    float rolling_average[number_of_sensors];
    float offsets[number_of_sensors];
    uint32_t emiter_pins;
    GPIO_TypeDef* emiter_gpio;
    ADC_TypeDef * ADCx;
    robot_identifier_t this_robot;

}; // IRSensors

#endif // IR_INCLUDED_H
