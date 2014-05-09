/****************************************************************************************
* File: distance_sensors_interface.h
*
* Description: Interface to sensor management class capable of reading in distace values.
*
* Created: 3/30/2014, by Kyle McGahee
****************************************************************************************/

#ifndef DISTANCE_SENSORS_INTERFACE_INCLUDED_H
#define DISTANCE_SENSORS_INTERFACE_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

enum sensor_id_t
{
    sensor_id_left,
    sensor_id_front_nw,
    sensor_id_front,
    sensor_id_front_ne,
    sensor_id_right,

    number_of_sensors
};

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: IDistanceSensors
*
* Description: Interface to sensor management class capable of reading in distace
*              values in centimeters.
******************************************************************************/
class IDistanceSensors
{
public: // methods

    virtual void Initialize(void) = 0;

    // Returns distance measurement of specified sensor in centimeters.
    virtual float ReadDistance
        (
            sensor_id_t sensor_id
        ) = 0;

    // Calibrates the specified sensor to match known distance.
    virtual void CalibrateSensor
        (
            sensor_id_t sensor_id,
            float known_distance
        ) = 0;

    // Returns if the sensor is saturated by checking if the reading
    // is near the logic voltage.
    virtual bool IsSaturated
        (
            sensor_id_t sensor_id   // Sensor to check
        ) = 0;
}; // IDistanceSensors

#endif // DISTANCE_SENSORS_INTERFACE_INCLUDED_H
