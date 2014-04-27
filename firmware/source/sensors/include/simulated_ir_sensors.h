/****************************************************************************************
* File: simulated_ir_sensors.h
*
* Description: Header file for simulated_ir_sensors.cpp
*
* Created: 3/29/2014, by Kyle McGahee
****************************************************************************************/

#ifndef SIMULATED_IR_SENSORS_INCLUDED_H
#define SIMULATED_IR_SENSORS_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "distance_sensors_interface.h"
#include "maze.h"
#include "micromouse.h"

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
* Class: SimulatedIRSensors
*
* Description: TODO
******************************************************************************/
class SimulatedIRSensors : public IDistanceSensors
{
public: // methods

    // Default constructor
    SimulatedIRSensors(void);

    // Constructor
    SimulatedIRSensors
        (
            Maze             & maze,      // Maze with all walls already set.
            Micromouse const & micromouse // Reference to micromouse.
        );

    void Initialize(void);

    float Calibrate(void);


    // Returns distance to robots center point in centimeters for the given sensor id.
    float ReadDistance
        (
            sensor_id_t sensor_id
        );

    void set_maze(Maze & maze) { this->maze = &maze; }
    void set_micromouse(Micromouse & micromouse) { this->micromouse = &micromouse; }

    void set_front_sensor_id(sensor_id_t id) { this->front_sensor_id = id; }
    void set_right_sensor_id(sensor_id_t id) { this->right_sensor_id = id; }
    void set_left_sensor_id(sensor_id_t id)  { this->left_sensor_id  = id; }

private: // methods

private: // fields

    // Maze with all walls already set.
    Maze * maze;

    // Reference to robot in order to get current position/heading.
    Micromouse const * micromouse;

    // IDs assigned by client for indivual sensors.
    sensor_id_t front_sensor_id;
    sensor_id_t right_sensor_id;
    sensor_id_t left_sensor_id;

}; // SimulatedIRSensors

#endif // SIMULATED_IR_SENSORS_INCLUDED_H
