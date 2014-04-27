/****************************************************************************************
* File: simulated_ir_sensors.cpp
*
* Description: TODO
*
* Created: 3/29/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "simulated_ir_sensors.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                   MEMORY CONSTANTS
---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: SimulatedIRSensors
*
* Description: Default Constructor
*****************************************************************************/
SimulatedIRSensors::SimulatedIRSensors(void)
{

} // SimulatedIRSensors::SimulatedIRSensors() - Default Constructor

/*****************************************************************************
* Function: SimulatedIRSensors
*
* Description: Constructor
*****************************************************************************/
SimulatedIRSensors::SimulatedIRSensors
    (
        Maze             & maze,      // Maze with all walls already set.
        Micromouse const & micromouse // Reference to micromouse.
    ) :
    maze(&maze),
    micromouse(&micromouse)
{
    SimulatedIRSensors();

} // SimulatedIRSensors::SimulatedIRSensors() - Constructor

/*****************************************************************************
* Function: Initialize
*
* Description:
*****************************************************************************/
void SimulatedIRSensors::Initialize(void)
{

} // SimulatedIRSensors::Initialize()


float SimulatedIRSensors::Calibrate( void )
{
    return maze->get_cell_length() / 2.0f;
}

/*****************************************************************************
* Function: ReadDistance
*
* Description: Uses pre-mapped maze to return distance to robots center point
*              in centimeters for the given sensor id.
*****************************************************************************/
float SimulatedIRSensors::ReadDistance
    (
        sensor_id_t sensor_id
    )
{
    //float forward_angle = micromouse->get_foward_angle();
    float x_distance  = micromouse->get_x_distance();
    float y_distance  = micromouse->get_y_distance();
    float cell_length = maze->get_cell_length();

    position_t robot_position = micromouse->get_position();
    heading_t  robot_heading  = micromouse->get_heading();

    uint8_t open_cells = 0;
    float distance_to_next_cell = 0.f;

    heading_t sensor_heading = robot_heading;
    if (sensor_id == sensor_id_right)
    {
        sensor_heading = ConvertToHeading(right, robot_heading);
    }
    else if (sensor_id == sensor_id_left)
    {
        sensor_heading = ConvertToHeading(left, robot_heading);
    }

    if (sensor_heading == north || sensor_heading == south)
    {
        distance_to_next_cell = cell_length - (y_distance - cell_length * robot_position.y);
    }
    else
    {
        distance_to_next_cell = cell_length - (x_distance - cell_length * robot_position.x);
    }

    position_t current_position = robot_position;

    while (!maze->get_cell(current_position)->IsWall(sensor_heading))
    {
        open_cells++;

        switch (sensor_heading)
        {
            case north:   current_position.y--;    break;
            case east:    current_position.x++;    break;
            case south:   current_position.y++;    break;
            case west:    current_position.x--;    break;
        }
    }

    return distance_to_next_cell + maze->get_cell_length() * open_cells;

} // SimulatedIRSensors::ReadDistance()
