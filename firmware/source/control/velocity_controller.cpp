/****************************************************************************************
* File: velocity_controller.cpp
*
* Description: Include file description here.
*
* Created: 4/20/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "velocity_controller.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                   MEMORY CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      VARIABLES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: VelocityController
*
* Description: Constructor
*****************************************************************************/
VelocityController::VelocityController
    (
        IMotor * motor, // Motor to control.
        PID    * pid    // PID controller to use to control motor.
    )
{
    this->motor = motor;
    this->velocity_pid = pid;
    this->last_distance = 0;

} // VelocityController::VelocityController() - Constructor

/*****************************************************************************
* Function: Run
*
* Description:
*****************************************************************************/
void VelocityController::Run
    (
        float delta_time // Change in time since last call.
    )
{
    float current_distance = motor->get_total_distance();

    float change_in_distance = current_distance - last_distance;

    float measured_velocity = change_in_distance / delta_time;

    float error = motor->get_commanded_velocity() - measured_velocity;

    float delta_velocity = velocity_pid->Calculate(error, delta_time);

    motor->AddVelocity(delta_velocity);

    last_distance = current_distance;

} // VelocityController::Run()
