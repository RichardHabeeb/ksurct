/****************************************************************************************
* File: velocity_controller.h
*
* Description: Header file for velocity_controller.cpp
*
* Created: 4/20/2014, by Kyle McGahee
****************************************************************************************/

#ifndef VELOCITY_CONTROLLER_INCLUDED_H
#define VELOCITY_CONTROLLER_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "pid.h"
#include "motor_interface.h"

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
* Class: VelocityController
*
* Description: Put class description here.
******************************************************************************/
class VelocityController
{
public: // methods

    // Constructor
    VelocityController
        (
            IMotor * motor, // Motor to control.
            PID    * pid    // PID controller to use to control motor.
        );

    // TODO
    void Run
        (
            float delta_time // Change in time since last call.
        );

private: // fields

    float    last_distance;
    IMotor * motor;
    PID    * velocity_pid;

}; // VelocityController

#endif // VELOCITY_CONTROLLER_INCLUDED_H
