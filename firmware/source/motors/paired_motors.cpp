/****************************************************************************************
* File: paired_motors.cpp
*
* Description: Include file description here.
*
* Created: 04/20/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <cmath>

#include "gpio.h"
#include "paired_motors.h"
#include "timer_interrupt_oc.h"
#include "util_math.h"

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

/******************************************************************************
* Name: PairedMotors
*
* Description: Constructor
******************************************************************************/
PairedMotors::PairedMotors
    (
        IMotor & right_motor,           // From driver perspective.
        IMotor & left_motor,            // From driver perspective.
        float    wheel_2_wheel_distance // AKA wheel base.  In centimeters.
    )
{
    this->right_motor = &right_motor;
    this->left_motor  = &left_motor;

    this->wheel_2_wheel_distance = wheel_2_wheel_distance;
    float wheel_2_wheel_circumference = wheel_2_wheel_distance * PI;
    this->cm_per_degree_zero_point = wheel_2_wheel_circumference / 360.f;

} // PairedMotors::PairedMotors() - Constructor

/******************************************************************************
* Name: Initialize
*
* Description: Initializes both motors.
******************************************************************************/
void PairedMotors::Initialize(void)
{
    right_motor->Initialize();
    left_motor->Initialize();

} // PairedMotors::Initialize()

/******************************************************************************
* Name: Drive
*
* Description: Set both motors to drive at specified velocity. If speed is negative
*              then motors will drive in reverse.
******************************************************************************/
void PairedMotors::Drive
    (
        float new_velocity // cm / sec
    )
{
    right_motor->Drive(new_velocity);
    left_motor->Drive(new_velocity);

} // PairedMotors::Drive()

/******************************************************************************
* Name: ZeroPointTurn
*
* Description: Turn the motors the correct 'angle' in degrees in the direction
*              specified using a zero point turn approach
******************************************************************************/
void PairedMotors::ZeroPointTurn
    (
        motor_turn_t  new_direction,  // Which direction to turn.
        float         turn_angle,     // How far to turn in degrees
        float         turn_speed      // How fast to turn in cm per second.
    )
{
    // Make sure both motors are stopped.
    Stop();

    // Determine number of centimeters to wait until turn will finish.
    float distance_to_turn = cm_per_degree_zero_point * turn_angle;

    // In order to tell how far we've turned we need to reset current distance.
    right_motor->reset_current_distance();
    left_motor->reset_current_distance();

    if (new_direction == turn_right)
    {
        left_motor->Drive(turn_speed);
        right_motor->Drive(-turn_speed);
    }
    else if (new_direction == turn_left)
    {
        right_motor->Drive(turn_speed);
        left_motor->Drive(-turn_speed);
    }

    // Wait for turn to finish.
    float center_robot_distance_travelled = 0.f;
    while (center_robot_distance_travelled < distance_to_turn)
    {
        center_robot_distance_travelled = (fabs(right_motor->get_current_distance()) +
                                           fabs(left_motor->get_current_distance()))
                                           / 2.f;
    }

    Stop();

} // PairedMotors::ZeroPointTurn()

/******************************************************************************
* Name: ArcTurn
*
* Description: Turn the robot the correct 'angle' in degrees in the direction
*              specified using a moving turn approach along an arc
******************************************************************************/
void PairedMotors::ArcTurn
    (
        motor_turn_t  new_direction,  // Which direction to turn.
        float         turn_angle,     // How far to turn in degrees
        float         turn_radius     // Point from center of robot to turn about.
    )
{
    float distance_to_turn = turn_radius * (turn_angle * PI / 180.f);

    // Reset the current distance so we know how far we have turned
    right_motor->reset_current_distance();
    left_motor->reset_current_distance();

    float arc_turn_factor = (((turn_radius + wheel_2_wheel_distance / 2.f)
                                          / turn_radius) - 1.f);

    float current_center_speed = (right_motor->get_commanded_velocity() +
                            left_motor->get_commanded_velocity()) / 2.f;

    // Velocity difference between wheel and velocity of robot
    float delta_velocity = current_center_speed * arc_turn_factor;

    if (new_direction == turn_right)
    {
        left_motor->AddVelocity(delta_velocity);
        right_motor->AddVelocity(-delta_velocity);
    }
    else if (new_direction == turn_left)
    {
        right_motor->AddVelocity(delta_velocity);
        left_motor->AddVelocity(-delta_velocity);
    }

    float distance_travelled = 0.f;

    // Wait until you've finished your turn
    while (distance_travelled < distance_to_turn)
    {
        distance_travelled = (right_motor->get_current_distance() +
                                left_motor->get_current_distance()) / 2.f;
    }

} // PairedMotors::ArcTurn()

/******************************************************************************
* Name: Stop
*
* Description: Tells both motors they need to stop and wait here until they do.
******************************************************************************/
void PairedMotors::Stop(void)
{
    right_motor->Stop();
    left_motor->Stop();

    while (!Stopped());

} // PairedMotors::Stop()

/******************************************************************************
* Name: Stopped
*
* Description: Returns true only if both motors are stopped
******************************************************************************/
bool inline PairedMotors::Stopped(void)
{
    return (right_motor->IsStopped() && left_motor->IsStopped());

} // PairedMotors::Stopped()

/******************************************************************************
* Name: reset_current_distance
*
* Description: Resets current distance measurement for both motors.
******************************************************************************/
void PairedMotors::reset_current_distance(void)
{
    right_motor->reset_current_distance();
    left_motor->reset_current_distance();

} // PairedMotors::reset_current_distance()
