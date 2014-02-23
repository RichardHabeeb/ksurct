/***************************************************************************************
* File: pid.cpp
*
* Description: Implementation of a generic Proportional, Integral, Derivative controller
*              with saturation.
*
* Created: 02/01/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "pid.h"
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

/**********************************************************************
* Function: PID
*
* Description: Constructor
**********************************************************************/
PID::PID
    (
        float kp,                        // Proportional Gain
        float ki,                        // Integral Gain
        float kd,                        // Derivative Gain
        float saturation_high,           // Upper Output Saturation Limit
        float saturation_low,            // Lower Output Saturation Limit
        float integral_saturation_high,  // Upper Error Integral Saturation Limit
        float integral_saturation_low    // Lower Error Integral Saturation Limit
    ):
    kp(kp),
    ki(ki),
    kd(kd),
    saturation_high(saturation_high),
    saturation_low(saturation_low),
    integral_saturation_high(integral_saturation_high),
    integral_saturation_low(integral_saturation_low),
    error_sum(0),
    previous_error(0)
{
}// PID::PID() - Constructor

/*****************************************************************************
* Function: Calculate
*
* Description: Returns the controller output for a given error.  Derivative error is
*              calculated as a simple dx/dt.  If a derivative error is already
*              calculated then use overloaded method.
*****************************************************************************/
float PID::Calculate
    (
        float error,      // Difference between desired and actual measurement.
        float delta_time  // Change in time since last measurement. (seconds)
    )
{
    float derivative_error = 0.f;

    if (delta_time != 0.0f) // Avoid division by zero.
    {
        derivative_error = (error - previous_error) / delta_time;
    }

    return Calculate(error, derivative_error, delta_time);

} // PID::Calculate()

/*****************************************************************************
* Function: Calculate
*
* Description: Returns output of PID controller calculation using updated
*              error, derivative error and change in time since last run.
*****************************************************************************/
float PID::Calculate
    (
        float error,            // Difference between desired and actual measurement.
        float derivative_error, // Change in error over specified change in time.
        float delta_time        // Change in time since last measurement. (seconds)
    )
{
    error_sum += error * delta_time;

    // Ensure integral term is between saturation limits.
    error_sum = CapBounds(error_sum, integral_saturation_low, integral_saturation_high);

    // Calculate controller output from given inputs.
    float result = (kp * error) + (ki * error_sum) + (kd * derivative_error);

    // Ensure controller output is between saturation limits.
    result = CapBounds(result, saturation_low, saturation_high);

    // Store error so can calculate derivative error as dx/dt if needed.
    previous_error = error;

    return result;

} // PID::Calculate()
