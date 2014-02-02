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
    error_sum(0)
{
}// PID::PID() - Constructor

/*****************************************************************************
* Function: Calculate
*
* Description: Returns output of PID controller calculation using updated
*              error, derivative error and change in time since last run.
*****************************************************************************/
float PID::Calculate
    (
        float  error,
        float  derivative_error,
        double delta_time
    )
{
    error_sum += error * delta_time;

    // Ensure integral term is between saturation limits.
    error_sum = CapBounds(error_sum, integral_saturation_low, integral_saturation_high);

    // Calculate controller output from given inputs.
    float result = (kp * error) + (ki * error_sum) + (kd * derivative_error);

    // Ensure controller output is between saturation limits.
    result = CapBounds(result, saturation_low, saturation_high);

   return result;

} // PID::Calculate()

/*****************************************************************************
* Function: ScaledCalculate
*
* Description: Performs PID calculation by using the error and derivative error
*              inputs. Returns the output scaled from -1 to 1 based on the
*              saturation limits. e.g. Sat limits +/-5, if Calculate() output
*              is 3, ScaledCalculate() output is (3-(5+(-5)/2))/((5-(-5)/2)) = 0.6
*****************************************************************************/
float PID::ScaledCalculate
    (
        float  error,
        float  derivative_error,
        double delta_time
    )
{
    float result = Calculate(error, derivative_error, delta_time);

    // Scale output
    float midpoint = (saturation_high + saturation_low) / 2.0f;
    float range    = (saturation_high - saturation_low) / 2.0f;

    // Scales between -1 and 1, where 0 is the midpoint of the saturation range
    float scaled_output = (result - midpoint) / range;

    return scaled_output;

} // PID::ScaledCalculate()
