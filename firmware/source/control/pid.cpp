/**********************************************************************
* File: pid.cpp
*
* Description: Function definitions for the PID class.
*
* Created: 11/11/2013, by Brian Blankenau and Alex Marsh
**********************************************************************/

/*---------------------------------------------------------------------
*                             INCLUDES
*--------------------------------------------------------------------*/

#include "pid.h"
#include "util_math.h"

/*---------------------------------------------------------------------
*                         LITERAL CONSTANTS
*--------------------------------------------------------------------*/

/*---------------------------------------------------------------------
*                              TYPES
*--------------------------------------------------------------------*/

/*---------------------------------------------------------------------
*                         MEMORY CONSTANTS
*--------------------------------------------------------------------*/

/*---------------------------------------------------------------------
*                            VARIABLES
*--------------------------------------------------------------------*/

/*---------------------------------------------------------------------
*                          CLASS METHODS
*--------------------------------------------------------------------*/

/**********************************************************************
* Function: PID - Constructor
*
* Description: Initializes fields for new instance of PID object.
**********************************************************************/
PID::PID
    (
        float kp,              // Proportional Gain
        float ki,              // Integral Gain
        float kd,              // Derivative Gain
        float sat_high,        // Upper Output Saturation Limit
        float sat_low,         // Lower Output Saturation Limit
        float int_sat_high,    // Upper Error Integral Saturation Limit
        float int_sat_low      // Lower Error Integral Saturation Limit
    ):
    kp(kp),
    ki(ki),
    kd(kd),
    sat_high(sat_high),
    sat_low(sat_low),
    int_sat_high(int_sat_high),
    int_sat_low(int_sat_low),
    error_sum(0)
{
}// PID() - Constructor

/*****************************************************************************
* Function: Calculate
*
* Description: Performs PID calculation by using the Calculate() function.
*              Scales output by using the saturation limits, and returns the
*              result.
* Note: this function does not update the scaled_output variable.
*****************************************************************************/
float PID::Calculate(float error, float derivative_error)
{
    // Add on error to integral error summation.
    error_sum += error;

    // Ensure integral term is between saturation limits.
    error_sum = CapBounds(error_sum, int_sat_low, int_sat_high);

    // Calculate controller output from given inputs.
    float result = (kp * error) + (ki * error_sum) + (kd * derivative_error);

    // Ensure controller output is between saturation limits
    result = CapBounds(result, sat_low, sat_high);

   return result;

} // Calculate()

/*****************************************************************************
* Function: ScaleCalculate
*
* Description: Performs PID calculation by using the error and derivative error
*              inputs. Returns the output scaled from -1 to 1 based on the
*              saturation limits. e.g. Sat limits +/-5, if Calculate() output
*              is 3, ScaleCalculate() output is (3-(5+(-5)/2))/((5-(-5)/2)) = 0.6
*****************************************************************************/
float PID::ScaleCalculate(float error, float derivative)
{
    float result = Calculate(error, derivative);

    // Scale output
    float midpoint = (sat_high + sat_low) / 2.0f;
    float range = (sat_high - sat_low) / 2.0f;

    // Scales between -1 and 1, where 0 is the midpoint of the saturation range
    float scaled_output = (result - midpoint) / range;

    return scaled_output;

} // Calculate()
