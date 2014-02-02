/****************************************************************************************
* File: pid.h
*
* Description: Header file for pid.cpp
*
* Created: 02/01/2013, by Kyle McGahee
****************************************************************************************/

#ifndef PID_INCLUDED_H
#define PID_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

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
* Class: PID
*
* Description: Generic Proportional/ Integral/ Derivative controller.
******************************************************************************/
class PID
{
public: // Methods

    // Constructor
    PID
    (
        float kp,                        // Proportional Gain
        float ki,                        // Integral Gain
        float kd,                        // Derivative Gain
        float saturation_high,           // Upper Output Saturation Limit
        float saturation_low,            // Lower Output Saturation Limit
        float integral_saturation_high,  // Upper Error Integral Saturation Limit
        float integral_saturation_low    // Lower Error Integral Saturation Limit
    );

    // Returns the controller output for a given error and error derivative input. If
    // derivative error control is to be used, it is the responsibility of the user to
    // calculate the derivative value before calling this function.
    float Calculate
    (
        float  error,
        float  derivative_error,
        double delta_time
    );

    // Same description as the Calculate function, but ScaledCalculate scales its output
    // between -1 and 1 based on the saturation limits.
    float ScaledCalculate
    (
        float  error,
        float  derivative_error,
        double delta_time
    );

    // Setters
    void set_kp(float kp) {this->kp = kp;}
    void set_ki(float ki) {this->ki = ki;}
    void set_kd(float kd) {this->kd = kd;}

private: // Variables

    float kp;                       // Proportional Gain
    float ki;                       // Integral Gain
    float kd;                       // Derivative Gain
    float saturation_high;          // Saturation for high output limits
    float saturation_low;           // Saturation for low output limits
    float integral_saturation_high; // Anti wind-up saturation
    float integral_saturation_low;  // Anti wind-up saturation
    float error_sum;                // Integral error

}; //PID

#endif // PID_INCLUDED_H
