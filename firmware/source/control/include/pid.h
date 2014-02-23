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

    // Returns the controller output for a given error.  Derivative error is calculated
    // as a simple dx/dt.  If a derivative error is already calculated then use overloaded
    // method.
    float Calculate
        (
            float error,      // Difference between desired and actual measurement.
            float delta_time  // Change in time since last measurement. (seconds)
        );

    // Overloads calculate method to allow the derivative error to be passed in.
    float Calculate
        (
            float error,            // Difference between desired and actual measurement.
            float derivative_error, // Change in error over specified change in time.
            float delta_time        // Change in time since last measurement. (seconds)
        );

    // Setters
    void reset_integral_error(void) { this->error_sum = 0; }
    void set_kp(float kp) { this->kp = kp; }
    void set_ki(float ki) { this->ki = ki; }
    void set_kd(float kd) { this->kd = kd; }

private: // Variables

    float kp;                       // Proportional Gain
    float ki;                       // Integral Gain
    float kd;                       // Derivative Gain
    float saturation_high;          // Saturation for high output limits
    float saturation_low;           // Saturation for low output limits
    float integral_saturation_high; // Anti wind-up saturation
    float integral_saturation_low;  // Anti wind-up saturation
    float error_sum;                // Integral error
    float previous_error;           // Error that was last used for calculation.

}; //PID

#endif // PID_INCLUDED_H
