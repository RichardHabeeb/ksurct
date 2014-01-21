/**********************************************************************
* File: pid.h
*
* Description: Header file for pid.cpp
*
* Created: 11/11/2013, by Brian Blankenau and Alex Marsh
**********************************************************************/

#ifndef PID_INCLUDED_H
#define PID_INCLUDED_H

/*---------------------------------------------------------------------
*                             INCLUDES
*--------------------------------------------------------------------*/

/*---------------------------------------------------------------------
*                             CONSTANTS
*--------------------------------------------------------------------*/

/*---------------------------------------------------------------------
*                               TYPES
*--------------------------------------------------------------------*/

/*----------------------------------------------------------------------
*                              CLASSES
*--------------------------------------------------------------------*/

/**********************************************************************
* Class: PID
*
* Description: Generic Proportional/ Integral/ Derivative controller.
**********************************************************************/
class PID
{
public: // Methods

    PID
    (
        float kp,              // Proportional Gain
        float ki,              // Integral Gain
        float kd,              // Derivative Gain
        float sat_high,        // Upper Output Saturation Limit
        float sat_low,         // Lower Output Saturation Limit
        float int_sat_high,    // Upper Error Integral Saturation Limit
        float int_sat_low      // Lower Error Integral Saturation Limit
    ); // PID Constructor

    // Returns the controller output for a given error and error derivative input. If
    // derivative error control is to be used, it is the responsibility of the user to
    // calculate the derivative value before calling this function.
    float Calculate
    (
        float error,
        float derivative
    );

    // Same description as the Calculate function, but ScaleCalculate scales its output
    // between -1 and 1 based on the saturation limits.
    float ScaleCalculate
    (
        float error,
        float derivative
    );

    // Setter functions for private variables
    void set_kp(float kp) {this->kp = kp;}
    void set_ki(float ki) {this->ki = ki;}
    void set_kd(float kd) {this->kd = kd;}

    // Getter functions for private variables
    float get_kp() const {return kp;}
    float get_ki() const {return ki;}
    float get_kd() const {return kd;}

private: // Variables

    float kp;           // Proportional Gain
    float ki;           // Integral Gain
    float kd;           // Derivative Gain
    float sat_high;     // Saturation for high output limits
    float sat_low;      // Saturation for low output limits
    float int_sat_high; // Anti wind-up saturation
    float int_sat_low;  // Anti wind-up saturation
    float error_sum;    // Integral error

}; //PID

#endif // PID_INCLUDED_H
