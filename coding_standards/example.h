/****************************************************************************************
* File: pid.h
*
* Description: Header file for pid.cpp
*
* Created: 10/21/2013, by Kyle McGahee
****************************************************************************************/

/// Always include header guards like that seen below.  Name it
/// <filename in all caps>_INCLUDED_H

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
*                                       CLASSES ///(or could be PROCEDURES or TEMPLATES or something else)
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: PID
*
* Description: Generic Proportional/Integral/Derivative controller.
*              Calculates output value to attempt to reach setpoint value.
******************************************************************************/
class PID /// <--- Class names use camel case with first letter capitalized.
          ///      Acronyms that are 2 or less characters should all be capitalized, otherwise use camel case.
          ///      Exception if entire class name is a single acronym then make whole thing upper-case.
{
public: // methods

    PID
        (
            float kp,     // Constant for proportional term
            float ki,     // Constant for integral term
            float kd,     // Constant for derivative term
            float limit,  // Output limit cap for calculation
            float i_limit // Integral error summation cap limit
        );

    void set_setpoint /// This is a setter so use lower case 'set_' followed by name of variable.  Same for getters.
        (
            float setpoint // New setpoint value
        );
        
    /// Trivial setters can just be defined here in the class definition.
    void set_kp(float new_kp) { this->kp = new_kp; }
    
    /// Same with getters. 
    float get_kp(void) const { return kp; }
    
    /// From Google C++ style guide.
    /// "Every function declaration should have comments immediately preceding it that describe what the function does and how to use it.
    /// These comments should be descriptive ("Opens the file") rather than imperative ("Open the file"); the comment describes the function,
    /// it does not tell the function what to do. In general, these comments do not describe how the function performs its task.
    /// Instead, that should be left to comments in the function definition."
    
    /// I don't see the need to comment constructors and getters/setters.  Anything else should have a comment though.

    // Returns the current controller output based off of the newly provided value and the amount 
    // of time (in seconds) that has passed since last reading.  The units of the input value are
    // flexible and the output value depends on the input units and the gain units.  Its possible
    // for the output limit to saturate depending on the 'limit' field of the class.    
    float Calculate /// Normal function so use camel case with first letter capitalized.
        (
            float value, // New value to perform calculation on
            float dt     // Seconds since last calculation
        );

public: // fields  /// <---- if you put something in here have a good reason for making it public

private: // methods

private: // fields

    float setpoint;
    float kp;
    float ki;
    float kd;
    float error_sum;
    float last_error;
    float output_limit;
    float integral_limit;

}; // PID

#endif // PID_INCLUDED_H
