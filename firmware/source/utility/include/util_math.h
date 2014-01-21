/******************************************************************************
* File: util_math.h
*
* Description: Header file for util_math.cpp with templates
*
* Created: 8/11/2013, by Kyle McGahee
******************************************************************************/

#ifndef UTIL_MATH_H
#define UTIL_MATH_H

/*-----------------------------------------------------------------------------
*                                  INCLUDES
*----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
*                                   TYPES
*----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
*                                 CONSTANTS
*----------------------------------------------------------------------------*/

const float PI = 3.14159265f;

/*-----------------------------------------------------------------------------
*                                 PROCEDURES
*----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
*                                 TEMPLATES
*----------------------------------------------------------------------------*/

/*******************************************************************
* Template: AbsoluteValue
*
* Description: Returns input but always positive.
*******************************************************************/
template <class T>
T AbsoluteValue
    (
        T input // Number to take absolute value of.
    )
{
    return (input < 0 ? input * -1 : input);

} // AbsoluteValue()

/*******************************************************************
* Template: max
*
* Description: Returns maximum of two input parameters.
*******************************************************************/
template <typename T>
inline T Maximum
    (
        T a,  // First value
        T b   // Second value
    )
{
    return (a > b ? a : b);

} // max()

/*******************************************************************
* Template: min
*
* Description: Returns minimum of two input parameters.
*******************************************************************/
template <typename T>
inline T Minimum
    (
        T a,  // First value
        T b   // Second value
    )
{
    return (a < b ? a : b);

} // min()

/*******************************************************************
* Template: cap_bounds
*
* Description: Returns value but limited between high and low bounds.
*******************************************************************/
template <typename T>
inline T CapBounds
    (
        T value,      // Value to cap bounds on
        T low_bounds, // Low boundary of value
        T high_bounds // High boundary of value
    )
{
    if (value > high_bounds)
    {
        value = high_bounds;
    }
    else if (value < low_bounds)
    {
        value = low_bounds;
    }

    return value;

} // cap_bounds()

#endif // UTIL_MATH_H
