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

const float degrees_per_radian = 180.f / PI;

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
* Template: Maximum
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

} // Maximum()

/*******************************************************************
* Template: Minimum
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

} // Minimum()

/*******************************************************************
* Template: CapBounds
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

} // CapBounds()

/*******************************************************************
* Template: Mod
*
* Description: Returns a mathematical modulus, taking into account
* negatives.
*******************************************************************/
template <typename T>
inline T Mod
	(
		T x,
		T m
	)
{
	m       = (m < 0) ? -m : m;
	int r	= x % m;
	return (r < 0) ? r + m : r;

} // Mod()

/*******************************************************************
* Template: Swap
*
* Description: Swaps values stored in first and second parameter.
*******************************************************************/
template <typename T>
inline void Swap
	(
		T & first,  // Value to swap with second.
		T & second  // Value to swap with first.
	)
{
    T temp = first;
    first = second;
    second = temp;

} // Swap()

#endif // UTIL_MATH_H
