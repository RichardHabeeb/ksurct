/****************************************************************************************
* File: frame_of_references.cpp
*
* Description:
*
* Created: 02/23/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "frame_of_references.h"
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
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: HeadingDistance
*
* Description:   Returns an integer giving the minimum number of 90 degree
*                rotations to get from h1 to h2. Example N->S is 2,  E->N
*                is 1
*****************************************************************************/
int8_t HeadingDistance
    (
        cardinal_t h1, // Start heading.
        cardinal_t h2  // Heading to get distance to.
    )
{
    return Minimum(Mod(h1 - h2, (int32_t)num_cardinal_directions), Mod(h2 - h1, (int32_t)num_cardinal_directions));

} // HeadingDistance()

/*****************************************************************************
* Function: ConvertToHeading
*
* Description: Converts input 'direction' into a cardinal heading using the
*              current heading of the robot.  The converted heading is returned.
*****************************************************************************/
cardinal_t ConvertToHeading
    (
        direction_t direction,      // Direction (ie forward, right, etc) to convert.
        cardinal_t  current_heading // Current heading of the robot.
    )
{
    if (current_heading == north)
    {
        if (direction == forward)  return north;
        if (direction == right)    return east;
        if (direction == left)     return west;
        if (direction == backward) return south;
    }
    else if (current_heading == east)
    {
        if (direction == forward)  return east;
        if (direction == right)    return south;
        if (direction == left)     return north;
        if (direction == backward) return west;
    }
    else if (current_heading == south)
    {
        if (direction == forward)  return south;
        if (direction == right)    return west;
        if (direction == left)     return east;
        if (direction == backward) return north;
    }
    else if (current_heading == west)
    {
        if (direction == forward)  return west;
        if (direction == right)    return north;
        if (direction == left)     return south;
        if (direction == backward) return east;
    }

    // Should never get here.
    return west;

} // ConvertToHeading()

/*****************************************************************************
* Function: ConvertToDirection
*
* Description: Converts input 'heading' into a relative direction using the
*              current heading of the robot.  The converted direction is returned.
*****************************************************************************/
direction_t ConvertToDirection
    (
        cardinal_t heading,        // Heading (ie north, east, etc) to convert.
        cardinal_t current_heading // Current heading of the robot.
    )
{
    if (current_heading == north)
    {
        if (heading == north)  return forward;
        if (heading == east)   return right;
        if (heading == south)  return backward;
        if (heading == west)   return left;
    }
    else if (current_heading == east)
    {
        if (heading == north)  return left;
        if (heading == east)   return forward;
        if (heading == south)  return right;
        if (heading == west)   return backward;
    }
    else if (current_heading == south)
    {
        if (heading == north)  return backward;
        if (heading == east)   return left;
        if (heading == south)  return forward;
        if (heading == west)   return right;
    }
    else if (current_heading == west)
    {
        if (heading == north)  return right;
        if (heading == east)   return backward;
        if (heading == south)  return left;
        if (heading == west)   return forward;
    }

    // Should never get here.
    return right;

} // ConvertToDirection()
