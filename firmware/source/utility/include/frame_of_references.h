/******************************************************************************
* File: frame_of_references.h
*
* Description: Defines constants that describe relationships between different
*              frames of reference.  Also provides some conversion functions
*              to switch the frame of reference.
*
* Created: 2/27/2013, by Kyle McGahee
******************************************************************************/

#ifndef FRAME_OF_REFERENCES_INCLUDED_H
#define FRAME_OF_REFERENCES_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <stdint.h>

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

// Cardinal directions. Used for maze frame commands.  This frame is relative to the
// robot starting in the top left hand corner of the maze.
typedef uint8_t cardinal_t;
enum
{
    north,
    east,
    south,
    west,
    num_cardinal_directions
};

typedef cardinal_t heading_t;

// Relative to robot from.  The Right/Left designations are using the 'driver' notation
// where they would be relative from someone sitting in the driver seat of the robot.
typedef uint8_t direction_t;
enum
{
    forward,
    right,
    left,
    backward,
    num_directions,
};

// Defines the position of a cell within the maze.  Position (0,0) would be the starting
// point in the top left hand corner of the maze.
typedef struct
{
    uint8_t x; // Cell number in the 'Horizontal' or 'East' direction.
    uint8_t y; // Cell number in the 'Vertical' or 'South' direction.
} position_t;

/*---------------------------------------------------------------------------------------
*                                        MACROS
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: GetReverseHeading
*
* Description:  Returns the heading 180 degress opposite of the given heading
*****************************************************************************/
#define GetReverseHeading(h)  (((h) + 2) % num_cardinal_directions)

/*---------------------------------------------------------------------------------------
*                                      PROCEDURES
*--------------------------------------------------------------------------------------*/

// Returns an integer giving the minimum number of 90 degree rotations to get from h1 to h2.
// Example N->S is 2,  E->N is 1
int8_t HeadingDistance
    (
        cardinal_t h1, // Start heading.
        cardinal_t h2  // Heading to get distance to.
    );

// Converts input 'direction' into a cardinal heading which is returned.
cardinal_t ConvertToHeading
    (
        direction_t direction,      // Direction (ie forward, right, etc) to convert.
        cardinal_t  current_heading // Current heading of the robot.
    );

// Converts input 'heading' into a relative direction which is returned.
direction_t ConvertToDirection
    (
        cardinal_t heading,        // Heading (ie north, east, etc) to convert.
        cardinal_t current_heading // Current heading of the robot.
    );

#endif // FRAME_OF_REFERENCES_INCLUDED_H
