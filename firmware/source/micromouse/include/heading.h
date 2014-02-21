/****************************************************************************************
* File: heading.h
*
* Description: Header file for the heading type
*
* Created: 2/20/2014, by Richard Habeeb
****************************************************************************************/

#ifndef HEADING_INCLUDED_H
#define HEADING_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/
#include "util_math.h"

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/
typedef unsigned char heading_t;
enum
{
	north,
	east,
	south,
	west,
	NUM_HEADINGS,
};

/*---------------------------------------------------------------------------------------
*                                       PROCEDURES
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: GetReverseHeading
*
* Description:	Returns the heading 180 degress opposite of the given heading
*****************************************************************************/
#define GetReverseHeading(h)	(h + 2) % NUM_HEADINGS


/*****************************************************************************
* Function: HeadingDistance
*
* Description:	Returns an integer giving the minimum number of 90 degree
*				rotations to get from h1 to h2. Example N->S is 2,  E->N
*				is 1
*****************************************************************************/
inline int HeadingDistance(heading_t h1, heading_t h2)
{
	return Minimum(Mod(h1 - h2, (int)NUM_HEADINGS), Mod(h2 - h1, (int)NUM_HEADINGS));
}


#endif // HEADING_INCLUDED_H