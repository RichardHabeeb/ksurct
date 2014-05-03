/****************************************************************************************
* File: ipathfinder.h
*
* Description: Interface definition for pathfindign algos
*
* Created: 2/20/2014, by Richard Habeeb
****************************************************************************************/

#ifndef IPATHFINDING_INCLUDED_H
#define IPATHFINDING_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/
#include "maze.h"

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
* Class: IPathFinder
*
* Description:	An interface for pathfinding algorithms
******************************************************************************/
class IPathFinder
{
public:

	virtual ~IPathFinder(void) {};

	// Compute the fastest route throught the maze.
	virtual void FindNextPathSegment
	(
		uint32_t		robot_current_row, // the current row of the robot
		uint32_t		robot_current_col,  // the current col of the robot
		heading_t		robot_current_heading, // the current heading of the robot
		heading_t*		next_heading, //out param of the next heading to travel
		uint32_t*		cells_to_travel // out param of the number of cells to travel in the given direction
	) = 0;
    
    // Make sure the AI knows whether we are going to the middle of the maze or back to the starting position
    virtual void FoundDestination(void) = 0;

	// For debugging. Print the maze walls with cell data, will allocate memory on the heap
	virtual char* ToString(void) = 0;
};

#endif // IPATHFINDING_INCLUDED_H