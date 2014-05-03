/****************************************************************************************
* File: simplefloodfill.h
*
* Description: TODO
*
* Created: 2/20/2014, by Richard Habeeb
****************************************************************************************/

#ifndef SIMPLEFLOODFILL_INCLUDED_H
#define SIMPLEFLOODFILL_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/
#include "util_math.h"
#include "ipathfinder.h"
#include "maze.h"

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/
#define INITIAL_VALUE 0
#define MAX_FLOOD_DEPTH 256

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: SimpleFloodFill
*
* Description:
******************************************************************************/
class SimpleFloodFill : public IPathFinder
{
public: //methods

	SimpleFloodFill
        (
            Maze* m
        );

	~SimpleFloodFill(void);

	// Compute the fastest route throught the maze
	void FindNextPathSegment
	(
		uint32_t		robot_current_row, // the current row of the robot
		uint32_t		robot_current_col,  // the current col of the robot
		heading_t		robot_current_heading, // the current heading of the robot
		heading_t*		next_heading, //out param of the next heading to travel
		uint32_t*		cells_to_travel // out param of the number of cells to travel in the given direction
	);
    
    // Make sure the AI knows whether we are going to the middle of the maze or back to the starting position
    void FoundDestination(void);

    // For debugging. Print the maze walls with cell data, will allocate memory on the heap
	char* ToString(void);

public: // fields


private: // methods

	// Run a classic flooding algorithm to fill in the maze
	uint32_t FloodFill(void);
    
	// create an instance of the cell data in each cell
	static void InitializeCellData
	(
		Cell*
	);

	// reset the value within the cell data
	static void ResetCellData
	(
		Cell*
	);


private: // fields
	Maze*				m;
    Cell*               target_cell;
    Cell*               origin_cell;


};

#endif // SIMPLEFLOODFILL_INCLUDED_H