/****************************************************************************************
* File: simplefloodfill.h
*
* Description: Interface definition for pathfindign algos
*
* Created: 2/20/2014, by Richard Habeeb
****************************************************************************************/

#ifndef WEIGHTEDFLOODFILL_INCLUDED_H
#define WEIGHTEDFLOODFILL_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/
#include "ipathfinding.h"
#include "util_math.h"
#include "queue.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

// Algorithm Constants
// (used for speedup estimates)
#define			STEP_WEIGHT			2
#define			VISITED_STEP_WEIGHT	1
#define			TURN_WEIGHT			4
#define			INITIAL_WEIGHT		-1

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/
typedef struct
{
	int32_t				weight;
	heading_t			robot_heading_sim;
	Cell*				next_cell;
} cell_data_t;

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: SimpleFloodFill
*
* Description:
******************************************************************************/
class WeightedPathfinding :
	public IPathFinding
{
public: //methods
	WeightedPathfinding
		(
		Maze* m
		);

	~WeightedPathfinding();

	// Compute the fastest route throught the maze
	void FindNextPathSegment
		(
		uint32_t		robot_current_row, // the current row of the robot
		uint32_t		robot_current_col,  // the current col of the robot
		heading_t		robot_current_heading, // the current heading of the robot
		heading_t*		next_heading, //out param of the next heading to travel
		uint32_t*		cells_to_travel // out param of the number of cells to travel in the given direction
		);

	//For debugging. Print the maze walls with cell data, will allocate memory on the heap
	char* ToString
		(void);


private:
	//allocate and assign cell data structs into all the cells of the maze
	static void InitializeCellData
		(
		Cell *
		);

	//reset all cell data to the default weight
	static void ResetCellData
		(
		Cell *
		);

public: // fields


private: // methods
private: // fields
	Queue<Cell>	cell_q;
	uint32_t	max_flood_depth;
	Maze*		m;
	uint32_t	maze_max_dim;

};

#endif //WEIGHTEDFLOODFILL_INCLUDED_H