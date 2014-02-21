/****************************************************************************************
* File: maze.h
*
* Description: Header file for maze.cpp
*
* Created: 2/20/2014, by Richard Habeeb
****************************************************************************************/

#ifndef MAZE_INCLUDED_H
#define MAZE_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/
#include "cell.h"
#include "stm32f4xx.h"

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/
#define MAZE_NUM_ROWS	16
#define MAZE_NUM_COLS	16

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: Maze
*
* Description:	This class represents a maze. It is built up of a 4-way linked
*				list of cells (forming a "grid" in the heap). This structure
*				is held by a 2d array (for fast indexing of cells).
******************************************************************************/
class Maze
{
public: // methods
	Maze(void);
	~Maze(void);

	Cell* get_starting_cell(void) const
	{
		return starting_cell;
	}

	void set_starting_cell
	(
		uint32_t r,
		uint32_t c
	)
	{
		starting_cell = cell_index[r][c];
	}


	Cell* get_goal_cell(void) const
	{
		return goal_cell;
	}

	void set_goal_cell
	(
		uint32_t r,
		uint32_t c
	)
	{
		goal_cell = cell_index[r][c];
	}


	Cell* get_cell
	(
		uint32_t r,
		uint32_t c
	);

	// Determine if a cell coords are within the bounds of the maze
	bool IsValidCell
	(
		uint32_t		r,
		uint32_t		c
	);

	// Determine if a cell is the goal cell (via referenec comparison)
	bool IsGoalCell
	(
		Cell*
	);

	// Determine if a cell is the goal cell (via referenec comparison)
	bool IsGoalCell
	(
		uint32_t r,
		uint32_t c
	);

	// Pass in a static function pointer to exceute the function on each cell of the maze
	void Map
	(
		void (*func)(Cell*)
	);

	// Once the maze has been solved we need to go back to the start cell.
	void SwapStartingAndGoal(void);

	// swap the row and column of every cell in the maze.
	void Transpose(void);

public: //fields

private: //methods

private:
	Cell*	starting_cell;
	Cell*	goal_cell;
	Cell*** cell_index;

};


#endif //MAZE_INCLUDED_H
