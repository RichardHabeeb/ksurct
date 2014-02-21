/****************************************************************************************
* File: maze.cpp
*
* Description: Implementation of the maze class methods
*
* Created: 2/20/2014, by Richard Habeeb
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/
#include "maze.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                   MEMORY CONSTANTS
---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      VARIABLES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                    CLASS METHODS
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: Maze - Constructor
*
* Description: Initializes fields for new instance of Maze object.
*****************************************************************************/
Maze::Maze(void)
{
	cell_index = new Cell**[MAZE_NUM_ROWS];

	for (uint32_t r = 0; r < MAZE_NUM_ROWS; r++)
	{
		cell_index[r] = new Cell*[MAZE_NUM_COLS];
		for (uint32_t c = 0; c < MAZE_NUM_COLS; c++)
		{
			cell_index[r][c] = new Cell();
			cell_index[r][c]->set_visited(false);

			if (c > 0)
			{
				cell_index[r][c]->set_adjacent_cell(west, cell_index[r][c - 1]);
			}

			if (r > 0) 
			{
				cell_index[r][c]->set_adjacent_cell(north, cell_index[r - 1][c]);
			}
		}
	}
} // Maze()

/*****************************************************************************
* Function: Maze - Destructor
*
* Description: calls the destructor for the first cell
*****************************************************************************/
Maze::~Maze(void)
{
	delete cell_index;
} // ~Maze()

/*****************************************************************************
* Function: get_cell
*
* Description: 
*****************************************************************************/
Cell* Maze::get_cell
	(
		uint32_t r,
		uint32_t c
	)
{
	if (IsValidCell(r,c))
		return cell_index[r][c];
	else
		return _NULL;
} // get_cell()

/*****************************************************************************
* Function: IsGoalCell
*
* Description:
*****************************************************************************/
bool Maze::IsGoalCell
	(
		uint32_t r,
		uint32_t c
	)
{
	return cell_index[r][c] == goal_cell;
} // IsGoalCell()

/*****************************************************************************
* Function: IsGoalCell
*
* Description:
*****************************************************************************/
bool Maze::IsGoalCell
	(
		Cell* c
	)
{
	return c == goal_cell;
} // IsGoalCell()

/*****************************************************************************
* Function: IsValidCell
*
* Description: is this cell within the bounds of the maze
*****************************************************************************/
bool Maze::IsValidCell
	(
		uint32_t		r,
		uint32_t		c
	)
{
	return (r < MAZE_NUM_ROWS && c < MAZE_NUM_COLS );
}// IsValidCell()

/*****************************************************************************
* Function: SwapStartingAndGoal
*
* Description:
*****************************************************************************/
void Maze::SwapStartingAndGoal(void)
{
	Cell* t		= starting_cell;
	starting_cell	= goal_cell;
	goal_cell	= t;
} // SwapStartingAndGoal()

/*****************************************************************************
* Function: SwapStartingAndGoal
*
* Description:
*****************************************************************************/
void Maze::Map
	(
		void (*func)(Cell*)
	)
{
	for (uint32_t r = 0; r < MAZE_NUM_ROWS; r++)
	{
		for (uint32_t c = 0; c < MAZE_NUM_COLS; c++)
		{
			(*func)(cell_index[r][c]);
		}
	}
} // Map()