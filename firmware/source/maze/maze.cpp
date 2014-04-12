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
Maze::Maze
    (
        uint32_t number_rows,    // Number of rows of maze. Must be at least one.
        uint32_t number_columns, // Number of columns of maze. Must be at least one.
        float    cell_length     // Side length of square cell in centimeters.
    )
{
    this->number_rows    = number_rows;
    this->number_columns = number_columns;
    this->cell_length    = cell_length;

	cell_index = new Cell**[number_rows];

	for (uint32_t r = 0; r < number_rows; r++)
	{
		cell_index[r] = new Cell*[number_columns];
		for (uint32_t c = 0; c < number_columns; c++)
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

    this->starting_cell = get_cell(0, 0);
    this->goal_cell     = get_cell(number_rows/2, number_columns/2);

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
* Function: get_cell
*
* Description:
*****************************************************************************/
Cell* Maze::get_cell
	(
		position_t cell_position
	)
{
	return get_cell(cell_position.y, cell_position.x);

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
	return (r < number_rows && c < number_columns );
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
* Function: Map
*
* Description:
*****************************************************************************/
void Maze::Map
	(
		void (*func)(Cell*)
	)
{
	for (uint32_t r = 0; r < number_rows; r++)
	{
		for (uint32_t c = 0; c < number_columns; c++)
		{
			(*func)(cell_index[r][c]);
		}
	}
} // Map()

/*****************************************************************************
* Function: Transpose
*
* Description: UNTESTED
*****************************************************************************/
void Maze::Transpose(void)
{

	for (uint32_t r = 0; r < number_rows; r++)
	{
		for (uint32_t c = 0; c < number_columns; c++)
		{
            if(c > number_rows-r && c != r)
            {
                Cell* temp = cell_index[r][c];
                cell_index[r][c] = cell_index[c][r];
                cell_index[c][r] = temp;
            }
            
            cell_index[r][c]->TransposeWalls();

		}
	}
    
} // Transpose()

/*****************************************************************************
* Function: MapCenterSquareWalls
*
* Description: UNTESTED
*****************************************************************************/
void Maze::MapCenterSquareWalls
    (
        uint32_t entrance_cell_r,
        uint32_t entrance_cell_c,
        heading_t entrance_heading
    )
{
    uint32_t south_west_corner_r = number_rows/2;
    uint32_t south_west_corner_c = number_columns/2;
    
    cell_index[south_west_corner_r][south_west_corner_c]->set_wall(east);
    cell_index[south_west_corner_r][south_west_corner_c]->set_wall(south);
    
    cell_index[south_west_corner_r][south_west_corner_c - 1]->set_wall(west);
    cell_index[south_west_corner_r][south_west_corner_c - 1]->set_wall(south);

    cell_index[south_west_corner_r - 1][south_west_corner_c]->set_wall(east);
    cell_index[south_west_corner_r - 1][south_west_corner_c]->set_wall(north);
    
    cell_index[south_west_corner_r - 1][south_west_corner_c - 1]->set_wall(west);
    cell_index[south_west_corner_r - 1][south_west_corner_c - 1]->set_wall(north);
    
    uint32_t prior_entrance_cell_r = entrance_cell_r + (entrance_heading == north) - (entrance_heading == south);
    uint32_t prior_entrance_cell_c = entrance_cell_c + (entrance_heading == east) - (entrance_heading == west);
    
    cell_index[entrance_cell_r][entrance_cell_c]->set_adjacent_cell
        (
        GetReverseHeading(entrance_heading),
        cell_index[prior_entrance_cell_r][prior_entrance_cell_c]
        );
    
    
    
} // MapCenterSquareWalls()
