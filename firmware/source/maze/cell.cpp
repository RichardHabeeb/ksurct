/****************************************************************************************
* File: cell.cpp
*
* Description: implementation of cell class methods
*
* Created: 2/20/2014, by Richard Habeeb
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/
#include "cell.h"

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
* Function: Cell - Constructor
*
* Description: Initializes fields for new instance of Cell object. 
*****************************************************************************/
Cell::Cell(void)
{
	visited = false;

	for(uint32_t i = 0; i < NUM_HEADINGS; i++)
	{
		adjacent_cells[i] = _NULL;
	}
} // Cell()

/*****************************************************************************
* Function: Cell - Destructor
*
* Description:	This destructor actually will also call the destructor on 
*				adjacent cells, so deleting a cell will delete the maze!!
*****************************************************************************/
Cell::~Cell(void)
{
	for (heading_t h = north; h < NUM_HEADINGS; h++)
	{
		if (adjacent_cells[h] != _NULL)
			delete[] adjacent_cells;
	}		
} // ~Cell()

/*****************************************************************************
* Function: set_wall
*
* Description:	Will set adjacent cell pointer to null and the adjacent
*				cell's pointer to null.
*****************************************************************************/
void Cell::set_wall( heading_t h )
{
	if (!IsWall(h))
	{
		adjacent_cells[ h ]->adjacent_cells[ GetReverseHeading( h ) ] = _NULL;
		adjacent_cells[ h ] = _NULL;
	}	
} // set_wall()

/*****************************************************************************
* Function: get_adjacent_cell
*
* Description:	getter
*****************************************************************************/
Cell* Cell::get_adjacent_cell( heading_t h )
{
	return adjacent_cells[ h ];
} // get_adjacent_cell()

/*****************************************************************************
* Function: set_adjacent_cell
*
* Description:	Will assign pointer in adjacent cell in addition to this 
*				cell.
*****************************************************************************/
void Cell::set_adjacent_cell( heading_t h, Cell* c )
{
	if(c != _NULL)
		c->adjacent_cells[GetReverseHeading(h)] = this;
	adjacent_cells[ h ] = c;
} // set_adjacent_cell()

/*****************************************************************************
* Function: IsWall
*
* Description:	If the adjacent cell pointer in a heading is null, then a
*				wall exists
*****************************************************************************/
bool Cell::IsWall(heading_t h)
{
	return (adjacent_cells[h] == _NULL);
} // IsWall()