/****************************************************************************************
* File: cell.h
*
* Description: Header file for cell.cpp
*
* Created: 2/20/2014, by Richard Habeeb
****************************************************************************************/

#ifndef CELL_INCLUDED_H
#define CELL_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/
#include "frame_of_references.h"
#include "stm32f4xx.h"

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
* Class: Cell
*
* Description: This class represents a cell in the maze. It contains info
*				about adjacent cells, walls, etc...
******************************************************************************/
class Cell
{
public: // methods

	Cell(void);
	~Cell(void);

    //Determine whether there is a wall in the given heading
	bool IsWall
        (
            heading_t h
        );

	void set_wall
        (
            heading_t h
        );

	void set_adjacent_cell
        (
            heading_t h,
            Cell* c
        );

	void set_visited(bool v) { visited = v; }

	void set_data(void* d) { this->data = d; }

    Cell* get_adjacent_cell(heading_t h) { return adjacent_cells[h]; }

    void* get_data(void) const { return data; }

    bool get_visited(void) const { return visited; }

public: // fields

private: // methods

private: // fields

	Cell*			adjacent_cells[4];
	void*			data;
	bool			visited;

};

#endif //CELL_INCLUDED_H