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
* Class: Maze
*
* Description:   This class represents a maze. It is built up of a 4-way linked
*                list of cells (forming a "grid" in the heap). This structure
*                is held by a 2d array (for fast indexing of cells).
******************************************************************************/
class Maze
{
public: // methods

    // Constructor
    Maze
        (
            uint32_t number_rows,    // Number of rows of maze. Must be at least one.
            uint32_t number_columns, // Number of columns of maze. Must be at least one.
            float    cell_length     // Side length of square cell in centimeters.
        );

    // Destructor
    ~Maze(void);

    // Determine if a cell coords are within the bounds of the maze
    bool IsValidCell
        (
            uint32_t        r,
            uint32_t        c
        );

    // Returns true if a cell is the goal cell (via reference comparison)
    bool IsGoalCell
        (
            Cell*
        );

    // Returns true if a cell is the goal cell (via reference comparison)
    bool IsGoalCell
        (
            uint32_t r,
            uint32_t c
        );
    
    void MapCenterSquareWalls
        (
            uint32_t entrance_cell_r,
            uint32_t entrance_cell_c,
            heading_t entrance_heading
        );

    // Pass in a static function pointer to exceute the function on each cell of the maze.
    void Map
        (
            void (*func)(Cell*)
        );

    // Once the maze has been solved we need to go back to the start cell.
    void SwapStartingAndGoal(void);

    // Swap the row and column of every cell in the maze.
    void Transpose(void);

    void set_starting_cell (uint32_t r, uint32_t c) { starting_cell = cell_index[r][c]; }

    void set_goal_cell (uint32_t r, uint32_t c ) { goal_cell = cell_index[r][c]; }

    Cell* get_cell
        (
            uint32_t r,
            uint32_t c
        );

    Cell* get_cell
        (
            position_t cell_position
        );

    Cell* get_starting_cell(void) const { return starting_cell; }

    Cell* get_goal_cell(void) const { return goal_cell; }

    // Returns side length of cell (assuming square cell) in centimeters.
    float get_cell_length(void) const { return this->cell_length; }

    // Returns maze dimensions.
    uint32_t get_number_rows(void) const { return this->number_rows; }
    uint32_t get_number_columns(void) const { return this->number_rows; }

public: //fields

private: //methods

private:

    Cell*    starting_cell;
    Cell*    goal_cell;
    Cell***  cell_index;

    // Number of rows and columns of maze. Each dimension is at least one.
    uint32_t number_rows;
    uint32_t number_columns;

    float cell_length; // In centimeters.

};

#endif //MAZE_INCLUDED_H
