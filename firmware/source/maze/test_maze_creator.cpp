/****************************************************************************************
* File: test_maze_creator.cpp
*
* Description: Creates maze instance from predefined walls in header file.
*
* Created: 11/13/2013, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <cstring>

#include "test_maze_creator.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                   MEMORY CONSTANTS
---------------------------------------------------------------------------------------*/

const uint8_t maze_values[] = {
#include "../../tools/maze_hex.h"
};

const uint32_t number_of_maze_values = sizeof(maze_values) / sizeof(maze_values[0]);

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: CreateMaze
*
* Description: Returns dynamically allocated maze with values corresponding to
*              'maze_hex.h" header file in the tools directory. The caller owns
*              the maze memory.
*****************************************************************************/
Maze * TestMazeCreator::CreateMaze
    (
        uint32_t number_rows,    // Number of rows in test maze values header file.
        uint32_t number_columns, // Number of columns in test maze values header file.
        float    cell_length     // Side length of square cell in centimeters.
    )
{
    if (number_of_maze_values != number_rows * number_columns)
    {
        return NULL; // Client passed in incorrect number of total cells.
    }

    Maze * maze = new Maze(number_rows, number_columns, cell_length);

    if (maze == NULL) { return NULL; }

    for (uint32_t r = 0; r < number_rows; ++r)
    {
        for (uint32_t c = 0; c < number_columns; ++c)
        {
            uint8_t cell_value = maze_values[r * number_columns + c];

            Cell * cell = maze->get_cell(r, c);

            bool is_wall_north = cell_value & 0x08;
            bool is_wall_east  = cell_value & 0x04;
            bool is_wall_south = cell_value & 0x02;
            bool is_wall_west  = cell_value & 0x01;

            if (is_wall_north) { cell->set_wall(north); }
            if (is_wall_east)  { cell->set_wall(east);  }
            if (is_wall_south) { cell->set_wall(south); }
            if (is_wall_west)  { cell->set_wall(west);  }
        }
    }

    return maze;

} // TestMazeCreator::CreateMaze()
