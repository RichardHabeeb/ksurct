/****************************************************************************************
* File: config_maze.cpp
*
* Description: Creates correct maze to use based on configuration settings.
*
* Created: 04/20/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <stddef.h>

#include "config_settings.h"
#include "private_system_config.h"
#include "maze.h"
#include "simplefloodfill.h"
#include "test_maze_creator.h"
#include "util_assert.h"
#include "weightedpathfinding.h"

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
                                    MEMORY CONSTANTS
---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: configure_maze
*
* Description: Configures correct maze to use based on configuration settings.
*****************************************************************************/
Maze * configure_maze(void)
{
    uint32_t number_of_rows    = MAZE_NUMBER_OF_ROWS;
    uint32_t number_of_columns = MAZE_NUMBER_OF_COLUMNS;
    float    cell_length       = MAZE_CELL_LENGTH;

    Maze * maze = NULL;

    if (USE_TEST_MAZE)
    {
        maze = TestMazeCreator().CreateMaze(number_of_rows, number_of_columns, cell_length);
    }
    else
    {
        maze = new Maze(number_of_rows, number_of_columns, cell_length);
    }

    assert(maze != NULL, ASSERT_STOP);

    return maze;

} // configure_maze()

/*****************************************************************************
* Function: configure_path_finder
*
* Description: Configures correct path finder to use based on configuration settings.
*****************************************************************************/
IPathFinder * configure_path_finder
    (
        Maze * maze
    )
{
    IPathFinder * path_finder = NULL;

    if (MAZE_SOLVER == SIMPLE_FLOOD_FILL_SOLVER)
    {
        path_finder = new SimpleFloodFill(maze);
    }
    else if (MAZE_SOLVER == WEIGHTED_PATH_SOLVER)
    {
        path_finder = new WeightedPathfinding(maze);
    }

    assert(path_finder != NULL, ASSERT_STOP);

    return path_finder;

} // configure_path_finder()
