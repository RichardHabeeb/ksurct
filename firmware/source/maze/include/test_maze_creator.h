/****************************************************************************************
* File: test_maze_creator.h
*
* Description: Header file for test_maze_creator.cpp
*
* Created: 3/29/2014, by Kyle McGahee
****************************************************************************************/

#ifndef TEST_MAZE_CREATOR_INCLUDED_H
#define TEST_MAZE_CREATOR_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "maze.h"

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
* Class: TestMazeCreator
*
* Description: Creates maze instance from predefined walls in header file.
******************************************************************************/
class TestMazeCreator
{
public: // methods

    // Returns dynamically allocated maze with values corresponding to 'maze_hex.h" header
    // file in the tools directory. The caller owns the maze memory.
    Maze * CreateMaze
        (
            uint32_t number_rows,    // Number of rows in test maze values header file.
            uint32_t number_columns, // Number of columns in test maze values header file.
            float    cell_length     // Side length of square cell in centimeters.
        );

private: // methods

private: // fields

}; // TestMazeCreator

#endif // TEST_MAZE_CREATOR_INCLUDED_H
