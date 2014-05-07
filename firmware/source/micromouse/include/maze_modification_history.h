/****************************************************************************************
* File: maze_modifiction_history.h
*
* Description: Header file for cell.cpp
*
* Created: 2/20/2014, by Richard Habeeb
****************************************************************************************/

#ifndef MAZE_MODIFICATION_HISTORY_H
#define MAZE_MODIFICATION_HISTORY_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/
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
* Class: MazeHodificationHistory
*
* Description:  This class represents an maze undo stack, represented as a 
                circular buffer. We can undo and redo changes to the
******************************************************************************/
class MazeHodificationHistory
{
public: // methods

	MazeHodificationHistory
        (
            uint32_t buffer_size,
            Maze maze
        );
    
	~MazeHodificationHistory(void);
    
private: // methods
    
public: // fields
    
private: // fields
    
    uint32_t buffer_size;
    Maze maze;
}