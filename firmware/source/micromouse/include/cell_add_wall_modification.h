/****************************************************************************************
* File: cell_add_wall_modification.h
*
* Description: Header file for cell.cpp
*
* Created: 2/20/2014, by Richard Habeeb
****************************************************************************************/

#ifndef CELL_ADD_WALL_MODIFICATION_H
#define CELL_WALL_ADD_MODIFICATION_H

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
* Class: CellAddWallModification
*
* Description:  This class represents an maze undo stack, represented as a 
                circular buffer. We can undo and redo changes to the
******************************************************************************/
class CellAddWallModification : public ICellModification
{
public: // methods

    CellAddWallModification
        (
            position_t cell_position;
            position_t adjacent_cell_position;
            heading_t heading;
        );
    
	~CellAddWallModification(void);
    
    void Undo();
    
private: // methods
    
public: // fields
    
private: // fields
    position_t cell_position;
    position_t adjacent_cell_position;
    heading_t heading;
}

#endif // CELL_WALL_ADD_MODIFICATION_H
