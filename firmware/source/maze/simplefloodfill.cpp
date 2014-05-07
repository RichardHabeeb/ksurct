/****************************************************************************************
* File: Maze.cpp
*
* Description: Implementation of the maze class methods
*
* Created: 2/20/2014, by Richard Habeeb
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/
#include "simplefloodfill.h"

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
* Function: SimpleFloodFill - Constructor
*
* Description: Allocates and initializes the cell data
*****************************************************************************/
SimpleFloodFill::SimpleFloodFill
	(
		Maze* m
	)
{
	m->Map(InitializeCellData);
	this->m = m;
    
    target_cell = m->get_goal_cell();
    origin_cell = m->get_starting_cell();
} // SimpleFloodFill()


/*****************************************************************************
* Function: WeightedPathfinding - Destructor
*
* Description:
*****************************************************************************/
SimpleFloodFill::~SimpleFloodFill
	(
		void
	)
{

}


/*****************************************************************************
* Function: FindNextPathSegment
*
* Description:	Compute the fastest route throught the maze. Simulate maze
*				solutions and ouput the next heading and the number of cells
*				to travel forward to get to that turn.
*****************************************************************************/
void SimpleFloodFill::FindNextPathSegment
	(
		uint32_t		robot_current_row,      // the current row of the robot
		uint32_t		robot_current_col,      // the current col of the robot
		heading_t		robot_current_heading,  // the current heading of the robot
		heading_t*		next_heading,           // out param of the next heading to travel
		uint32_t*		cells_to_travel         // out param of the number of cells to travel in the given direction
	)
{
    Cell* current_cell          = m->get_cell(robot_current_row, robot_current_col);
    uint32_t depth		        = FloodFill();
    uint32_t current_cell_depth = *((int32_t*)current_cell->get_data());
    *cells_to_travel            = 0;

    if (depth == MAX_FLOOD_DEPTH) return;
    
    
    
    Cell* adjacent_cell = current_cell->get_adjacent_cell(robot_current_heading);
    
    //Decide how many cells to travel forward (0->N cells) 
    while(adjacent_cell != _NULL &&                                             //Make sure no wall is in front of us
          *((int32_t*)adjacent_cell->get_data()) == current_cell_depth - 1 &&   //Check to see if the cell in front of us is the next best decision
          ( adjacent_cell->get_visited() || *cells_to_travel == 0 ) )           //If we are traveling more than one cell verify that we have visited it before
    {
        *cells_to_travel += 1;
        current_cell_depth--;
        current_cell = adjacent_cell;
        adjacent_cell = adjacent_cell->get_adjacent_cell(robot_current_heading);
    }
    
    //decide which direction to turn after moving forward
    for (heading_t h = north; h < num_cardinal_directions; h++)
    {
        if(h == robot_current_heading  || (h == GetReverseHeading(robot_current_heading) && cells_to_travel > 0)) continue;
        
        Cell* adjacent_cell = current_cell->get_adjacent_cell(h);

        if (adjacent_cell != _NULL && *((int32_t*)adjacent_cell->get_data()) == current_cell_depth - 1)
        {
            *next_heading = h;
            return;
        }   
        

    }

} // FindNextPathSegment()


/*****************************************************************************
* Function: FloodFill
*
* Description:	Run a simplistic flooding algorithm.
*****************************************************************************/
uint32_t SimpleFloodFill::FloodFill(void)
{
        uint32_t depth			= 1;

        m->Map(SimpleFloodFill::ResetCellData);

        *((int32_t*)target_cell->get_data()) = 1;

        while (++depth < MAX_FLOOD_DEPTH)
        {
                for (uint32_t r = 0; r < m->get_number_rows() ; r++)
                {
                        for (uint32_t c = 0; c < m->get_number_columns(); c++)
                        {
                                Cell* search_cell = m->get_cell(r, c);
                                if (*((int32_t*)search_cell->get_data()) == 0)
                                {
                                        for (heading_t h = north; h < num_cardinal_directions; h++)
                                        {
                                                Cell* adjacent_cell = search_cell->get_adjacent_cell(h);

                                                if (adjacent_cell != _NULL && *((int32_t*)adjacent_cell->get_data()) == depth - 1)
                                                {
                                                        *((int32_t*)search_cell->get_data()) = depth;
                                                        if (search_cell == origin_cell)
                                                                return depth;
                                                }
                                        }
                                }
                        }
                }
        }

	return MAX_FLOOD_DEPTH;
} // SimpleFloodFill::FloodFill()

/*****************************************************************************
* Function: FoundDestination
*
* Description:	Keep track of whether the robot is going to the middle or
*               going to the corner.
*****************************************************************************/
void SimpleFloodFill::FoundDestination(void)
{
    Swap(target_cell, origin_cell);
} // SimpleFloodFill::FoundDestination()

/*****************************************************************************
* Function: ToString
*
* Description:	For debugging. Print the maze walls with cell data, will
*				allocate memory on the heap
*****************************************************************************/
char* SimpleFloodFill::ToString(void)
{
	char* s = new char[m->get_number_rows()*(m->get_number_columns()*(num_cardinal_directions + 4) + 1)];
	int32_t write_index = 0;
	char heading_names[] = "NESW";

	for (uint32_t r = 0; r < m->get_number_rows(); r++)
	{
		for (uint32_t c = 0; c < m->get_number_columns(); c++)
		{
			for (heading_t h = north; h < num_cardinal_directions; h++)
			{
				s[write_index++] = (m->get_cell(r, c)->IsWall(h)) ? heading_names[h] : ' ';
			}
			s[write_index++] = (m->get_cell(r, c)->get_visited()) ? '!' : ' ';
			s[write_index++] = '0' + *((int32_t*)m->get_cell(r, c)->get_data()) / 10;
			s[write_index++] = '0' + *((int32_t*)m->get_cell(r, c)->get_data()) % 10;
			s[write_index++] = m->IsGoalCell(r, c) ? '#' : '|';
		}
		s[write_index++] = '\n';
	}
	s[write_index] = '\0';
	return s;
} // ToString()


/*****************************************************************************
* Function: InitializeCellData
*
* Description:	allocate and assign cell data structs into all the cells
*				of the maze
*****************************************************************************/
void SimpleFloodFill::InitializeCellData
	(
		Cell* c
	)
{
	c->set_data( new int32_t );
}


/*****************************************************************************
* Function: InitializeCellData
*
* Description:	reset all cell data to the default weight
*****************************************************************************/
void SimpleFloodFill::ResetCellData
	(
		Cell* c
	)
{
	*((int32_t*)c->get_data()) = INITIAL_VALUE;
}