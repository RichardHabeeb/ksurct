/****************************************************************************************
* File: weightedpathfinding.cpp
*
* Description: TODO
*
* Created: 2/20/2014, by Richard Habeeb
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include "weightedpathfinding.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

// Algorithm Constants
// (used for speedup estimates)
#define			STEP_WEIGHT			2
#define			VISITED_STEP_WEIGHT	1
#define			TURN_WEIGHT			4
#define			INITIAL_WEIGHT		-1

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
* Function: WeightedPathfinding - Constructor
*
* Description: Allocates and initializes the cell data
*****************************************************************************/
WeightedPathfinding::WeightedPathfinding
	(
		Maze* m
	)
{
	m->Map(WeightedPathfinding::InitializeCellData);
	maze_max_dim = Maximum(m->get_number_rows(), m->get_number_columns());
	this->m = m;

} // WeightedPathfinding()

/*****************************************************************************
* Function: FindNextPathSegment
*
* Description:	Compute the fastest route throught the maze. Simulate maze
*				solutions and ouput the next heading and the number of cells
*				to travel forward to get to that turn.
*****************************************************************************/
void WeightedPathfinding::FindNextPathSegment
	(
		uint32_t		robot_current_row, // the current row of the robot
		uint32_t		robot_current_col,  // the current col of the robot
		heading_t		robot_current_heading, // the current heading of the robot
		heading_t*		next_heading, //out param of the next heading to travel
		uint32_t*		cells_to_travel // out param of the number of cells to travel in the given direction
	)
{
	Cell* start_cell	= m->get_cell(robot_current_row, robot_current_col);
	Cell* goal_cell		= m->get_goal_cell();

	m->Map(WeightedPathfinding::ResetCellData);
	((cell_data_t*)start_cell->get_data())->robot_heading_sim = robot_current_heading;
	((cell_data_t*)start_cell->get_data())->weight = 0;

	cell_q.Reset()->Enqueue(start_cell);

	while (cell_q.get_count() > 0)
	{
		Cell*				current_cell			= cell_q.Dequeue();
		cell_data_t*		current_cell_data		= (cell_data_t*)current_cell->get_data();

		for (heading_t h = north; h < num_cardinal_directions; h++)
		{
			Cell*			adjacent_cell			= current_cell->get_adjacent_cell(h);
			if (adjacent_cell != _NULL)
			{

				cell_data_t*	adjacent_cell_data = (cell_data_t*)adjacent_cell->get_data();

				int32_t new_weight = current_cell_data->weight
					+ (HeadingDistance(h, current_cell_data->robot_heading_sim)*TURN_WEIGHT)
					+ ((adjacent_cell->get_visited()) ? VISITED_STEP_WEIGHT : STEP_WEIGHT);

				if (adjacent_cell != _NULL
					&& (adjacent_cell_data->weight > new_weight
						|| adjacent_cell_data->weight == INITIAL_WEIGHT))
				{
					adjacent_cell_data->weight				= new_weight;
					adjacent_cell_data->robot_heading_sim	= h;
					current_cell_data->next_cell			= adjacent_cell;
					cell_q.Enqueue(adjacent_cell);
				}
			}
		}
	}


	Cell* current_cell	= start_cell;
	*cells_to_travel = ~0;

	while (current_cell != _NULL && current_cell != goal_cell && robot_current_heading == ((cell_data_t*)current_cell->get_data())->robot_heading_sim)
	{
		*cells_to_travel += 1;
		current_cell = ((cell_data_t*)current_cell->get_data())->next_cell;
	}
	*next_heading = ((cell_data_t*)current_cell->get_data())->robot_heading_sim;

} // WeightedPathfinding::FindNextPathSegment()

/*****************************************************************************
* Function: FoundDestination
*
* Description:	Keep track of whether the robot is going to the middle or
*               going to the corner.
*****************************************************************************/
void WeightedPathfinding::FoundDestination(void)
{
    //TODO
} //WeightedPathfinding::FoundDestination()

/*****************************************************************************
* Function: ToString
*
* Description:	For debugging. Print the maze walls with cell data, will
*				allocate memory on the heap
*****************************************************************************/
char* WeightedPathfinding::ToString(void)
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
				s[write_index++] = (m->get_cell(r,c)->IsWall(h)) ? heading_names[h] : ' ';
			}
			s[write_index++] = (m->get_cell(r, c)->get_visited()) ? '!' : ' ';
			s[write_index++] = '0' + ((cell_data_t*)m->get_cell(r, c)->get_data())->weight / 10;
			s[write_index++] = '0' + ((cell_data_t*)m->get_cell(r, c)->get_data())->weight % 10;
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
void WeightedPathfinding::InitializeCellData
	(
		Cell* c
	)
{
	cell_data_t* d			= new cell_data_t();
	d->next_cell			= _NULL;
	d->robot_heading_sim	= north;
	d->weight				= INITIAL_WEIGHT;
	c->set_data(d);
}  // InitializeCellData()


/*****************************************************************************
* Function: InitializeCellData
*
* Description:	reset all cell data to the default weight
*****************************************************************************/
void WeightedPathfinding::ResetCellData
	(
        Cell* c
	)
{
	cell_data_t* d			= (cell_data_t*) c->get_data();
	d->next_cell			= _NULL;
	d->robot_heading_sim	= north;
	d->weight				= INITIAL_WEIGHT;
}  // ResetCellData()