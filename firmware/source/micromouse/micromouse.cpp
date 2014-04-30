/****************************************************************************************
* File: micromouse.cpp
*
* Description: Represents a robot capable of solving autonomously using a maze.
*              The robot is equipped with distance sensors and two differentially
*              paired motors for movement.
*
* Created: 01/25/2014, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <cmath>
#include <cstdio>
#include <cstring>

#include "micromouse.h"
#include "system_timer.h"
#include "util_math.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                   MEMORY CONSTANTS
---------------------------------------------------------------------------------------*/

// Minimum amount of time to wait before running centering controls again.
const float minimum_centering_period = .01; // seconds

// Distance (in centimeters) before the target cell center that the maze evaulation action
// (map walls/flood maze/setup target information, etc) needs to occur.
const float maze_evaluate_checkpoint_offset_distance = 5.5f;

/*---------------------------------------------------------------------------------------
*                                      VARIABLES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: Micromouse
*
* Description: Constructor
*****************************************************************************/
Micromouse::Micromouse
    (
        Maze                    & maze,                 // Maze to solve.
        IPathFinder             & path_finder,          // Used to find center of maze.
        IDistanceSensors        & sensors,              // Sensors to find distance to walls.
        PairedMotors            & motors,               // Differential paired motors.
        PID                     & centering_controller, // Controller for staying in middle of cell.
        wall_threshold_t  const & thresholds,           // Maximum distances from center of cell for a wall to be detected.
        float                     travelling_speed,     // Speed to move through maze (centimeters / second)
        float                     turning_speed         // Rotational turning speed of robot (degrees / second)
    ) :
    maze(maze),
    path_finder(path_finder),
    sensors(sensors),
    motors(motors),
    centering_controller(centering_controller)
{
    this->travelling_speed = travelling_speed;

    this->turning_speed = turning_speed;

    this->thresholds = thresholds;

    in_middle = false;

    original_heading = east; // Default starting heading. Might be changed in runtime to South.

    know_original_heading = false; // We will once we hit first turn.

    current_heading = original_heading;

    num_maze_solves = 0;

    right_wall_calibrated_distance = 0.f;

    starting_x_distance = 5.0f;
    starting_y_distance = maze.get_cell_length() / 2.f;

    net_x_distance = starting_x_distance;
    net_y_distance = starting_y_distance;

    forward_angle = 0.f;

    current_position.x = 0;
    current_position.y = 0;

    memset(&target_cell, 0, sizeof(target_cell));

    current_checkpoint_type = maze_evaluate_checkpoint;

    distance_to_checkpoint = 0.f;

    last_centering_time = 0.0;

} // Micromouse::Micromouse() - Constructor

/*****************************************************************************
* Function: ResetToStartingCell
*
* Description: Reset robot's orientation/position to that of the starting cell.
*****************************************************************************/
void Micromouse::ResetToStartingCell(void)
{
    in_middle = false;

    current_heading = original_heading;

    net_x_distance = starting_x_distance;
    net_y_distance = starting_y_distance;

    forward_angle = 0.f;

    current_position.x = 0;
    current_position.y = 0;

} // Micromouse::ResetToStartingCell()

/*****************************************************************************
* Function: SolveMaze
*
* Description: Attempts to find the middle of the maze.  Returns true if successful.
*****************************************************************************/
bool Micromouse::SolveMaze(void)
{
    // TODO: Determine if need to actually calibrate this at start. Currently it's only
    //       being used for centering in cell.  Could also use as an offset to the mapping
    //       defined by sensors class.
    right_wall_calibrated_distance = sensors.Calibrate();

    maze.get_cell(current_position)->set_visited(true);

    // Need to do an initial maze evaluation in order to get our bearings and determine
    // the cell that's going to be our initial target cell.
    EvaluateMaze();

    while (!in_middle)
    {
        // Determine distance to next checkpoint and what type of checkpoint it is.
        SetupNextCheckpoint();

        TravelForward(distance_to_checkpoint);

        current_position = target_cell.position;

        //printf("\n(%d,%d)", current_position.x, current_position.y);

        switch (current_checkpoint_type)
        {
            case maze_evaluate_checkpoint:
                EvaluateMaze();
                break;
            case turn_checkpoint:
                Turn(target_cell.next_heading);
                FindNextPathSegment();
                break;
            case maze_solve_checkpoint:
                HandleMazeSolve();
                break;
        }
    }

    return true;

} // Micromouse::SolveMaze()


/*****************************************************************************
* Function: SetupNextCheckpoint
*
* Description: Determines what type of checkpoint needs to occur next and the
*              physical location in the maze that the checkpoint resides. The
*              updated waypoint data is stored in the 'current waypoint' field.
*****************************************************************************/
void Micromouse::SetupNextCheckpoint(void)
{
    float distance_to_center_of_target_cell = CalculateDistanceToCenterOfCell(target_cell.position);

    if (target_cell.is_middle)
    {
        current_checkpoint_type = maze_solve_checkpoint;

        distance_to_checkpoint = distance_to_center_of_target_cell;
    }
    else if (!target_cell.has_been_visited)
    {
        current_checkpoint_type = maze_evaluate_checkpoint;

        distance_to_checkpoint = distance_to_center_of_target_cell - maze_evaluate_checkpoint_offset_distance;
    }
    else // We know where we're going so setup a turn checkpoint.
    {
        current_checkpoint_type = turn_checkpoint;

        // Right now we're just doing a zero point turn so want to stop in middle of cell.
        distance_to_checkpoint = distance_to_center_of_target_cell;
    }

} // Micromouse::SetupNextCheckpoint()


/*****************************************************************************
* Function: CalculateDistanceToCenterOfCell
*
* Description: Returns distance in centimeters between our current location
*              and the center of specified cell.  The input cell must be in
*              a forward (or backward) direction from the current heading.
*****************************************************************************/
float Micromouse::CalculateDistanceToCenterOfCell
    (
        position_t cell // Cell position to calculate distance to.
    )
{
    float distance_to_middle_of_cell = 0.f;

    // The main point here is to calculate how for the robot would have to travel to arrive in
    // the center of the next cell
    if (current_heading == east || current_heading == west)
    {
        float cell_x_distance = cell.x * maze.get_cell_length() + (maze.get_cell_length() / 2.0f);

        distance_to_middle_of_cell = AbsoluteValue(cell_x_distance - net_x_distance);
    }
    else if (current_heading == north || current_heading == south)
    {
        float cell_y_distance = cell.y * maze.get_cell_length() + (maze.get_cell_length() / 2.0f);

        distance_to_middle_of_cell = AbsoluteValue(cell_y_distance - net_y_distance);
    }

    return distance_to_middle_of_cell;

} // Micromouse::CalculateDistanceToCenterOfCell()


/*****************************************************************************
* Function: TravelForward
*
* Description: Uses a 'centering' controller to keep robot in the middle of
*              the cells. The function will return once the specified distance
*              is travelled.
*****************************************************************************/
void Micromouse::TravelForward
    (
        float distance_to_travel // In centimeters
    )
{
    // Current distance travelled not counting lateral error.
    float forward_distance = 0.f;

    // Current lateral distance. Positive to right of robot's forward direction.
    float lateral_distance = 0.f;

    // Need variables to remember how far we travelled the previous time through the loop.
    float last_right_distance = 0.0f;
    float last_left_distance  = 0.0f;

    // Reset current distance so we know how far we've travelled.
    motors.reset_current_distance();

    // Start wheels spinning forward (might already be moving forward)
    // This will also reset the current distance travelled reported by motors.
    motors.Drive(travelling_speed);

    while (forward_distance < distance_to_travel)
    {
        this->Center();

        float right_distance = motors.get_right_motor().get_current_distance();
        float left_distance  = motors.get_left_motor().get_current_distance();

        // Estimate where robot is pointing.  Since this is information is needed elsewhere
        // (such as turning) it is a field. The angle is referenced from 0 radians being in
        // from of robot.  Increases clockwise.
        // From small angle approximation for arcs then applying law of cosines.
        // http://en.wikipedia.org/wiki/Odometry
        forward_angle = (PI / 2.f) - acosf((right_distance - left_distance) / (2.f * motors.get_wheel_base_distance()));

        // Calculate how far forward each wheel has travelled since last time.
        float delta_right_distance = right_distance - last_right_distance;
        float delta_left_distance  = left_distance  - last_left_distance;

        // Calculate incremental distance that center of robot has moved.
        float delta_distance_travelled = fabs(delta_right_distance + delta_left_distance) / 2.f;

        forward_distance += delta_distance_travelled * cosf(forward_angle);
        lateral_distance += delta_distance_travelled * sinf(forward_angle);

        // Save values so next time through loop can remember how far we've travelled.
        last_right_distance = right_distance;
        last_left_distance  = left_distance;
    }

    UpdateNetLocation(forward_distance, lateral_distance);

} // Micromouse::TravelForward()

/*****************************************************************************
* Function: UpdateNetLocation
*
* Description: Updates net centimeters travelled in each axes into maze with
*              the specificied forward and lateral distance.
*****************************************************************************/
void Micromouse::UpdateNetLocation
    (
        float forward_distance, // Just forward movement.
        float lateral_distance  // Side to side movement. Positive to the right of forward direction.
    )
{
    switch (current_heading)
    {
        case north:
            net_y_distance -= forward_distance;
            net_x_distance += lateral_distance;
            break;
        case east:
            net_x_distance += forward_distance;
            net_y_distance += lateral_distance;
            break;
        case south:
            net_y_distance += forward_distance;
            net_x_distance -= lateral_distance;
            break;
        case west:
            net_x_distance -= forward_distance;
            net_y_distance -= lateral_distance;
            break;
    }

} // Micromouse::UpdateNetLocation()

/*****************************************************************************
* Function: UpdateWalls
*
* Description: Maps all walls for the current cell position.  Should be called
*              whenever a new cell is being explored.
*****************************************************************************/
void Micromouse::UpdateWalls(void)
{
    float right_side_distance = sensors.ReadDistance(sensor_id_right);
    float left_side_distance  = sensors.ReadDistance(sensor_id_left);
    float front_distance      = sensors.ReadDistance(sensor_id_front);

    bool is_wall_on_right = right_side_distance <= thresholds.side;
    bool is_wall_on_left  = left_side_distance  <= thresholds.side;
    bool is_wall_in_front = front_distance      <= (thresholds.front + maze_evaluate_checkpoint_offset_distance);

    if (!know_original_heading)
    {
        DetermineOriginalHeading(is_wall_on_right, is_wall_on_left);
    }

    // Even if we still don't know our original heading still update walls so can run
    // maze evaluator regardless.  It's easy to switch the walls later.

    Cell * current_cell = maze.get_cell(current_position);

    if (is_wall_on_right)
    {
        current_cell->set_wall(ConvertToHeading(right, current_heading));
    }
    if (is_wall_on_left)
    {
        current_cell->set_wall(ConvertToHeading(left, current_heading));
    }
    if (is_wall_in_front)
    {
        current_cell->set_wall(ConvertToHeading(forward, current_heading));
    }

} // Micromouse::UpdateWalls()


/*****************************************************************************
* Function: DetermineOriginalHeading
*
* Description: Attempts to determine which heading (east or south) that we
*              were actually facing when we started.  Returns true if can
*              tell and sets original heading and tranposes and walls that
*              have been mapped as well as the current cell.
*****************************************************************************/
bool Micromouse::DetermineOriginalHeading
    (
        bool is_wall_on_right,
        bool is_wall_on_left
    )
{
    // Can't just automatically update walls if we're not sure what our original
    // heading is. We can figure that out once there's an open wall next to us.
    // Once we figure this out we can remap all walls to the correct heading
    // if our original guess was wrong.
    bool guessed_original_heading = true;

    know_original_heading = (!is_wall_on_left || !is_wall_on_right);

    if (!is_wall_on_right && original_heading == south)
    {
        guessed_original_heading = false;
        original_heading = east;
    }
    else if (!is_wall_on_left && original_heading == east)
    {
        guessed_original_heading = false;
        original_heading = south;
    }

    if (!guessed_original_heading)
    {
        current_heading = original_heading;
        Swap(current_position.x, current_position.y);
        Swap(net_x_distance, net_y_distance);
        maze.Transpose();
    }

    return know_original_heading;

} // Micromouse::DetermineOriginalHeading()


/*****************************************************************************
* Function: Center
*
* Description: Keeps robot in middle of the cells when driving forward through
*              the maze.   Needs at least a left OR a right wall to be effective.
*              If no walls are currently available then robot will not add any
*              motor speed offsets.
*****************************************************************************/
void Micromouse::Center(void)
{
    double current_time = system_timer.get_time();

    float delta_time = (float)(current_time - last_centering_time);

    // Only want to run controls at a maximum frequency so make sure enough time has
    // elapsed before trying to run PID calculation again.
    if (delta_time >= minimum_centering_period)
    {
        // In order to avoid large delta times due to not constantly centering (ie turning / etc)
        // fix change in time to minimum period.  This should be ok since this method is getting
        // constantly run when travelling forward.
        delta_time = minimum_centering_period;

        float measured_distance = MeasureDistanceToRightWall();

        float commanded_distance = right_wall_calibrated_distance;

        float distance_error = commanded_distance - measured_distance;

        float delta_linear_speed = centering_controller.Calculate(distance_error, delta_time);

        motors.get_right_motor().Drive(travelling_speed + delta_linear_speed);
        motors.get_left_motor().Drive(travelling_speed - delta_linear_speed);

        last_centering_time = current_time;
    }

} // Micromouse::Center()


/*****************************************************************************
* Function: MeasureDistanceToRightWall
*
* Description: Returns estimated distance (centimeters) to right wall at current time.
*              If no left or right wall available then will just return initial
*              calibration reading from right sensor.
*****************************************************************************/
float Micromouse::MeasureDistanceToRightWall(void)
{
    float right_side_distance = sensors.ReadDistance(sensor_id_right);
    float left_side_distance  = sensors.ReadDistance(sensor_id_left);

    bool is_wall_on_right = right_side_distance <= thresholds.side;
    bool is_wall_on_left  = left_side_distance  <= thresholds.side;

    float distance_to_wall = 0.f;

    if (is_wall_on_right)
    {
        distance_to_wall = right_side_distance;
    }
    else if (is_wall_on_left)
    {
        distance_to_wall = maze.get_cell_length() - left_side_distance;
    }
    else // No walls to use for balancing.
    {
        distance_to_wall = right_wall_calibrated_distance;
    }

    return distance_to_wall;

} // Micromouse::MeasureDistanceToRightWall()


/*****************************************************************************
* Function: FindNextPathSegment
*
* Description: Uses the current maze evaluation algorithm in order to determine
*              how many cells we need to go forward before we do something.
*              Using this information and where we've been a target cell is setup.
*****************************************************************************/
void Micromouse::FindNextPathSegment(void)
{
    uint32_t cells_to_travel = 0;
    cardinal_t next_heading = north;

    path_finder.FindNextPathSegment(current_position.y, current_position.x, current_heading, &next_heading, &cells_to_travel);
    SetupTargetCell(cells_to_travel, next_heading);

} // Micromouse::FindNextPathSegment()


/*****************************************************************************
* Function: SetupTargetCell
*
* Description: Depending on where we've already been define the target cell
*              we're trying to get to.  For example if # cells to travel is
*              6 but we haven't been to the next cell yet then that next
*              cell will become the target cell since we need to do a
*              maze evaluation of that cell.
*****************************************************************************/
void Micromouse::SetupTargetCell
    (
        uint16_t   cells_to_travel,  // Number of cells to travel in current heading.
        cardinal_t next_heading      // Where to go once hit target checkpoint.
    )
{
    target_cell_t new_target;

    // Setup where to go after target is hit.  Only valid if have been to target cell before.
    new_target.next_heading = next_heading;

    // Determine the next cell to go to.
    for (uint16_t forward_offset = 0; forward_offset <= cells_to_travel; ++forward_offset)
    {
        new_target.position = ForwardPosition(forward_offset);

        new_target.has_been_visited = maze.get_cell(new_target.position)->get_visited();

        if (!new_target.has_been_visited)
        {
            break; // Haven't been to cell on segment so must stop there to evaluate maze.
        }
    }

    new_target.is_middle = maze.IsGoalCell(new_target.position.y, new_target.position.x);

    this->target_cell = new_target;

} // Micromouse::SetupTargetCell()


/*****************************************************************************
* Function: EvaluateMaze
*
* Description: Updates maze will wall and location information and then finds
*              next target cell using maze solver.
*****************************************************************************/
void Micromouse::EvaluateMaze(void)
{
    UpdateWalls();

    maze.get_cell(current_position)->set_visited(true);

    FindNextPathSegment();

} // Micromouse::EvaluateMaze()

/*****************************************************************************
* Function: Turn
*
* Description: Turns to face the specified heading while also taking into account
*              any forward angle error.
*****************************************************************************/
void Micromouse::Turn
    (
        heading_t heading_to_face // New heading to point torwards (north, east, etc)
    )
{
    direction_t direction_to_turn = ConvertToDirection(heading_to_face, current_heading);

    float forward_angle_in_degrees = forward_angle * degrees_per_radian;

    switch (direction_to_turn)
    {
        case right:
            motors.ZeroPointTurn(turn_right, 90.f - forward_angle_in_degrees, turning_speed);
            break;
        case left:
            motors.ZeroPointTurn(turn_left, 90.f + forward_angle_in_degrees, turning_speed);
            break;
        case backward:
            motors.ZeroPointTurn(turn_left, 180.f + forward_angle_in_degrees, turning_speed);
            break;
    }

    current_heading = heading_to_face;

    // Turning is our reset case for heading used for odometry.
    forward_angle = 0.f;

    // Since we turned we now want to start summation error back to zero.
    centering_controller.reset_integral_error();

} // Micromouse::Turn()

/*****************************************************************************
* Function: HandleMazeSolve
*
* Description: Stops robot then performs special actions when finding middle of maze.
*****************************************************************************/
void Micromouse::HandleMazeSolve(void)
{
    motors.Stop();

    num_maze_solves++; // Ohhhhhhh yeeeeeeah

    // If first time solved maze then (assuming there's only one entrance to middle square) we can
    // now mark rest of square as walls.
    if (num_maze_solves == 1)
    {
        maze.MapCenterSquareWalls(current_position.y, current_position.x, current_heading);
    }

    in_middle = true;

} // Micromouse::HandleMazeSolve()

/*****************************************************************************
* Function: ForwardPosition
*
* Description: Returns cell in front of robot.  Example if specified cell
*              offset is 1 then then the cell directly in front will be returned.
*              If goes out of maze then location will be capped at edge cell.
*****************************************************************************/
position_t Micromouse::ForwardPosition
    (
        uint16_t forward_cell_offset  // Number of cells to in 'front' direction.
    )
{
    position_t forward_cell = current_position;

    switch (current_heading)
    {
        case north:   forward_cell.y -= forward_cell_offset;    break;
        case east:    forward_cell.x += forward_cell_offset;    break;
        case south:   forward_cell.y += forward_cell_offset;    break;
        case west:    forward_cell.x -= forward_cell_offset;    break;
    }

    CapPositionToMazeSize(&forward_cell);

    return forward_cell;

} // Micromouse::ForwardPosition()

/*****************************************************************************
* Function: CapPositionToMazeSize
*
* Description: Bounds both x and y location of cell to the minimum and maximum boundaries.
*****************************************************************************/
void Micromouse::CapPositionToMazeSize
    (
        position_t * cell // Reference to cell to cap.
    )
{
    if (cell == NULL) { return; }

    cell->x = CapBounds<uint8_t>(cell->x, 0, maze.get_number_columns() - 1);
    cell->y = CapBounds<uint8_t>(cell->y, 0, maze.get_number_rows() - 1);

} // Micromouse::CapPositionToMazeSize()
