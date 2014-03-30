/****************************************************************************************
* File: micromouse.h
*
* Description: Header file for micromouse.cpp
*
* Created: 01/25/2013, by Kyle McGahee
****************************************************************************************/

#ifndef MICROMOUSE_INCLUDED_H
#define MICROMOUSE_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <stdint.h>

#include "differential_paired_stepper_motors.h"
#include "distance_sensors_interface.h"
#include "frame_of_references.h"
#include "ipathfinder.h"
#include "maze.h"
#include "pid.h"

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

// Thresholds that determine the distance (in centimeters) that a wall is there or not there.
// Referenced from middle of cell.
typedef struct
{
    float side;
    float diagonal;
    float front;
} wall_threshold_t;

// Information regarding the current target cell that we need to get to.
typedef struct
{
    position_t  position;         // Cell position in maze.
    bool        has_been_visited; // True if have already been to cell.
    bool        is_middle;        // True if cell is one of the middle cells.
    cardinal_t  next_heading;     // What direction to head after reaching cell.
                                  // Only valid if have already visited cell.
} target_cell_t;

// Type of checkpoint that can be setup to trigger an action.
typedef enum
{
    maze_evaluate_checkpoint,
    turn_checkpoint,
    maze_solve_checkpoint,
} checkpoint_type_t;

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: Micromouse
*
* Description: TODO
******************************************************************************/
class Micromouse
{
public: // methods

    // Constructor
    Micromouse
        (
            Maze                            & maze,                 // Maze to solve.
            IPathFinder                     & path_finder,          // Used to find center of maze.
            IDistanceSensors                & sensors,              // Sensors to find distance to walls.
            DifferentialPairedStepperMotors & motors,               // Differential motor driver reference.
            PID                             & centering_controller, // Controller for staying in middle of cell.
            wall_threshold_t          const & thresholds,           // Maximum distances from center of cell for a wall to be detected.
            float                             travelling_speed      // Speed to move through maze (centimeters / second)
        );

    // Attempts to find the middle of the maze.  Returns true if successful.
    // If want to resolve then should physically move to start cell and call
    // ResetToStartingCell() before calling this again.
    bool SolveMaze(void);

    // Reset robot's orientation/position to that of the starting cell.
    void ResetToStartingCell(void);

    // Can be called directly for debugging. Usually just need to call SolveMaze().
    // Uses a 'centering' controller to keep robot in the middle of the cells. The function
    // will return once the next checkpoint location is reached.
    void TravelForward
        (
            float distance_to_travel // In centimeters
        );

    // Setters
    void set_travelling_speed(float new_speed) { this->travelling_speed = new_speed; }

    // Getters. Distances are in centimeters and angles are in radians.
    float get_foward_angle(void) const { return this->forward_angle; }
    float get_x_distance(void) const { return this->net_x_distance; }
    float get_y_distance(void) const { return this->net_y_distance; }
    position_t get_position(void) const { return this->current_position; }
    heading_t get_heading(void) const { return this->current_heading; }

private: // methods

    // Determines what type of checkpoint needs to occur next and the physical location in
    // the maze that the checkpoint resides. The updated waypoint data is stored in the
    // 'current waypoint' field.
    void SetupNextCheckpoint(void);

    // Returns distance in centimeters between our current location and the center of
    // specified cell.  The input cell must be in a forward (or backward) direction
    // from the current heading.
    float CalculateDistanceToCenterOfCell
        (
            position_t cell // Cell position to calculate distance to.
        );

    // Updates net centimeters travelled in each axes into maze with the specificied
    // forward and lateral distance.
    void UpdateNetLocation
        (
            float forward_distance, // Just forward movement.
            float lateral_distance  // Side to side movement. Positive to the right of forward direction.
        );

    // Maps all walls for the cell the robot is moving into.  Should be called whenever a
    // new cell is being explored.
    void UpdateWalls(void);

    // Stops robot then performs special actions when finding middle of maze.
    void HandleMazeSolve(void);

    // Updates maze will wall and location information and then finds next target cell
    // using maze solver.
    void EvaluateMaze(void);

    // Uses the current maze evaluation algorithm in order to determine how many cells we
    // need to go forward before we do something. Using this information and where we've
    // been a target cell is setup.
    void FindNextPathSegment(void);

    // Determine what cell to try to get to next.  Must be in forward direction of robot.
    void SetupTargetCell
        (
            uint16_t   cells_to_travel,  // Number of cells to travel in current heading.
            cardinal_t next_heading      // Where to go once hit target checkpoint.
        );

    // Attempts to determine which heading (east or south) that we were actually facing
    // when we started.  Returns true if can tell and sets original heading and tranposes
    // and walls that have been mapped as well as the current cell.
    bool DetermineOriginalHeading
        (
            bool is_wall_on_right,
            bool is_wall_on_left
        );

    // Turns the specified direction taking into account any error in forward angle.
    void Turn
        (
            direction_t direction_to_turn
        );

    // Keeps robot in middle of the cells when driving forward through the maze.
    void Center(void);

    // Returns estimated distance (centimeters) to right wall at current time.
    float MeasureDistanceToRightWall(void);

    // Returns cell in front of robot by specified offset.
    position_t ForwardPosition
        (
            uint16_t forward_cell_offset  // Number of cells to in 'FRONT' direction.
        );

    // Bounds both x and y location of cell to the minimum and maximum boundaries.
    void CapPositionToMazeSize
        (
            position_t * cell // Reference to cell to cap.
        );

private: // fields

    bool in_middle; // Set to true if currently in one of middle maze cells.

    // Reference to maze to solve.
    Maze & maze;

    // Used to find center of maze.
    IPathFinder & path_finder;

    // Sensors to find distance to walls.
    IDistanceSensors & sensors;

    // Reference to motors that share the same axis of rotation. (Side by side)
    DifferentialPairedStepperMotors & motors;

    // Maximum distances from center of cell for a wall to be detected.
    wall_threshold_t thresholds;

    // Controller for staying in middle of cell.
    PID & centering_controller;

    // Heading at start position (could be either east or south). Need to make at least
    // one turn to find this out.
    cardinal_t original_heading;

    // True if we known for sure what our original heading is.
    bool know_original_heading;

    // The current cardinal direction the robot is heading (North, East, ect).
    cardinal_t current_heading;

    // Defines (x,y) location of cell the robot is in.
    position_t current_position;

    // How many time micromouse has reach middle.
    uint8_t num_maze_solves;

    // Measure in first cell before solving maze.  This is the distance that the centering
    // controller tries to maintain.
    float right_wall_calibrated_distance;

    // Net distance travelled east and south in centimeters.  Defines absolute position
    // in maze (not just what cell we're in).
    float net_x_distance;
    float net_y_distance;

    // When starting out the robot will have some offset already into the maze.  The following
    // two distances (in centimeters) are that offset to the center of the robot.
    float starting_x_distance;
    float starting_y_distance;

    // The angle of robot relative to forward being 0 radians.  Increase clockwise. (in radians)
    float forward_angle;

    // Current target cell information.  Updated after every regular checkpoint.
    target_cell_t target_cell;

    // Defines type of action to trigger when checkpoint is reached.
    checkpoint_type_t current_checkpoint_type;

    // How many centimeters to travel to reach current checkpoint.
    float distance_to_checkpoint;

    // Speed to move through maze. (centimeters / second)
    float travelling_speed;

    // System time that last centering control loop was ran.
    double last_centering_time;

}; // Micromouse

#endif // MICROMOUSE_INCLUDED_H
