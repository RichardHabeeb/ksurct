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

#include "distance_sensors_interface.h"
#include "frame_of_references.h"
#include "ipathfinder.h"
#include "maze.h"
#include "paired_motors.h"
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

// Contains the offsets of the sensors to the center of the robot.
typedef struct
{
    float side;     // Offset from the front of the side sensor
    float diagonal; // Offset from the front of the diagonal sensor
    float front;    // Offset from the front of the front sensor
} sensor_offset_t;

// Information regarding the current target cell that we need to get to.
typedef struct
{
    position_t  position;         // Cell position in maze.
    bool        has_been_visited; // True if have already been to cell.
    bool        is_middle;        // True if cell is one of the middle cells.
    bool        is_start;        // True if cell is 0,0.
    cardinal_t  next_heading;     // What direction to head after reaching cell.
                                  // Only valid if have already visited cell.
} target_cell_t;

// Type of checkpoint that can be setup to trigger an action.
typedef enum
{
    maze_evaluate_checkpoint,
    turn_checkpoint,
    maze_solve_checkpoint,
    return_to_start_checkpoint
} checkpoint_type_t;

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: Micromouse
*
* Description: Represents a robot capable of solving autonomously using a maze.
*              The robot is equipped with distance sensors and two differentially
*              paired motors for movement.
******************************************************************************/
class Micromouse
{
public: // methods

    // Constructor
    Micromouse
        (
            Maze                    & maze,                 // Maze to solve.
            IPathFinder             & path_finder,          // Used to find center of maze.
            IDistanceSensors        & sensors,              // Sensors to find distance to walls.
            PairedMotors            & motors,               // Differential paired motors.
            PID                     & centering_controller, // Controller for staying in middle of cell.
            wall_threshold_t  const & thresholds,           // Maximum distances from center of cell for a wall to be detected.
            sensor_offset_t   const & sensor_offsets,       // The offset of each sensor to the center of the robot.
            float                     travelling_speed,     // Speed to move through maze (centimeters / second)
            float                     turning_speed         // Rotational turning speed of robot (degrees / second)
        );

    // Corrects for any constant offsets in direct sensor readings. Should only be called
    // when perfectly centered in cell and have walls on both sides.
    void CalibrateSensors(void);

    // Check to see if the side sensors are covered
    bool CheckForCoveredSensors(void);

    // Attempts to find the middle of the maze.  Returns true if successful.
    //   Once it does then will automatically go back to starting cell and re-solve it.
    void SolveMaze(void);

    // Reset robot's orientation/position to that of the starting cell.
    void ResetToStartingCell(void);

    // Can be called directly for debugging. Usually just need to call SolveMaze().
    // Uses a 'centering' controller to keep robot in the middle of the cells. The function
    // will return once the next checkpoint location is reached.
    void TravelForward
        (
            float distance_to_travel // In centimeters
        );

    // Wait for user input via sensors to begin SolveMaze
    // A speed selection menu / destroy maze /  reset position
    void ConfigureRobotMenu(void);

    // Setters
    void set_travelling_speed(float new_speed) { this->travelling_speed = new_speed; }

    // Getters. Distances are in centimeters and angles are in radians.
    float get_foward_angle(void) const { return this->forward_angle; }
    float get_x_distance(void) const { return this->net_x_distance; }
    float get_y_distance(void) const { return this->net_y_distance; }
    position_t get_position(void) const { return this->current_position; }
    heading_t get_heading(void) const { return this->current_heading; }

    // Allow access to components for testing outside of micromouse logic.
    PairedMotors & get_motors(void) { return motors; }
    IDistanceSensors & get_sensors(void) { return sensors; }

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

    // Returns how far the robot is from next cell in the specified heading. (in centimeters)
    float CalculateDistanceToNeighborCell
        (
            cardinal_t heading // Get to distance to next cell in this heading.
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

    // Stops robot then performs special actions when returning to the outer edge
    void HandleReturnToStart(void);

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

    // Turns to face the specified heading while also taking into account any forward angle error.
    void Turn
        (
            heading_t heading_to_face // New heading to point torwards (north, east, etc)
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

    bool IsWallReadingReliable(void);

    void IsRightDistanceReliable(void);
    // Reading functions that account for distance to center of robot. Should always be
    // used instead of directly reading in from sensors.
    inline float ReadRightDistance(void);
    inline float ReadLeftDistance(void);
    inline float ReadFrontDistance(void);
    inline float ReadRightDiagonalDistance(void);
    inline float ReadLeftDiagonalDistance(void);

private: // fields

    bool in_middle; // Set to true if currently in one of middle maze cells.

    // Reference to maze to solve.
    Maze & maze;

    // Used to find center of maze.
    IPathFinder & path_finder;

    // Sensors to find distance to walls.
    IDistanceSensors & sensors;

    // Reference to motors that share the same axis of rotation. (Side by side)
    PairedMotors & motors;

     // The offset of each sensor to the center of the robot.
    sensor_offset_t sensor_offsets;

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

    // Rotation turn speed. (degrees / second)
    float turning_speed;

    // System time that last centering control loop was ran.
    double last_centering_time;

}; // Micromouse

#endif // MICROMOUSE_INCLUDED_H
