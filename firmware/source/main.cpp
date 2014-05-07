/****************************************************************************************
* File: main.cpp
*
* Description: Configures micromouse depending on configuration settings and starts system
*              clock.  Then tells micromouse to solve maze.
*
* Created: 12/2/2013, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <cstdio>

#include "stm32f4xx.h"
#include "micromouse.h"
#include "public_system_config.h"
#include "system_timer.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
                                    MEMORY CONSTANTS
---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      VARIABLES
*--------------------------------------------------------------------------------------*/

SystemTimer system_timer;

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

// Forward declarations.
void test_motors(PairedMotors & motors);
void test_sensors(IDistanceSensors & sensors);

/*****************************************************************************
* Function: main
*
* Description: Configures micromouse depending on configuration settings and starts system
*              clock.  Then tells micromouse to solve maze.
*****************************************************************************/
int main(void)
{
    Micromouse * micromouse = configure_micromouse();

    system_timer.Initialize(1e6);
    
    micromouse->ConfigureRobotMenu();

    while (true)
    {
        micromouse->SolveMaze();
    }

    // return 0; KLM: Remove warning: statement is unreachable.

} // main()

/*****************************************************************************
* Function: test_motors
*
* Description: TODO
*****************************************************************************/
void test_motors(PairedMotors & motors)
{
    // ZeroPointTurn test.
    motors.ZeroPointTurn(turn_right, 90, 3);

    // ArcTurn test.
    //motors.Drive(10);
    //motors.ArcTurn(turn_right, 90, 9);

    double time = system_timer.get_time();
    while (system_timer.get_time() < time + 3.f);

    while (true)
    {
        for (uint32_t i = 0; i < 5e6; ++i);
        motors.Drive(5);
        motors.reset_current_distance();
        while (motors.get_right_motor().get_current_distance() < 30.f); //3 * 3.14);
        motors.Stop();
        while(1);

        //printf("\n\n\n%f", motors.get_left_motor().get_current_distance());
        //printf("\n%f", motors.get_right_motor().get_current_distance());
    }

} // test_motors()

/*****************************************************************************
* Function: test_sensors
*
* Description: TODO
*****************************************************************************/
void test_sensors(IDistanceSensors & sensors)
{
    sensors.CalibrateSensor( sensor_id_front, 8.0f );

    double time;

    while (true)
    {
        time = system_timer.get_time();
        while (system_timer.get_time() < time + 2.f);
        printf("\n\nl:  %f",sensors.ReadDistance(sensor_id_left));
        printf("\nfl: %f",  sensors.ReadDistance(sensor_id_front_nw));
        printf("\nf:  %f",  sensors.ReadDistance(sensor_id_front));
        printf("\nfr: %f",  sensors.ReadDistance(sensor_id_front_ne));
        printf("\nr:  %f",  sensors.ReadDistance(sensor_id_right));
    }

} // test_sensors()