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

    //test_motors(micromouse->get_motors());
    //test_sensors(micromouse->get_sensors());

    while (true)
    {
        if (!micromouse->SolveMaze())
        {
            // :(
        }
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
    //motors.ZeroPointTurn(turn_right, 90, 45);

    // ArcTurn test.
    //motors.Drive(10);
    //motors.ArcTurn(turn_right, 90, 9);

    while (true)
    {
        for (uint32_t i = 0; i < 5e6; ++i);
        motors.Drive(20);
        while(1);
        //motors.reset_current_distance();
        //while (motors.get_right_motor().get_current_distance() < 3 * 3.14);
        //motors.Stop();
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
    while (true)
    {
        volatile float left        = sensors.ReadDistance(sensor_id_left);
        volatile float front_left  = sensors.ReadDistance(sensor_id_front_nw);
        volatile float front       = sensors.ReadDistance(sensor_id_front);
        volatile float front_right = sensors.ReadDistance(sensor_id_front_ne);
        volatile float right       = sensors.ReadDistance(sensor_id_right);
        printf("\n\nl:  %f", left);
        printf("\nfl: %f", front_left);
        printf("\nf:  %f", front);
        printf("\nfr: %f", front_right);
        printf("\nr:  %f", right);
    }

} // test_sensors()