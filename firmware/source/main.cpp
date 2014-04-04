/****************************************************************************************
* File: main.cpp
*
* Description: TODO
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

/*****************************************************************************
* Function: main
*
* Description: TODO
*****************************************************************************/
int main(void)
{
    Micromouse * micromouse = configure_micromouse();

    system_timer.Initialize(1e6);

    // Motors test.
//    PairedMotors & motors = micromouse->get_motors();
//    while (true)
//    {
//        for (uint32_t i = 0; i < 1e6; ++i);
//        motors.Drive(5);
//        for (uint32_t i = 0; i < 1e6; ++i);
//        printf("\n\n\n%f", motors.get_left_motor().get_current_distance());
//        printf("\n%f", motors.get_right_motor().get_current_distance());
//    }

    // Sensor test.
//    IDistanceSensors & sensors = micromouse->get_sensors();
//    while (true)
//    {
//        volatile float left        = sensors.ReadDistance(sensor_id_left);
//        volatile float front_left  = sensors.ReadDistance(sensor_id_front_nw);
//        volatile float front       = sensors.ReadDistance(sensor_id_front);
//        volatile float front_right = sensors.ReadDistance(sensor_id_front_ne);
//        volatile float right       = sensors.ReadDistance(sensor_id_right);
//        printf("\n\nl:  %f", left);
//        printf("\nfl: %f", front_left);
//        printf("\nf:  %f", front);
//        printf("\nfr: %f", front_right);
//        printf("\nr:  %f", right);
//    }

    while (true)
    {
        if (!micromouse->SolveMaze())
        {
            // :(
        }
    }

    // return 0; KLM: Remove warning: statement is unreachable.

} // main()
