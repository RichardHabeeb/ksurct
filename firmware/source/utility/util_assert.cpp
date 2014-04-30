/****************************************************************************************
* File: util_assert.cpp
*
* Description: Provides assert utility messages that are called through assert macros
*              definited in 'util_assert.h'.
*
* Created: 12/30/2013, by Kyle McGahee
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <cstdio>
#include <cstring>
#include <cstdarg>

#include "util_assert.h"

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

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: util_assert_failed
*
* Description:  Called whenever an assert fails.  Logs assert message or
*               condition (if no explicit message provided) and then performs
*               the action specified.  The variable argument (...) are provided
*               so this function can be used similar to printf().
*
* Notes: A newline character is automatically appended to format message string.
*****************************************************************************/
void util_assert_failed
    (
        int action,             // What to do after logging.  Such as restarting system.
        char const * file_name, // File name that assert failed in.
        int line_number,        // Line number that assert failed on.
        char * format,          // Message to log.
        ...                     // Variable arguments. (just like printf() uses)
    )
{
    const int buffer_size = 256;
    char buffer[buffer_size];
    int current_index = 0;
    char const * just_file_name = NULL;

    // If line number is zero then meta logging is disabled.
    // Otherwise need to take it into account.
    if (line_number >= 0)
    {
        // By default __FILE__ expands to full filename path given to compiler.
        // Split path to make sure only get filename and extension.
        just_file_name = strrchr(file_name, '/');

        if (just_file_name == NULL)
        {
            just_file_name = strrchr(file_name, '\\');

            if (just_file_name == NULL)
            {
                just_file_name = file_name;
            }
        }

        current_index += snprintf(buffer, buffer_size, "File: %s %d: ", just_file_name, line_number);
    }

    // Copy variable arguments into buffer.
    va_list args;
    va_start(args, format);
    current_index += vsnprintf(buffer + current_index, buffer_size - current_index, format, args);
    current_index += 2; // Advance to next spot.
    va_end(args);

    // Need to make sure have at least three spots for carriage return, newline, and nul.
    // Only need to check '-2' instead of '-3' since existing nul from vsnprintf is saving
    // us one spot already.
    if (current_index >= buffer_size - 2)
    {
        current_index = buffer_size - 2;
    }

    // Copy newline character automatically since all asserts should be independent.
    buffer[current_index-2] = '\r'; //
    buffer[current_index-1] = '\n'; // Replace existing nul character with newline.
    buffer[current_index]   = '\0'; // Re-establish nul character to terminate string.

    // Log message. All debug prints are automatically logged to the SD card.
    // debug_printf(buffer);

    if (action == ASSERT_RESTART)
    {
        // Right now this is treated as the same as a STOP assert.
        //restart_system();
        // To see the output of printf select 'View'->'Terminal I/O' in the debugger window.
        printf(buffer);
        while(1);
    }
    else if (action == ASSERT_STOP)
    {
        // To see the output of printf select 'View'->'Terminal I/O' in the debugger window.
        printf(buffer);
        while(1);
    }

} // util_assert_failed()
