/// This file is meant to be an example of the coding standards. However there's too much
/// to cover in a single file so if what you're looking for isn't in the file then refer
/// to the wiki on the project page.

/// Here are a list of some general rules.
/// - Code (including comments) should not exceed column 90.
/// - All filenames should be completely lowercase and separated by underscores ('_')
/// - Tabs shall be inserted as spaces (always use 4 spaces)
/// - Avoid trailing whitespace at end of lines
/// - End every file with exactly one blank line
/// - Parentheses and brackets shall be balanced, ie. no (abc ) or [ abc]
/// - Prefer pre-increment and decrement operators over post when you have the choice.
///   (ie ++i instead of i++)

/// Keep in mind there are exceptions to every rule.
/// If you feel you can break the coding standard in a certain case in order to make the code
/// more readable then do so.  An example would be a large table with multiple columns could
/// exceed past column 90.

/// In general ONLY use /* */ style comments for file, section, or function blocks.
/// So in most cases use double slash ('//') comments. I use triple slash in this example
/// to show that they're not real code comments, just narration comments like this one.

/// When ever you define a constant or variable that has some type of units associated with it
/// clearly define these units in the comment.

/// Since we're doing embedded work almost always use floats instead of doubles.  Do not mix the two.
/// If you need more precision/range then a float can offer then use a double, but be careful
/// in doing so.

/// Use your full name followed by a colon to leave a temporary comment.
/// Kyle McGahee: Some temporary comment relating to the code. This should not be left
///               in once code is finished.

/// A lot of influences in this coding standard came from Garmin's and Google's coding standards.

/****************************************************************************************
* File: Here simply name the file.  If the filename isn't completely clear to a newcomer
*       then put a dash (-) followed by a more descriptive name. See the example below.
* File: pid.cpp - Proportional Integral Derivative Control
*
* Description: Here provide a general overview of what the file is about. This is a simple
*              class so the description is short.  See the example below.
* Description: Implementation of PID class that provides generic proportional, integral,
*              derivative control calculations.
*
* Created: 10/22/2013, by Kyle McGahee
****************************************************************************************/

/// The following are called file section blocks.  They must be included in all *.cpp
/// files even if there is nothing in that section.  There should always be exactly one
/// empty line before and after each comment block.  In certain cases it makes sense to
/// add a new section to a specific file, this is fine.

/// **Note: Most of the types and constants in this file have nothing to do with PID.
///         They are just there for examples.

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

/// Header files shall be included in this order:
///  - ANSI standard headers
///  - public headers
/// A blank line shall separate each of the header categories.  The header
/// files shall be alphabetized within each category. ANSI header includes use the
/// <> characters.  All others use the "" characters.

#include <stdlib.h>
#include <string.h>

#include "cats.h"
#include "pid.h"
#include "util_math.h"
#include "zebras.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

#define SOME_CONSTANT_0    1000 // You can give each constant a description.
#define SOME_CONSTANT_1     128 // Some description.
#define SOME_CONSTANT_2      32 // Some other description.
#define SOME_CONSTANT_3       7 // Some other description.

// Some literal constants are better served by a single comment describing the bunch.
#define ALMOST_SELF_EXPLANATORY_1    1
#define ALMOST_SELF_EXPLANATORY_2    1
#define ALMOST_SELF_EXPLANATORY_3    2
#define ALMOST_SELF_EXPLANATORY_4    3
#define ALMOST_SELF_EXPLANATORY_5    5
#define ALMOST_SELF_EXPLANATORY_6    8

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/// All types should end in a '_t'

// Always have a comment above a struct that describes it.  Even if it's fields are
// also commented.  Note these field names are not descriptive enough and pxl_cnt is
// abbreviated by taking out letters.  Both of these are bad.  Instead the field names
// should be 'bits_per_pixel' and 'pixel_count'. 
typedef struct
{
    int     bpp;        // bits per pixel
    int     pxl_cnt;    // pixel count
} image_header_t;

// Example of enumerated type.
typedef enum
{
    NORTH,
    EAST,
    SOUTH,
    WEST
} cardinal_direction_t;

/*---------------------------------------------------------------------------------------
*                                   MEMORY CONSTANTS
*--------------------------------------------------------------------------------------*/

uint32_t const some_constant = 800; // Describe constant here with units if applicable

// Can also put description here if it is too long to fit on a single line single line it 
// just keeps going and going and going.
float const some_other_cool_constant = 75.323432;

/// **Note in C++ (not C) const implies internal linkage so you don't need to define
/// private const variables as 'static'.

/*---------------------------------------------------------------------------------------
*                                      VARIABLES
*--------------------------------------------------------------------------------------*/

/// Try to avoid using public global variables.
/// Use the 'static' keyword to make it so nothing in another file can see the variable.

static int         foo_call_count;        // Times foo() has been called
static a_data_type some_descriptive_name; // Some other value

/*---------------------------------------------------------------------------------------
*                                    CLASS METHODS  // Note this could be PROCEDURES or something else
*--------------------------------------------------------------------------------------*/

/// Always list arguments in a function starting with all 'input' arguments and
/// ending with all 'output' arguments.  An 'input' argument is something the
/// function uses.  An 'output' argument is something the function changes or
/// stores things in.  If a 'pointer' is NOT used as an 'output' argument then ALWAYS
/// qualify it as 'const'.  This way if you accidentally try to change it in the function
/// it will complain when compiling.

// If you have any function prototypes then put them right here. Before the first
// function/method body in the file.
int some_function(void);

/// From Google C++ style guide
/// "If there is anything tricky about how a function does its job, the function definition should have an explanatory comment.
/// For example, in the definition comment you might describe any coding tricks you use, give an overview of the steps you go through,
/// or explain why you chose to implement the function in the way you did rather than using a viable alternative. For instance,
/// you might mention why it must acquire a lock for the first half of the function but why it is not needed for the second half.
/// Note you should not just repeat the comments given with the function declaration, in the .h file or wherever.
/// It's okay to recapitulate briefly what the function does, but the focus of the comments should be on how it does it."

/*****************************************************************************
* Function: PID
*
* Description: Constructor
*****************************************************************************/
PID::PID
    (
        float kp,     // Constant for proportional term
        float ki,     // Constant for integral term
        float kd,     // Constant for derivative term
        float limit,  // Output limit cap for calculation
        float i_limit // Integral error summation cap limit
    ) :
    setpoint(0),
    kp(kp),
    ki(ki),
    kd(kd),
    error_sum(0),
    last_error(0),
    output_limit(limit),
    integral_limit(i_limit)
{
} // PID::PID() - Constructor

/*****************************************************************************
* Function: set_setpoint
*
* Description: Updates setpoint and clears integral summation term.
*****************************************************************************/
void PID::set_setpoint
    (
        float setpoint // New setpoint value
    )
{
    this->setpoint = setpoint;
    error_sum = 0;
                       /// <----- Always put a single blank line after last valid statement in function
} // PID::set_setpoint()

/// **Notice at the closing brace of every method/function there is the name
/// of the function followed by an empty set of parentheses.

/// Getters/Setters like the method above use a c-style naming conventions
/// (all lowercase with underscores) while all other methods use a camel-case
/// such as Calculate() or SomeMethodName().

/*****************************************************************************
* Function: Calculate
*
* Description: Finds derivative and integral components based on error trends
*              and delta t.  Then combines PID terms with gains to get an
*              output value which is returned.
*****************************************************************************/
float PID::Calculate
    (
        float value, // New value to perform calculation on
        float dt     // Seconds since last calculation
    )
{
    float error = setpoint - value;

    float derivative_error = 0.0f;

    // Check for division by zero
    if (dt != 0.0f)
    {
        derivative_error = (error - last_error) / dt;
    }

    error_sum += error * dt;

    // Cap integral term at limit
    error_sum = cap_bounds(error_sum, -integral_limit, integral_limit);

    float output = (kp * error) + (ki * error_sum) + (kd * derivative_error);

    // Cap output term at limit
    output = cap_bounds(output, -output_limit, output_limit);

    last_error = error;

    return output;
                          /// <----- Always put a single blank line after last valid statement in function
} // PID::Calculate()
                          /// <----- Always put a single blank line at end of file