/******************************************************************************
 * @brief This file is a utility used to check if any file from the example
 *      directory has been included. When the CHECK_IF_EXAMPLE_INCLUDED
 *      macro is ran, the RunExampleFlag will be true or false depending on if an
 *      example file has been #included.
 *
 * @file ExampleChecker.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-28
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef EXAMPLE_CHECK_H
#define EXAMPLE_CHECK_H

// Define the macro to check if the example directory is included.
#define CHECK_IF_EXAMPLE_INCLUDED           \
    static bool bRunExampleFlag = false;    \
    namespace                               \
    {                                       \
        struct RunExampleInitializer        \
        {                                   \
                RunExampleInitializer()     \
                {                           \
                    bRunExampleFlag = true; \
                }                           \
        } initializer;                      \
    }

#endif    // EXAMPLE_CHECK_H
