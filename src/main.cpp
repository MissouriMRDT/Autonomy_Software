/******************************************************************************
 * @brief Main program file. Sets up classes and runs main program functions.
 *
 * @file main.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-0620
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

// #include "../examples/OpenCV/TagGenerator.hpp"

#include "./Autonomy_Globals.h"
#include "./interfaces/StateMachine.hpp"

/******************************************************************************
 * @brief Autonomy main function.
 *
 * @return int - Exit status number.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-0620
 ******************************************************************************/
int main()
{
    // This leaks memory when the generateImageMarker function is called lol. Damn OpenCV, never knew it was so bad.
    // GenerateOpenCVArucoMarker(cv::aruco::DICT_4X4_50, 1);

    InitializeAutonomyLoggers();

    StateMachine states_machine;
    states_machine.initiate();
    states_machine.process_event(Idle_NavigatingTransition());

    return 0;
}
