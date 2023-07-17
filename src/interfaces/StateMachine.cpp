/******************************************************************************
 * @brief
 *
 * @file StateMachine.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-0716
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "./StateMachine.h"

struct StateMachine : sc::state_machine<StateMachine, IdleState>
{
        StateMachine() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "Starting State Machine...\n"; }
};