/******************************************************************************
 * @brief
 *
 * @file CrashedState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-0716
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "./StateMachine.hpp"

struct CrashedState : sc::simple_state<CrashedState, StateMachine>
{
        CrashedState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Crashed\n"; }

        typedef mpl::list<sc::custom_reaction<Crashed_ExitTransition>, sc::custom_reaction<Crashed_AbortTransition>> reactions;

        sc::result react(const Crashed_ExitTransition& event) { exit(0); }

        sc::result react(const Crashed_AbortTransition& event) { return transit<AbortState>(); }
};

struct Crashed_ExitTransition : sc::event<Crashed_ExitTransition>
{
        Crashed_ExitTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Crashed (Exit)\n"; }
};

struct Crashed_AbortTransition : sc::event<Crashed_AbortTransition>
{
        Crashed_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Crashed (Abort)\n"; }
};
