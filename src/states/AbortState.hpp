/******************************************************************************
 * @brief
 *
 * @file AbortState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../Autonomy_Globals.h"

/******************************************************************************
 * @brief Abort State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct AbortState : sc::simple_state<AbortState, StateMachine>
{
        AbortState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Abort"; }

        typedef mpl::list<sc::custom_reaction<Abort_RestartTransition>, sc::custom_reaction<Abort_ExitTransition>> reactions;

        sc::result react(const Abort_RestartTransition& event) { return transit<IdleState>(); }

        sc::result react(const Abort_ExitTransition& event) { return transit<IdleState>(); }
};

struct Abort_RestartTransition : sc::event<Abort_RestartTransition>
{
        Abort_RestartTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Abort (Restart)"; }
};

struct Abort_ExitTransition : sc::event<Abort_ExitTransition>
{
        Abort_ExitTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Abort (Exit)"; }
};
