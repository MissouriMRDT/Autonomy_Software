/******************************************************************************
 * @brief
 *
 * @file IdleState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../Autonomy_Globals.h"

/******************************************************************************
 * @brief Idle State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct IdleState : sc::simple_state<IdleState, StateMachine>
{
        IdleState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Idle"; }

        typedef mpl::list<sc::custom_reaction<Idle_AbortTransition>, sc::custom_reaction<Idle_NavigatingTransition>, sc::custom_reaction<Idle_ReverseTransition>>
            reactions;

        sc::result react(const Idle_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Idle_NavigatingTransition& event) { return transit<NavigationState>(); }

        sc::result react(const Idle_ReverseTransition& event) { return transit<ReverseState>(); }
};

struct Idle_AbortTransition : sc::event<Idle_AbortTransition>
{
        Idle_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Idle (Abort)"; }
};

struct Idle_NavigatingTransition : sc::event<Idle_NavigatingTransition>
{
        Idle_NavigatingTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Idle (Navigating)"; }
};

struct Idle_ReverseTransition : sc::event<Idle_NavigatingTransition>
{
        Idle_ReverseTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Idle (Reverse)"; }
};
