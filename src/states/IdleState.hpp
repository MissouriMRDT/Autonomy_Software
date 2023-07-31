/******************************************************************************
 * @brief Idle State Implementation for Autonomy State Machine
 *
 * @file IdleState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Idle State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct IdleState : sc::simple_state<IdleState, StateMachine>
{
        IdleState() { LOG_INFO(g_qSharedLogger, "In State: Idle"); }

        typedef mpl::list<sc::custom_reaction<Idle_AbortTransition>, sc::custom_reaction<Idle_NavigatingTransition>, sc::custom_reaction<Idle_ReverseTransition>>
            reactions;

        sc::result react(const Idle_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Idle_NavigatingTransition& event) { return transit<NavigationState>(); }

        sc::result react(const Idle_ReverseTransition& event) { return transit<ReverseState>(); }
};

struct Idle_AbortTransition : sc::event<Idle_AbortTransition>
{
        Idle_AbortTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Idle (Abort)"); }
};

struct Idle_NavigatingTransition : sc::event<Idle_NavigatingTransition>
{
        Idle_NavigatingTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Idle (Navigating)"); }
};

struct Idle_ReverseTransition : sc::event<Idle_NavigatingTransition>
{
        Idle_ReverseTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Idle (Reverse)"); }
};
