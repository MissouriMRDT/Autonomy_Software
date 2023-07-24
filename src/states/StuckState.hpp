/******************************************************************************
 * @brief
 *
 * @file StuckState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Stuck State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct StuckState : sc::simple_state<StuckState, StateMachine>
{
        StuckState() { LOG_INFO(g_qSharedLogger, "In State: Stuck"); }

        typedef mpl::list<sc::custom_reaction<Stuck_AbortTransition>, sc::custom_reaction<Stuck_ReverseTransition>> reactions;

        sc::result react(const Stuck_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Stuck_ReverseTransition& event) { return transit<ReverseState>(); }
};

struct Stuck_AbortTransition : sc::event<Stuck_AbortTransition>
{
        Stuck_AbortTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Stuck (Abort)"); }
};

struct Stuck_ReverseTransition : sc::event<Stuck_ReverseTransition>
{
        Stuck_ReverseTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Stuck (Reverse)"); }
};
