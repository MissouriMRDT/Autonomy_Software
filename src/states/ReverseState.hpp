/******************************************************************************
 * @brief Reverse State Implementation for Autonomy State Machine
 *
 * @file ReverseState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Reverse State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct ReverseState : sc::simple_state<ReverseState, StateMachine>
{
        ReverseState() { LOG_INFO(g_qSharedLogger, "In State: Reverse"); }

        typedef mpl::list<sc::custom_reaction<Reverse_ContinueTransition>, sc::custom_reaction<Reverse_AbortTransition>, sc::custom_reaction<Reverse_StuckTransition>>
            reactions;

        sc::result react(const Reverse_ContinueTransition& event)
        {
            // If - Idle
            //      return transit<IdleState>();

            // If - Navigate
            //      return transit<NavigateState>();

            // If - Search Pattern
            //      return transit<SearchPatternState>();

            // If - Avoidance
            //      return transit<AvoidanceState>();

            // If - Approaching Marker
            //      return transit<ApproachingMarkerState>();

            // If - Approaching Gate
            //      return transit<ApproachingGateState>();

            return transit<IdleState>();
        }

        sc::result react(const Reverse_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Reverse_StuckTransition& event) { return transit<StuckState>(); }
};

struct Reverse_ContinueTransition : sc::event<Reverse_ContinueTransition>
{
        Reverse_ContinueTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Reverse (Continue)"); }
};

struct Reverse_AbortTransition : sc::event<Reverse_AbortTransition>
{
        Reverse_AbortTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Reverse (Abort)"); }
};

struct Reverse_StuckTransition : sc::event<Reverse_StuckTransition>
{
        Reverse_StuckTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Reverse (Stuck)"); }
};
