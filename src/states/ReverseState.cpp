/******************************************************************************
 * @brief
 *
 * @file ReverseState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-0716
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "./StateMachine.hpp"

struct ReverseState : sc::simple_state<ReverseState, StateMachine>
{
        ReverseState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Reverse\n"; }

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
        Reverse_ContinueTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Reverse (Continue)\n"; }
};

struct Reverse_AbortTransition : sc::event<Reverse_AbortTransition>
{
        Reverse_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Reverse (Abort)\n"; }
};

struct Reverse_StuckTransition : sc::event<Reverse_StuckTransition>
{
        Reverse_StuckTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Reverse (Stuck)\n"; }
};
