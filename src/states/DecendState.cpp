/******************************************************************************
 * @brief
 *
 * @file DecendState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-0716
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "./StateMachine.hpp"

struct DecendState : sc::simple_state<DecendState, StateMachine>
{
        DecendState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Decend\n"; }

        typedef mpl::
            list<sc::custom_reaction<Decend_NoResponseTransition>, sc::custom_reaction<Decend_AbortTransition>, sc::custom_reaction<Decend_ReachedAltitudeTransition>>
                reactions;

        sc::result react(const Decend_NoResponseTransition& event) { return transit<CrashedState>(); }

        sc::result react(const Decend_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Decend_ReachedAltitudeTransition& event)
        {
            // If - Takeoff
            //      return transit<TakeoffState>();

            // If - Navigate
            //      return transit<NavigateState>();

            // If - Search Pattern
            //      return transit<SearchPatternState>();

            // If - Avoidance
            //      return transit<AvoidanceState>();

            return transit<IdleState>();
        }
};

struct Decend_NoResponseTransition : sc::event<Decend_NoResponseTransition>
{
        Decend_NoResponseTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Decend (No Response)\n"; }
};

struct Decend_AbortTransition : sc::event<Decend_AbortTransition>
{
        Decend_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Decend (Abort)\n"; }
};

struct Decend_ReachedAltitudeTransition : sc::event<Decend_ReachedAltitudeTransition>
{
        Decend_ReachedAltitudeTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Decend (Reached Altitude)\n"; }
};
