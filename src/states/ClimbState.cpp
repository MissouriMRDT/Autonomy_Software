/******************************************************************************
 * @brief
 *
 * @file ClimbState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-0716
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "./StateMachine.hpp"

struct ClimbState : sc::simple_state<ClimbState, StateMachine>
{
        ClimbState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Climb\n"; }

        typedef mpl::
            list<sc::custom_reaction<Climb_NoResponseTransition>, sc::custom_reaction<Climb_AbortTransition>, sc::custom_reaction<Climb_ReachedAltitudeTransition>>
                reactions;

        sc::result react(const Climb_NoResponseTransition& event) { return transit<CrashedState>(); }

        sc::result react(const Climb_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Climb_ReachedAltitudeTransition& event)
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

struct Climb_NoResponseTransition : sc::event<Climb_NoResponseTransition>
{
        Climb_NoResponseTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Climb (No Response)\n"; }
};

struct Climb_AbortTransition : sc::event<Climb_AbortTransition>
{
        Climb_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Climb (Abort)\n"; }
};

struct Climb_ReachedAltitudeTransition : sc::event<Climb_ReachedAltitudeTransition>
{
        Climb_ReachedAltitudeTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Climb (Reached Altitude)\n"; }
};
