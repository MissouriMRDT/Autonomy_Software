/******************************************************************************
 * @brief
 *
 * @file TakeoffState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-0716
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "./StateMachine.hpp"

struct TakeoffState : sc::simple_state<TakeoffState, StateMachine>
{
        TakeoffState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Takeoff\n"; }

        typedef mpl::list<sc::custom_reaction<Takeoff_NoResponseTransition>,
                          sc::custom_reaction<Takeoff_ClimbTransition>,
                          sc::custom_reaction<Takeoff_AbortTransition>,
                          sc::custom_reaction<Takeoff_ReachedAltitudeTransition>>
            reactions;

        sc::result react(const Takeoff_NoResponseTransition& event) { return transit<CrashedState>(); }

        sc::result react(const Takeoff_ClimbTransition& event) { return transit<DecendState>(); }

        sc::result react(const Takeoff_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Takeoff_ReachedAltitudeTransition& event) { return transit<NavigationState>(); }
};

struct Takeoff_NoResponseTransition : sc::event<Takeoff_NoResponseTransition>
{
        Takeoff_NoResponseTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Takeoff (No Response)\n"; }
};

struct Takeoff_ClimbTransition : sc::event<Takeoff_ClimbTransition>
{
        Takeoff_ClimbTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Takeoff (Climb)\n"; }
};

struct Takeoff_AbortTransition : sc::event<Takeoff_AbortTransition>
{
        Takeoff_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Takeoff (Abort)\n"; }
};

struct Takeoff_ReachedAltitudeTransition : sc::event<Takeoff_ReachedAltitudeTransition>
{
        Takeoff_ReachedAltitudeTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Takeoff (Reached Altitude)\n"; }
};
