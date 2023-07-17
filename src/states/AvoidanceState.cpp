/******************************************************************************
 * @brief
 *
 * @file AvoidanceState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-0716
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "./StateMachine.hpp"

struct AvoidanceState : sc::simple_state<AvoidanceState, StateMachine>
{
        AvoidanceState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Avoidance\n"; }

        typedef mpl::list<sc::custom_reaction<Avoidance_EndAvoidanceTransition>,
                          sc::custom_reaction<Avoidance_AbortTransition>,
                          sc::custom_reaction<Avoidance_StuckTransition>,
                          sc::custom_reaction<Avoidance_AdjustAltitudeTransition>>
            reactions;

        sc::result react(const Avoidance_EndAvoidanceTransition& event)
        {
            // If - Search Pattern
            //      return transit<SearchPatternState>();

            // If - Navigation
            //      return transit<NavigationState>();

            return transit<IdleState>();
        }

        sc::result react(const Avoidance_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Avoidance_StuckTransition& event) { return transit<StuckState>(); }

        sc::result react(const Avoidance_AdjustAltitudeTransition& event)
        {
            // If - Climb
            //      return transit<ClimbState>();

            // If - Decend
            //      return transit<DecendState>();

            return transit<IdleState>();
        }
};

struct Avoidance_EndAvoidanceTransition : sc::event<Avoidance_EndAvoidanceTransition>
{
        Avoidance_EndAvoidanceTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Avoidance (End Avoidance)\n"; }
};

struct Avoidance_AbortTransition : sc::event<Avoidance_AbortTransition>
{
        Avoidance_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Avoidance (Abort)\n"; }
};

struct Avoidance_StuckTransition : sc::event<Avoidance_StuckTransition>
{
        Avoidance_StuckTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Avoidance (Stuck)\n"; }
};

struct Avoidance_AdjustAltitudeTransition : sc::event<Avoidance_AdjustAltitudeTransition>
{
        Avoidance_AdjustAltitudeTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Avoidance (Adjust Altitude)\n"; }
};
