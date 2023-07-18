/******************************************************************************
 * @brief
 *
 * @file ApproachingGateState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

/******************************************************************************
 * @brief Approaching Gate State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct ApproachingGateState : sc::simple_state<ApproachingGateState, StateMachine>
{
        ApproachingGateState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Approaching Gate"; }

        typedef mpl::list<sc::custom_reaction<ApproachingGate_MarkerLostTransition>,
                          sc::custom_reaction<ApproachingGate_AbortTransition>,
                          sc::custom_reaction<ApproachingGate_ReachedMarkerTransition>>
            reactions;

        sc::result react(const ApproachingGate_MarkerLostTransition& event) { return transit<SearchPatternState>(); }

        sc::result react(const ApproachingGate_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const ApproachingGate_ReachedMarkerTransition& event) { return transit<IdleState>(); }
};

struct ApproachingGate_MarkerLostTransition : sc::event<ApproachingGate_MarkerLostTransition>
{
        ApproachingGate_MarkerLostTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Approaching Gate (Marker Lost)"; }
};

struct ApproachingGate_AbortTransition : sc::event<ApproachingGate_AbortTransition>
{
        ApproachingGate_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Approaching Gate (Abort)"; }
};

struct ApproachingGate_ReachedMarkerTransition : sc::event<ApproachingGate_ReachedMarkerTransition>
{
        ApproachingGate_ReachedMarkerTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Approaching Gate (Reached Marker)"; }
};
