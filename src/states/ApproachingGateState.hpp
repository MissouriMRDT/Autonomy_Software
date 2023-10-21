/******************************************************************************
 * @brief Approaching Gate State Implementation for Autonomy State Machine
 *
 * @file ApproachingGateState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"
#include "../AutonomyLogging.h"

/******************************************************************************
 * @brief Approaching Gate State Handler
 *
 *        Primarily the Approaching Gate State Handler, handles the navigation
 *        of the Rover in the final approach steps of traveling up to and
 *        through the gate.
 *
 *        It also listens for state events that pertain to the Approaching
 *        Gate State and calls the appropriate transition handler to transition
 *        states as needed.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct ApproachingGateState : sc::simple_state<ApproachingGateState, StateMachine>
{
        ApproachingGateState() { LOG_INFO(logging::g_qSharedLogger, "In State: Approaching Gate"); }

        typedef mpl::list<sc::custom_reaction<ApproachingGate_MarkerLostTransition>,
                          sc::custom_reaction<ApproachingGate_AbortTransition>,
                          sc::custom_reaction<ApproachingGate_ReachedMarkerTransition>>
            reactions;

        sc::result react(const ApproachingGate_MarkerLostTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<SearchPatternState>();
        }

        sc::result react(const ApproachingGate_AbortTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<AbortState>();
        }

        sc::result react(const ApproachingGate_ReachedMarkerTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<IdleState>();
        }
};

/******************************************************************************
 * @brief Approaching Gate State - Transition to Marker Lost
 *
 *        When the state machine reaches the 'Marker Lost' transition
 *        handler, Autonomy will transition into Search Pattern to
 *        attempt to find the marker again.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct ApproachingGate_MarkerLostTransition : sc::event<ApproachingGate_MarkerLostTransition>
{
        ApproachingGate_MarkerLostTransition() { LOG_INFO(logging::g_qSharedLogger, "In Transition: Approaching Gate (Marker Lost)"); }
};

/******************************************************************************
 * @brief Approaching Gate State - Transition to Abort
 *
 *        When the state machine reaches the 'Abort' transition handler,
 *        Autonomy will stop all processes and transition to the Abort State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct ApproachingGate_AbortTransition : sc::event<ApproachingGate_AbortTransition>
{
        ApproachingGate_AbortTransition() { LOG_INFO(logging::g_qSharedLogger, "In Transition: Approaching Gate (Abort)"); }
};

/******************************************************************************
 * @brief Approaching Gate State - Transition to Reached Marker
 *
 *        When the state machine reaches the 'Reached Marker' transition
 *        handler, Autonomy will send a packet to update the LED panel and
 *        will transition to Idle.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct ApproachingGate_ReachedMarkerTransition : sc::event<ApproachingGate_ReachedMarkerTransition>
{
        ApproachingGate_ReachedMarkerTransition() { LOG_INFO(logging::g_qSharedLogger, "In Transition: Approaching Gate (Reached Marker)"); }
};
