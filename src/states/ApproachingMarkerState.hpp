/******************************************************************************
 * @brief Approaching Marker State Implementation for Autonomy State Machine
 *
 * @file ApproachingMarkerState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Approaching Marker State Handler
 *
 *        Primarily the Approaching Marker State Handler, handles the
 *        navigation of the Rover in the final approach steps of traveling
 *        up to and stopping at a the end of a marker leg.
 *
 *        It also listens for state events that pertain to the Approaching
 *        Marker State and calls the approprate transition handler to transition
 *        states as needed.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct ApproachingMarkerState : sc::simple_state<ApproachingMarkerState, StateMachine>
{
        ApproachingMarkerState() { LOG_INFO(g_qSharedLogger, "In State: Approaching Marker"); }

        typedef mpl::list<sc::custom_reaction<ApproachingMarker_MarkerLostTransition>,
                          sc::custom_reaction<ApproachingMarker_AbortTransition>,
                          sc::custom_reaction<ApproachingMarker_ReachedMarkerTransition>>
            reactions;

        sc::result react(const ApproachingMarker_MarkerLostTransition& event) { return transit<SearchPatternState>(); }

        sc::result react(const ApproachingMarker_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const ApproachingMarker_ReachedMarkerTransition& event) { return transit<IdleState>(); }
};

/******************************************************************************
 * @brief Approaching Marker State - Transition to Marker Lost
 *
 *        When the state machine reaches the 'Marker Lost' transition
 *        handler, Autonomy will transition into Search Pattern to
 *        attempt to find the marker again.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct ApproachingMarker_MarkerLostTransition : sc::event<ApproachingMarker_MarkerLostTransition>
{
        ApproachingMarker_MarkerLostTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Approaching Marker (Marker Lost)"); }
};

/******************************************************************************
 * @brief Approaching Marker State - Transition to Abort
 *
 *        When the state machine reaches the 'Abort' transition handler,
 *        Autonomy will stop all processes and transition to the Abort State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct ApproachingMarker_AbortTransition : sc::event<ApproachingMarker_AbortTransition>
{
        ApproachingMarker_AbortTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Approaching Marker (Abort)"); }
};

/******************************************************************************
 * @brief Approaching Marker State - Transition to Reached Marker
 *
 *        When the state machine reaches the 'Reached Marker' transition
 *        handler, Autonomy will send a packet to update the LED panel and
 *        will transition to Idle.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct ApproachingMarker_ReachedMarkerTransition : sc::event<ApproachingMarker_ReachedMarkerTransition>
{
        ApproachingMarker_ReachedMarkerTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Approaching Marker (Reached Marker)"); }
};
