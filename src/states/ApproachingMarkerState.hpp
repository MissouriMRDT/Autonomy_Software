/******************************************************************************
 * @brief Approaching Marker State Implementation for Autonomy State Machine
 *
 * @file ApproachingMarkerState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Approaching Marker State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
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

struct ApproachingMarker_MarkerLostTransition : sc::event<ApproachingMarker_MarkerLostTransition>
{
        ApproachingMarker_MarkerLostTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Approaching Marker (Marker Lost)"); }
};

struct ApproachingMarker_AbortTransition : sc::event<ApproachingMarker_AbortTransition>
{
        ApproachingMarker_AbortTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Approaching Marker (Abort)"); }
};

struct ApproachingMarker_ReachedMarkerTransition : sc::event<ApproachingMarker_ReachedMarkerTransition>
{
        ApproachingMarker_ReachedMarkerTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Approaching Marker (Reached Marker)"); }
};
