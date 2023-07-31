/******************************************************************************
 * @brief Search Pattern State Implementation for Autonomy State Machine
 *
 * @file SearchPatternState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Search Pattern State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct SearchPatternState : sc::simple_state<SearchPatternState, StateMachine>
{
        SearchPatternState() { LOG_INFO(g_qSharedLogger, "In State: Search Pattern"); }

        typedef mpl::list<sc::custom_reaction<SeachPattern_GateSeenTransition>,
                          sc::custom_reaction<SeachPattern_ObstacleAvoidanceTransition>,
                          sc::custom_reaction<SeachPattern_MarkerSeenTransition>,
                          sc::custom_reaction<SeachPattern_AbortTransition>,
                          sc::custom_reaction<SeachPattern_StuckTransition>>
            reactions;

        sc::result react(const SeachPattern_GateSeenTransition& event) { return transit<ApproachingGateState>(); }

        sc::result react(const SeachPattern_ObstacleAvoidanceTransition& event) { return transit<AvoidanceState>(); }

        sc::result react(const SeachPattern_MarkerSeenTransition& event) { return transit<ApproachingMarkerState>(); }

        sc::result react(const SeachPattern_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const SeachPattern_StuckTransition& event) { return transit<StuckState>(); }
};

/******************************************************************************
 * @brief Gate Seen Transition
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct SeachPattern_GateSeenTransition : sc::event<SeachPattern_GateSeenTransition>
{
        SeachPattern_GateSeenTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Search Pattern (Gate Seen)"); }
};

/******************************************************************************
 * @brief Obstacle Avoidance Transition
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct SeachPattern_ObstacleAvoidanceTransition : sc::event<SeachPattern_ObstacleAvoidanceTransition>
{
        SeachPattern_ObstacleAvoidanceTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Search Pattern (Obstacle Avoidance)"); }
};

/******************************************************************************
 * @brief Marker Seen Transition
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct SeachPattern_MarkerSeenTransition : sc::event<SeachPattern_MarkerSeenTransition>
{
        SeachPattern_MarkerSeenTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Search Pattern (Marker Seen)"); }
};

/******************************************************************************
 * @brief Abort Transition
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct SeachPattern_AbortTransition : sc::event<SeachPattern_AbortTransition>
{
        SeachPattern_AbortTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Search Pattern (Abort)"); }
};

/******************************************************************************
 * @brief Stuck Transition
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct SeachPattern_StuckTransition : sc::event<SeachPattern_StuckTransition>
{
        SeachPattern_StuckTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Search Pattern (Stuck)"); }
};
