/******************************************************************************
 * @brief Avoidance State Implementation for Autonomy State Machine.
 *
 * @file AvoidanceState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef AVOIDANCE_STATE_H
#define AVOIDANCE_STATE_H

#include "../algorithms/controllers/StanleyController.h"
#include "../algorithms/planners/AStar.h"
#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"
#include "../util/states/StuckDetection.hpp"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief The AvoidanceState class implements the Avoidance state for the Autonomy
     *        State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class AvoidanceState : public State
    {
        private:
            statemachine::TimeIntervalBasedStuckDetector m_stStuckChecker;
            pathplanners::AStar m_stPlanner;
            controllers::StanleyController m_stController;
            geoops::RoverPose m_stPose;
            geoops::UTMCoordinate m_stStart;
            geoops::UTMCoordinate m_stGoal;
            geoops::Waypoint m_stGoalWaypoint;
            std::vector<geoops::UTMCoordinate> m_vPlannedRoute;
            States m_eTriggeringState;
            bool m_bInitialized;

        protected:
            void Start() override;
            void Exit() override;

        public:
            AvoidanceState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // AVOIDANCESTATE_H
