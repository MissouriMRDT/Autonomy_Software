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
            std::vector<double> m_vRoverXPath;
            std::vector<double> m_vRoverYPath;
            std::vector<double> m_vRoverYawPath;
            std::vector<double> m_vRoverXPosition;
            std::vector<double> m_vRoverYPosition;
            std::vector<double> m_vRoverYawPosition;
            std::vector<double> m_vRoverVelocity;
            double m_nLastIDX;
            double m_nTargetIDX;
            double m_nMaxDataPoints;
            time_t m_tPathStartTime;
            time_t m_tStuckCheckTime;

            pathplanners::AStar m_stPlanner;
            controllers::StanleyController m_stController =
                controllers::StanleyController(constants::STANLEY_STEER_CONTROL_GAIN, constants::STANLEY_DIST_TO_FRONT_AXLE, constants::STANLEY_YAW_TOLERANCE);
            geoops::UTMCoordinate m_stStart;
            geoops::UTMCoordinate m_stGoal;
            geoops::Waypoint m_stGoalWaypoint;
            std::vector<geoops::UTMCoordinate> m_vPlannedRoute;

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
