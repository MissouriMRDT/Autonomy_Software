/******************************************************************************
 * @brief Navigating State Implementation for Autonomy State Machine.
 *
 * @file NavigatingState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef NAVIGATING_STATE_H
#define NAVIGATING_STATE_H

#include "../algorithms/controllers/PredictiveStanleyController.h"
#include "../algorithms/planners/GeoPlanner.h"
#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"
#include "../util/states/StuckDetection.hpp"
#include "../vision/aruco/TagDetector.h"
#include "../vision/objects/ObjectDetector.h"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief The NavigatingState class implements the Navigating state for the Autonomy
     *        State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class NavigatingState : public State
    {
        private:
            /////////////////////////////////////////
            // Declare private member structs.
            /////////////////////////////////////////
            struct VirtualObstacle
            {
                    geoops::Waypoint stWaypoint;
                    std::chrono::system_clock::time_point tmTimeDetected;
            };

            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////
            bool m_bWasStuck;
            double m_dStuckDistanceToGoal;
            double m_dHeadingBeforeStuck;
            bool m_bFetchNewWaypoint;
            geoops::Waypoint m_stGoalWaypoint;
            bool m_bInitialized;
            std::vector<std::shared_ptr<TagDetector>> m_vTagDetectors;
            std::vector<std::shared_ptr<ObjectDetector>> m_vObjectDetectors;
            statemachine::TimeIntervalBasedStuckDetector m_StuckDetector;
            std::unique_ptr<controllers::PredictiveStanleyController> m_pStanleyController;
            std::vector<geoops::Waypoint> m_vPathCoordinates;
            std::vector<VirtualObstacle> m_vActiveVirtualObstacles;

            /////////////////////////////////////////
            // Declare private class methods.
            /////////////////////////////////////////
            geoops::UTMCoordinate ModifyPathAfterStuckState();

        protected:
            /////////////////////////////////////////
            // Declare protected class methods.
            /////////////////////////////////////////
            void Start() override;
            void Exit() override;

        public:
            /////////////////////////////////////////
            // Declare public class methods.
            /////////////////////////////////////////
            NavigatingState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // NAVIGATINGSTATE_H
