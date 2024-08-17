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

#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"
#include "../util/states/StuckDetection.hpp"
#include "../vision/aruco/TagDetector.h"

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
            // Declare private member variables.
            /////////////////////////////////////////
            bool m_bFetchNewWaypoint;
            geoops::Waypoint m_stGoalWaypoint;
            int m_nMaxDataPoints;
            std::vector<double> m_vRoverXPosition;
            std::vector<double> m_vRoverYPosition;
            time_t m_tStuckCheckTime;
            double m_dStuckCheckLastPosition[2];
            bool m_bInitialized;
            std::vector<TagDetector*> m_vTagDetectors;
            statemachine::TimeIntervalBasedStuckDetector m_StuckDetector;

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
