/******************************************************************************
 * @brief Approaching Marker State Implementation for Autonomy State Machine.
 *
 * @file ApproachingMarkerState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef APPROACHING_MARKER_STATE_H
#define APPROACHING_MARKER_STATE_H

#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"
#include "../util/logging/PathTracer.hpp"
#include "../util/states/StuckDetection.hpp"
#include "../util/vision/TagDetectionUtilty.hpp"
#include "../vision/aruco/TagDetector.h"

/// \cond
#include <chrono>

/// \endcond

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief The ApproachingMarkerState class implements the Approaching Marker
     * state for the Autonomy State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class ApproachingMarkerState : public State
    {
        private:
            // Core Components
            std::vector<std::shared_ptr<TagDetector>> m_vTagDetectors;
            statemachine::TimeIntervalBasedStuckDetector m_StuckDetector;
            std::unique_ptr<logging::graphing::PathTracer> m_pRoverPathPlot;

            // State tracking
            States m_eTriggeringState;
            bool m_bInitialized;
            geoops::Waypoint m_stGoalWaypoint;

            // Persistent tracking variables for Run()
            double m_dHeadingSetPoint;
            double m_dDistanceFromTag;
            bool m_bDriveBackwards;

            // Geolocation fallback
            geoops::Waypoint m_stLastGeolocatedPosition;
            bool m_bHasLastGeolocatedPosition;
            bool m_bHasSeenTarget;

            // Timing and logging flags
            std::chrono::system_clock::time_point m_tmLastSeenTime;
            std::chrono::system_clock::time_point m_tmLastLogTime;
            bool m_bAlreadyPrintedLost;
            bool m_bAlreadyPrintedVisualLostFallback;

        protected:
            void Start() override;
            void Exit() override;

        public:
            ApproachingMarkerState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // APPROACHING_MARKER_STATE_H
