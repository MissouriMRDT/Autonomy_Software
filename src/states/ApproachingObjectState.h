/******************************************************************************
 * @brief Approaching Object State Implementation for Autonomy State Machine.
 *
 * @file ApproachingObjectState.h
 * @author Eli Byrd (edbgkk@mst.edu), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef APPROACHING_OBJECT_STATE_H
#define APPROACHING_OBJECT_STATE_H

#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"
#include "../util/logging/PathTracer.hpp"
#include "../util/states/StuckDetection.hpp"
#include "../util/vision/ObjectDetectionUtility.hpp"
#include "../vision/objects/ObjectDetector.h"

#include <chrono>

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief The ApproachingObjectState class implements the Approaching Object
     * state for the Autonomy State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    class ApproachingObjectState : public State
    {
        private:
            // Core Components
            std::vector<std::shared_ptr<ObjectDetector>> m_vObjectDetectors;
            statemachine::TimeIntervalBasedStuckDetector m_StuckDetector;
            std::unique_ptr<logging::graphing::PathTracer> m_pRoverPathPlot;

            // State tracking
            States m_eTriggeringState;
            bool m_bInitialized;
            geoops::Waypoint m_stGoalWaypoint;

            // Persistent tracking variables for Run()
            double m_dHeadingSetPoint;
            double m_dDistanceFromObject;
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
            ApproachingObjectState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // APPROACHING_OBJECT_STATE_H
