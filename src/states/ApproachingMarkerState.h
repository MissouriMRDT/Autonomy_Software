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
#include "../util/states/StuckDetection.hpp"
#include "../util/vision/TagDetectionUtilty.hpp"
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
     * @brief The ApproachingMarkerState class implements the Approaching Marker
     *        state for the Autonomy State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class ApproachingMarkerState : public State
    {
        private:
            std::vector<std::shared_ptr<TagDetector>> m_vTagDetectors;
            statemachine::TimeIntervalBasedStuckDetector m_StuckDetector;
            States m_eTriggeringState;
            bool m_bInitialized;
            int m_nTargetTagID;
            geoops::Waypoint m_stGoalWaypoint;

            void IdentifyTargetMarker(tagdetectutils::ArucoTag& stArucoTarget, tagdetectutils::ArucoTag& stTorchTarget);
            void LoadDetectedTags(std::vector<tagdetectutils::ArucoTag>& vDetectedArucoTags, const std::vector<std::shared_ptr<TagDetector>>& vTagDetectors);

            void Start() override;
            void Exit() override;

        public:
            ApproachingMarkerState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // APPROACHINGMARKERSTATE_H
