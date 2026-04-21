/******************************************************************************
 * @brief Verifying Marker State Implementation for Autonomy State Machine.
 *
 * @file VerifyingMarkerState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef VERIFYING_MARKER_STATE_H
#define VERIFYING_MARKER_STATE_H

#include "../handlers/TagDetectionHandler.h"
#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"
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
     * @brief The VerifyingMarkerState class implements the Verifying Marker state for
     *        the Autonomy State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class VerifyingMarkerState : public State
    {
        private:
            bool m_bInitialized;
            geoops::Waypoint m_stGoalWaypoint;
            tagdetectutils::ArucoTag m_stBestArucoTag;
            tagdetectutils::ArucoTag m_stBestTorchTag;
            std::vector<std::shared_ptr<TagDetector>> m_vTagDetectors;
            std::chrono::system_clock::time_point m_tmTagVerificationStartTime;
            std::chrono::system_clock::time_point m_tmTagLastSeenTime;

        protected:
            void Start() override;
            void Exit() override;

        public:
            VerifyingMarkerState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // VERIFYINGMARKERSTATE_H
