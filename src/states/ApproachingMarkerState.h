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
            bool m_bInitialized;

            States m_eTriggeringState;                                    // The state that the rover was in before triggering the MarkerSeen event.
            int m_nNumDetectionAttempts;                                  // Number of consecutive unsuccessful attempts to detect a tag.
            int m_nTargetTagID;                                           // ID of the target tag.
            bool m_bDetected;                                             // Has a target tag been detected and identified yet.
            arucotag::ArucoTag m_stTargetTagAruco;                        // Detected target tag from OpenCV.
            torchtag::TorchTag m_stTargetTagTorch;                        // Detected target tag from Torch.
            tensorflowtag::TensorflowTag m_stTargetTagTensorflow;         // Detected target tag from Tensorflow.
            double m_dLastTargetHeading;                                  // Last recorded heading of the target with respect to the rover's position.
            double m_dLastTargetDistance;                                 // Last recorded distance of the target with respect to the rover's position.
            std::vector<std::shared_ptr<TagDetector>> m_vTagDetectors;    // Vector of tag detectors to use for detection in order of highest to lowest priority.
            statemachine::TimeIntervalBasedStuckDetector m_StuckDetector;

            bool IdentifyTargetMarker(arucotag::ArucoTag& stArucoTarget, torchtag::TorchTag& stTorchTarget, tensorflowtag::TensorflowTag& stTensorflowTarget);

            void Start() override;
            void Exit() override;

        public:
            ApproachingMarkerState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // APPROACHINGMARKERSTATE_H
