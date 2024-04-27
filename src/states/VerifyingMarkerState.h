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

#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"
#include "../util/vision/TagDetectionUtilty.hpp"
#include "../vision/aruco/ArucoDetection.hpp"
#include "../vision/aruco/TagDetector.h"
#include "../vision/aruco/TensorflowTagDetection.hpp"

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
            std::vector<int> m_vMarkerIDs;
            int m_nMaxMarkerIDs;
            bool m_bInitialized;

            arucotag::ArucoTag m_stTargetTagAR;                             // Detected target tag from OpenCV.
            tensorflowtag::TensorflowTag m_stTargetTagTF;                   // Detected target tag from Tensorflow.
            std::vector<TagDetector*> m_vTagDetectors;                      // Vector of tag detectors to use for detection in order of highest to lowest priority.

            std::chrono::system_clock::time_point m_tmLastDetectedTag;      // When verification began.
            std::chrono::system_clock::time_point m_tmVerificationStart;    // When verification began.
            std::chrono::system_clock::time_point m_tmLighStart;            // When lights began.

            bool bVerification;

            bool IdentifyTargetArucoMarker(arucotag::ArucoTag& stTarget);
            bool IdentifyTargetTensorflowMarker(tensorflowtag::TensorflowTag& stTarget);

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
