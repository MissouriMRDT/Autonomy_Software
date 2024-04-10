/******************************************************************************
 * @brief Approaching Marker State Implementation for Autonomy State Machine.
 *
 * @file ApproachingMarkerState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef APPROACHINGMARKERSTATE_H
#define APPROACHINGMARKERSTATE_H

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

            int m_nNumDetectionAttempts;                     // Number of consecutive unsuccessful attempts to detect a tag.
            int m_nTargetTagID;                              // ID of the target tag.
            bool m_bDetected;                                // Has a target tag been detected and identified yet.
            arucotag::ArucoTag m_stTargetTagAR;              // Detected target tag from OpenCV.
            tensorflowtag::TensorflowTag m_stTargetTagTF;    // Detected target tag from Tensorflow.
            double m_dLastTargetHeading;                     // Last recorded heading of the target with respect to the rover's position.
            double m_dLastTargetDistance;                    // Last recorded distance of the target with respect to the rover's position.
            std::vector<TagDetector*> m_vTagDetectors;       // Vector of tag detectors to use for detection in order of highest to lowest priority.

            bool IdentifyTargetArucoMarker(arucotag::ArucoTag& stTarget);
            bool IdentifyTargetTensorflowMarker(tensorflowtag::TensorflowTag& stTarget);

        protected:
            void Start() override;
            void Exit() override;

        public:
            ApproachingMarkerState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // APPROACHINGMARKERSTATE_H
