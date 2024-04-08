/******************************************************************************
 * @brief Approaching Marker State Implementation for Autonomy State Machine.
 *
 * @file ApproachingMarkerState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "ApproachingMarkerState.h"
#include "../AutonomyConstants.h"
#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief This method is called when the state is first started. It is used to
     *        initialize the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingMarkerState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Scheduling next run of state logic.");

        m_nNumDetectionAttempts = 0;
        m_nTargetTagID          = -1;
        m_bDetected             = false;
        m_dLastTargetHeading    = 0;
        m_dLastTargetDistance   = 0;

        m_vTagDetectors         = {globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam),
                                   globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eFrameLeftCam),
                                   globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eFrameRightCam)};
    }    // namespace statemachine

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingMarkerState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Exiting state.");
    }

    /******************************************************************************
     * @brief Accessor for the State private member. Returns the state as a string.
     *
     * @return std::string - The current state as a string.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    ApproachingMarkerState::ApproachingMarkerState() : State(States::eApproachingMarker)
    {
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

        m_bInitialized = false;

        if (!m_bInitialized)
        {
            Start();
            m_bInitialized = true;
        }
    }

    /******************************************************************************
     * @brief Run the state machine. Returns the next state.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingMarkerState::Run()
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "ApproachingMarkerState: Running state-specific behavior.");

        // Create instance variables.
        bool bDetectedTagAR = false;    // Was the tag detected through OpenCV.
        bool bDetectedTagTF = false;    // Was the tag detected through Tensorflow.

        // Get the current rover pose.
        geoops::RoverPose stCurrentRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();

        // If a target hasn't been identified yet attempt to find a target tag in the rover's vision.
        if (!m_bDetected && m_nNumDetectionAttempts < constants::APPROACH_MARKER_DETECT_ATTEMPTS_LIMIT)
        {
            // Attempt to identify the target with OpenCV.
            // While OpenCV struggles to find tags, the tags it does find are much more reliable compared to TensorFlow.
            bDetectedTagAR = IdentifyTargetArucoMarker(m_stTargetTagAR);
            if (bDetectedTagAR)
            {
                // Save the identified tag's ID.
                m_nTargetTagID          = m_stTargetTagAR.nID;
                m_bDetected             = true;
                m_nNumDetectionAttempts = 0;
            }
            else
            {
                // If OpenCV fails to find the tag rely on the TensorFlow vision algorithms to identify it.
                bDetectedTagTF = IdentifyTargetTensorflowMarker(m_stTargetTagTF);
                if (bDetectedTagTF)
                {
                    // Save the identified tag's ID.
                    // LEAD: Commented this out since TensorflowTag no longer has ID.
                    // m_nTargetTagID          = m_stTargetTagTF.nID;
                    m_bDetected             = true;
                    m_nNumDetectionAttempts = 0;
                }
            }

            // Both OpenCV & TensorFlow failed to identify a target tag.
            if (!m_bDetected)
            {
                ++m_nNumDetectionAttempts;
            }

            return;
        }
        // A target hasn't been identified and the amount of attempts has exceeded the limit.
        else if (!m_bDetected)
        {
            // Abort approaching marker.
            globals::g_pStateMachineHandler->HandleEvent(Event::eAbort);
            return;
        }

        // Attempt to find the target marker in OpenCV.
        bDetectedTagAR = tagdetectutils::FindArucoTagByID(m_nTargetTagID, m_stTargetTagAR, m_vTagDetectors);
        // LEAD: Commented this out since TensorflowTag no longer has ID.
        // if (!bDetectedTagAR)
        // {
        //     // Attempt to find the target marker in TensorFlow.
        //     bDetectedTagTF = tagdetectutils::FindTensorflowTagByID(m_nTargetTagID, m_stTargetTagTF, m_vTagDetectors);
        // }

        // The target marker wasn't found.
        if (!bDetectedTagAR && !bDetectedTagTF)
        {
            ++m_nNumDetectionAttempts;
        }
        // The target marker was found.
        else
        {
            m_nNumDetectionAttempts = 0;
        }

        // If we have made too many consecutive failed detection attempts
        // inform the statemachine the marker has been lost.
        if (m_nNumDetectionAttempts >= constants::APPROACH_MARKER_DETECT_ATTEMPTS_LIMIT)
        {
            globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerUnseen);
            return;
        }

        // Get the current absolute heading of the rover.
        double dCurrHeading = stCurrentRoverPose.GetCompassHeading();

        // Find the target's heading and distance with respect to the rover's current position.
        double dTargetHeading;
        double dTargetDistance;

        if (bDetectedTagAR)
        {
            dTargetHeading  = numops::InputAngleModulus<double>(dCurrHeading + m_stTargetTagAR.dYawAngle, 0, 360);
            dTargetDistance = m_stTargetTagAR.dStraightLineDistance;
        }
        else if (bDetectedTagTF)
        {
            dTargetHeading  = numops::InputAngleModulus<double>(dCurrHeading + m_stTargetTagTF.dYawAngle, 0, 360);
            dTargetDistance = m_stTargetTagTF.dStraightLineDistance;
        }
        // Use the last recorded heading and distance.
        else
        {
            dTargetHeading  = m_dLastTargetHeading;
            dTargetDistance = m_dLastTargetDistance;
        }
        // Save the found heading and distance.
        m_dLastTargetHeading  = dTargetHeading;
        m_dLastTargetDistance = dTargetDistance;

        // If we are close enough to the target inform the state machine we have reached the marker.
        if (dTargetDistance < constants::APPROACH_MARKER_PROXIMITY_THRESHOLD)
        {
            globals::g_pStateMachineHandler->HandleEvent(Event::eReachedMarker);
            return;
        }

        // Move the rover to the target's estimated position.
        diffdrive::DrivePowers stDrivePowers = globals::g_pDriveBoard->CalculateMove(constants::APPROACH_MARKER_MOTOR_POWER,
                                                                                     dTargetHeading,
                                                                                     dCurrHeading,
                                                                                     diffdrive::DifferentialControlMethod::eArcadeDrive);
        globals::g_pDriveBoard->SendDrive(stDrivePowers);

        return;
    }

    /******************************************************************************
     * @brief Trigger an event in the state machine. Returns the next state.
     *
     * @param eEvent - The event to trigger.
     * @return std::shared_ptr<State> - The next state.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    States ApproachingMarkerState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eIdle;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eReachedMarker:
            {
                LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Handling ReachedMarker event.");
                eNextState = States::eIdle;
                break;
            }
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eMarkerUnseen:
            {
                LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Handling MarkerUnseen event.");
                eNextState = States::eSearchPattern;
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "ApproachingMarkerState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eIdle)
        {
            LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }

    /******************************************************************************
     * @brief Identify a target marker in the rover's vision, using OpenCV detection.
     *
     * @note If multiple markers are detected the closest one will be chosen as the target.
     *
     * @param tTarget - Reference to store the tag identified as the target.
     * @return true - A target marker was identified.
     * @return false - A target marker was not identified.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-29
     ******************************************************************************/
    bool ApproachingMarkerState::IdentifyTargetArucoMarker(arucotag::ArucoTag& stTarget)
    {
        // Load all detected tags in the rover's vision.
        std::vector<arucotag::ArucoTag> vDetectedTags;
        tagdetectutils::LoadDetectedArucoTags(vDetectedTags, m_vTagDetectors, true);

        arucotag::ArucoTag stBestTag;
        stBestTag.dStraightLineDistance = std::numeric_limits<double>::max();
        stBestTag.nID                   = -1;

        // Select the tag that is the closest to the rover's current position.
        for (const arucotag::ArucoTag& stCandidate : vDetectedTags)
        {
            if (stCandidate.dStraightLineDistance < stBestTag.dStraightLineDistance)
            {
                stBestTag = stCandidate;
            }
        }

        // A tag was found.
        if (stBestTag.nID >= 0)
        {
            // Save it to the passed in reference.
            stTarget = stBestTag;
            return true;
        }
        // No target tag was found.
        else
        {
            return false;
        }
    }

    /******************************************************************************
     * @brief Identify a target marker in the rover's vision, using Tensorflow detection.
     *
     * @note If multiple markers are detected the closest one will be chosen as the target. Also all tags must
     *      have a confidence of at least APPROACH_MARKER_TF_CONFIDENCE_THRESHOLD.
     *
     * @param tTarget - Reference to store the tag identified as the target.
     * @return true - A target marker was identified.
     * @return false - A target marker was not identified.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-29
     ******************************************************************************/
    bool ApproachingMarkerState::IdentifyTargetTensorflowMarker(tensorflowtag::TensorflowTag& stTarget)
    {
        // Load all detected tags in the rover's vision.
        std::vector<tensorflowtag::TensorflowTag> vDetectedTags;
        tagdetectutils::LoadDetectedTensorflowTags(vDetectedTags, m_vTagDetectors);

        tensorflowtag::TensorflowTag stBestTag;
        stBestTag.dStraightLineDistance = std::numeric_limits<double>::max();
        bool bTagIdentified             = false;
        // stBestTag.nID                   = -1;

        // Select the tag that is the closest to the rover's current position and above the confidence threshold.
        for (const tensorflowtag::TensorflowTag& stCandidate : vDetectedTags)
        {
            if (stCandidate.dStraightLineDistance < stBestTag.dStraightLineDistance && stCandidate.dConfidence >= constants::APPROACH_MARKER_TF_CONFIDENCE_THRESHOLD)
            {
                stBestTag = stCandidate;

                // Update tag found toggle.
                bTagIdentified = true;
            }
        }

        // Check if a tag has been identified.
        if (bTagIdentified)
        {
            // Save it to the passed in reference.
            stTarget = stBestTag;
        }

        return bTagIdentified;

        // LEAD: Since TensorflowTag no longer has ID, commented out.
        // // A tag was found.
        // if (stBestTag.nID >= 0)
        // {
        //     // Save it to the passed in reference.
        //     stTarget = stBestTag;
        //     return true;
        // }
        // // No target tag was found.
        // else
        // {
        //     return false;
        // }
    }
}    // namespace statemachine
