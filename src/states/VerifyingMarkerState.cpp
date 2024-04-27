/******************************************************************************
 * @brief Verifying Marker State Implementation for Autonomy State Machine.
 *
 * @file VerifyingMarkerState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "VerifyingMarkerState.h"
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
    void VerifyingMarkerState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Scheduling next run of state logic.");

        m_nMaxMarkerIDs = 50;

        m_vMarkerIDs.reserve(m_nMaxMarkerIDs);

        m_tmVerificationStart = std::chrono::system_clock::now();
        bVerification         = true;

        m_vTagDetectors       = {globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam),
                                 globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eFrameLeftCam),
                                 globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eFrameRightCam)};
        m_tmLastDetectedTag   = std::chrono::system_clock::now();
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void VerifyingMarkerState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Exiting state.");

        m_vMarkerIDs.clear();
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    VerifyingMarkerState::VerifyingMarkerState() : State(States::eVerifyingMarker)
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
    void VerifyingMarkerState::Run()
    {
        // TODO: Implement the behavior specific to the VerifyingMarker state
        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingMarkerState: Running state-specific behavior.");

        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
        if (bVerification)
        {
            double dTimeSinceVerificationStart = std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmVerificationStart).count();
            if (dTimeSinceVerificationStart > constants::VERIFY_MARKER_VERIFY_TIMESPAN)
            {
                bVerification  = false;
                m_tmLightStart = tmCurrentTime;
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
            }
        }
        else
        {
            double dTimeSinceLightStart = std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmLightStart).count();
            if (dTimeSinceLightStart > constants::VERIFY_MARKER_LIGHT_TIMESPAN)
            {
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eOff);
                globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingComplete);
                return;
            }
        }

        bool bDetectedTagAR, bDetectedTagTF;

        bDetectedTagAR = IdentifyTargetArucoMarker(m_stTargetTagAR);
        if (!bDetectedTagAR)
        {
            bDetectedTagTF = IdentifyTargetTensorflowMarker(m_stTargetTagTF);
        }

        if (bDetectedTagAR || bDetectedTagTF)
        {
            m_tmLastDetectedTag = std::chrono::system_clock::now();
        }

        double dTimeSinceLastDetectedTag = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - m_tmLastDetectedTag).count();
        if (dTimeSinceLastDetectedTag > constants::VERIFY_MARKER_DETECTION_TIMESPAN)
        {
            globals::g_pStateMachineHandler->HandleEvent(Event::eAbort);
            return;
        }
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
    States VerifyingMarkerState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eVerifyingMarker;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Handling Start event.");
                // Send multimedia command to update state display.
                // globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eVerifyingComplete:
            {
                LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Handling Verifying Complete event.");
                eNextState = States::eIdle;
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Handling Abort event.");
                // Send multimedia command to update state display.
                // globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eSearchPattern;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "VerifyingMarkerState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eVerifyingMarker)
        {
            LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Transitioning to {} State.", StateToString(eNextState));

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
    bool VerifyingMarkerState::IdentifyTargetArucoMarker(arucotag::ArucoTag& stTarget)
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
    bool VerifyingMarkerState::IdentifyTargetTensorflowMarker(tensorflowtag::TensorflowTag& stTarget)
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
            if (stCandidate.dStraightLineDistance < stBestTag.dStraightLineDistance && stCandidate.dConfidence >= constants::VERIFY_MARKER_TF_CONFIDENCE_THRESHOLD)
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
