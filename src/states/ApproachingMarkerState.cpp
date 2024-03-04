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
        LOG_DEBUG(logging::g_qSharedLogger, "ApproachingMarkerState: Scheduling next run of state logic.");

        m_nNumDetectionAttempts = 0;
        m_nTargetTagID          = -1;
        m_bDetected             = false;
        m_dLastTargetHeading    = 0;
        m_dLastTargetDistance   = 0;
    }

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
        LOG_DEBUG(logging::g_qSharedLogger, "ApproachingMarkerState: Exiting state.");
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
    States ApproachingMarkerState::Run()
    {
        LOG_DEBUG(logging::g_qSharedLogger, "ApproachingMarkerState: Running state-specific behavior.");

        bool bDetectedTagAR;    // Was the tag detected through OpenCV.
        bool bDetectedTagTF;    // Was the tag detected through Tensorflow.

        // If a target hasn't been identified yet attempt to find a target tag in the rover's vision.
        if (!m_bDetected && m_nNumDetectionAttempts < DETECT_ATTEMPTS_LIMIT)
        {
            // Attempt to identify the target with OpenCV.
            // While OpenCV struggles to find tags, the tags it does find are much more reliable compared to TensorFlow.
            bDetectedTagAR = IdentifyTargetMarker(m_stTargetTagAR);
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
                bDetectedTagTF = IdentifyTargetMarker(m_stTargetTagTF);
                if (bDetectedTagTF)
                {
                    // Save the identified tag's ID.
                    m_nTargetTagID          = m_stTargetTagTF.nID;
                    m_bDetected             = true;
                    m_nNumDetectionAttempts = 0;
                }
            }

            // Both OpenCV & TensorFlow failed to identify a target tag.
            if (!m_bDetected)
            {
                ++m_nNumDetectionAttempts;
            }

            return States::eApproachingMarker;
        }
        // A target hasn't been identified and the amount of attempts has exceeded the limit.
        else if (!m_bDetected)
        {
            // Abort approaching marker.
            globals::g_pStateMachineHandler->HandleEvent(Event::eAbort);
            return States::eApproachingMarker;
        }

        // Attempt to find the target marker in OpenCV.
        bDetectedTagAR = FindTargetMarker(m_nTargetTagID, m_stTargetTagAR);
        if (!bDetectedTagAR)
        {
            // Attempt to find the target marker in TensorFlow.
            bDetectedTagTF = FindTargetMarker(m_nTargetTagID, m_stTargetTagAR);
        }

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
        if (m_nNumDetectionAttempts >= DETECT_ATTEMPTS_LIMIT)
        {
            globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerUnseen);
            return States::eApproachingMarker;
        }

        // Get the current absolute heading of the rover.
        double dCurrHeading = globals::g_pNavigationBoard->GetIMUData().dHeading;

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
        if (dTargetDistance < CLOSE_ENOUGH)
        {
            globals::g_pStateMachineHandler->HandleEvent(Event::eReachedMarker);
            return States::eApproachingMarker;
        }

        // Move the rover to the target's estimated position.
        diffdrive::DrivePowers stDrivePowers = globals::g_pDriveBoard->CalculateMove(MOTOR_SPEED_ON_APPROACH, dTargetHeading, dCurrHeading, diffdrive::eTankDrive);
        globals::g_pDriveBoard->SendDrive(stDrivePowers.dLeftDrivePower, stDrivePowers.dRightDrivePower);

        return States::eApproachingMarker;
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
        States eNextState       = States::eIdle;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eReachedMarker:
            {
                LOG_DEBUG(logging::g_qSharedLogger, "ApproachingMarkerState: Handling ReachedMarker event.");
                eNextState = States::eIdle;
                break;
            }
            case Event::eStart:
            {
                LOG_DEBUG(logging::g_qSharedLogger, "ApproachingMarkerState: Handling Start event.");
                eNextState = States::eApproachingMarker;
                break;
            }
            case Event::eMarkerUnseen:
            {
                LOG_DEBUG(logging::g_qSharedLogger, "ApproachingMarkerState: Handling MarkerUnseen event.");
                eNextState = States::eSearchPattern;
                break;
            }
            case Event::eAbort:
            {
                LOG_DEBUG(logging::g_qSharedLogger, "ApproachingMarkerState: Handling Abort event.");
                eNextState = States::eIdle;
                break;
            }
            default:
            {
                LOG_DEBUG(logging::g_qSharedLogger, "ApproachingMarkerState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eIdle)
        {
            LOG_DEBUG(logging::g_qSharedLogger, "ApproachingMarkerState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }

    /******************************************************************************
     * @brief Find the target marker in the rover's vision.
     *
     * @tparam T -T ype of tag/detection to use. (ArucoTag - OpenCV) and (TensorflowTag - Tensorflow).
     * @param nID - ID of the target marker to be detected.
     * @param tIdentifiedTag - Reference to store the tag identified as the target.
     * @return true - The target marker was found in the rover's vision.
     * @return false - The target marker was not found in the rover's vision.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-29
     ******************************************************************************/
    template<typename T>
    bool ApproachingMarkerState::FindTargetMarker(int nID, T& tIdentifiedTag)
    {
        // Load all detected tags in the rover's vision.
        std::vector<T> vDetectedTags;
        LoadDetectedTags(vDetectedTags);

        // Find the tag with the corresponding ID.
        typename std::vector<T>::const_iterator itDetectedTag = vDetectedTags.begin();
        while (itDetectedTag != vDetectedTags.end())
        {
            // Tag is the target.
            if (itDetectedTag->nID == nID)
            {
                // Save the tag to the passed in reference.
                tIdentifiedTag = *itDetectedTag;
                return true;
            }
        }

        // Target tag was not found.
        return false;
    }

    /******************************************************************************
     * @brief Identify a target marker in the rover's vision.
     *
     * @note If multiple markers are detected the closest one will be chosen as the target.
     *
     * @tparam T - Type of tag/detection to use. (ArucoTag - OpenCV) and (TensorflowTag - Tensorflow).
     * @param tTarget - Reference to store the tag identified as the target.
     * @return true - A target marker was identified.
     * @return false - A target marker was not identified.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-29
     ******************************************************************************/
    template<typename T>
    bool ApproachingMarkerState::IdentifyTargetMarker(T& tTarget)
    {
        // Load all detected tags in the rover's vision.
        std::vector<T> vDetectedTags;
        LoadDetectedTags(vDetectedTags);

        T tBestTag;
        tBestTag.dStraightLineDistance = std::numeric_limits<double>::max();
        tBestTag.nID                   = -1;

        // Select the tag that is the closest to the rover's current position.
        typename std::vector<T>::const_iterator itCandidate;
        while (itCandidate != vDetectedTags.end())
        {
            if (itCandidate->dStraightLineDistance < tBestTag.dStraightLineDistance)
            {
                tBestTag = *itCandidate;
            }
            ++itCandidate;
        }

        // A tag was found.
        if (tBestTag.nID >= 0)
        {
            // Save it to the passed in reference.
            tTarget = tBestTag;
            return true;
        }
        // No target tag was found.
        else
        {
            return false;
        }
    }

    /******************************************************************************
     * @brief Load all detected tags for a given tag type into the passed in vector.
     *
     *
     * @tparam T - Type of tag/detection to use. (ArucoTag - OpenCV) and (TensorflowTag - Tensorflow).
     * @param vDetectedTags - Vector to store the detected tags.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-29
     ******************************************************************************/
    template<typename T>
    void ApproachingMarkerState::LoadDetectedTags(std::vector<T> vDetectedTags)
    {
        // Pointers to each tag detector.
        TagDetector* pTagDetectorMainCam  = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam);
        TagDetector* pTagDetectorLeftEye  = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadLeftArucoEye);
        TagDetector* pTagDetectorRightEye = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadRightArucoEye);

        // Vectors to store detected tags for each detector.
        std::vector<T> vDetectedTagsMain;
        std::vector<T> vDetectedTagsLeft;
        std::vector<T> vDetectedTagsRight;

        // Futures informing us when detected tags have been loaded.
        std::future<bool> fuDetectedTagsMain;
        std::future<bool> fuDetectedTagsLeft;
        std::future<bool> fuDetectedTagsRight;

        // Dealing with OpenCV detection using arucotag::ArucoTag.
        if constexpr (std::is_same_v<T, arucotag::ArucoTag>)
        {
            fuDetectedTagsMain  = pTagDetectorMainCam->RequestDetectedArucoTags(vDetectedTagsMain);
            fuDetectedTagsLeft  = pTagDetectorLeftEye->RequestDetectedArucoTags(vDetectedTagsLeft);
            fuDetectedTagsRight = pTagDetectorRightEye->RequestDetectedArucoTags(vDetectedTagsRight);
        }
        // Dealing with Tensorflow detection using tensorflowtag::TensorFlowTag.
        else if constexpr (std::is_same_v<T, tensorflowtag::TensorflowTag>)
        {
            fuDetectedTagsMain  = pTagDetectorMainCam->RequestDetectedTensorflowTags(vDetectedTagsMain);
            fuDetectedTagsLeft  = pTagDetectorLeftEye->RequestDetectedTensorflowTags(vDetectedTagsLeft);
            fuDetectedTagsRight = pTagDetectorRightEye->RequestDetectedTensorflowTags(vDetectedTagsRight);
        }
        // Unknown tag type.
        else
        {
            LOG_ERROR(logging::g_qSharedLogger, "Unknown tag type passed to LoadDetectedTags(), must be ArucoTag or TensorflowTag.");
            return;
        }

        // Save the main detector's detected tags.
        fuDetectedTagsMain.wait();
        vDetectedTags.insert(vDetectedTags.end(), vDetectedTagsMain.begin(), vDetectedTagsMain.end());

        // Save the left detector's detected tags.
        fuDetectedTagsLeft.wait();
        vDetectedTags.insert(vDetectedTags.end(), vDetectedTagsLeft.begin(), vDetectedTagsLeft.end());

        // Save the right detector's detected tags.
        fuDetectedTagsRight.wait();
        vDetectedTags.insert(vDetectedTags.end(), vDetectedTagsRight.begin(), vDetectedTagsRight.end());
    }
}    // namespace statemachine
