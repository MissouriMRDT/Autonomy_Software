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
#include "../AutonomyNetworking.h"

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

        // Initialize member variables.
        m_nTargetTagID   = -1;
        m_stGoalWaypoint = globals::g_pWaypointHandler->PeekNextWaypoint();

        // Store the state that got stuck and triggered a MarkerSeen event.
        m_eTriggeringState = globals::g_pStateMachineHandler->GetPreviousState();

        // Get tag detectors.
        m_vTagDetectors = {globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam),
                           globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eFrameLeftCam),
                           globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eFrameRightCam)};
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

        m_bInitialized  = false;

        m_StuckDetector = statemachine::TimeIntervalBasedStuckDetector(constants::STUCK_CHECK_ATTEMPTS,
                                                                       constants::STUCK_CHECK_INTERVAL,
                                                                       constants::STUCK_CHECK_VEL_THRESH,
                                                                       constants::STUCK_CHECK_ROT_THRESH);
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

        // Get the current rover pose.
        geoops::RoverPose stCurrentRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();

        // Check Rover radius from marker waypoint.
        geoops::GeoMeasurement stCurrentMeasurement = geoops::CalculateGeoMeasurement(m_stGoalWaypoint.GetGPSCoordinate(), stCurrentRoverPose.GetGPSCoordinate());
        if (stCurrentMeasurement.dDistanceMeters > m_stGoalWaypoint.dRadius)
        {
            globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerUnseen);
            return;
        }

        // Identify target marker.
        tagdetectutils::ArucoTag stBestArucoTag, stBestTorchTag;
        this->IdentifyTargetMarker(stBestArucoTag, stBestTorchTag);

        // Check if both tag types are unseen.
        if (stBestArucoTag.nID == -1 && stBestTorchTag.nID == -1)
        {
            globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerUnseen);
            return;
        }

        double dHeadingSetPoint = 0.0;
        double dDistanceFromTag = 9999;

        if (stBestArucoTag.nID != -1)
        {
            cv::Size cvCameraResolution = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam)->GetProcessFrameResolution();
            tagdetectutils::EstimatePoseFromCameraFrame(cvCameraResolution, constants::ZED_MAINCAM_HORIZONTAL_FOV, stBestArucoTag);
            dHeadingSetPoint = stBestArucoTag.dYawAngle + stCurrentRoverPose.GetCompassHeading();
            dDistanceFromTag = stBestArucoTag.dStraightLineDistance;
        }

        else if (stBestTorchTag.nID != -1)
        {
            cv::Size cvCameraResolution = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam)->GetProcessFrameResolution();
            tagdetectutils::EstimatePoseFromCameraFrame(cvCameraResolution, constants::ZED_MAINCAM_HORIZONTAL_FOV, stBestTorchTag);
            dHeadingSetPoint = stBestTorchTag.dYawAngle + stCurrentRoverPose.GetCompassHeading();
            dDistanceFromTag = stBestTorchTag.dStraightLineDistance;
        }

        // FIXME: CLAYTON WAS HERE. Remove all this to make parsing through errors during the consolidation of tag structs easier.
        // LEAD: Rewrite after refactor is finished.

        // Move the rover to the target's estimated position.
        diffdrive::DrivePowers stDrivePowers = globals::g_pDriveBoard->CalculateMove(constants::APPROACH_MARKER_MOTOR_POWER,
                                                                                     dHeadingSetPoint,
                                                                                     stCurrentRoverPose.GetCompassHeading(),
                                                                                     diffdrive::DifferentialControlMethod::eArcadeDrive);
        globals::g_pDriveBoard->SendDrive(stDrivePowers);

        // Check if tag is reached.
        if (dDistanceFromTag < constants::APPROACH_MARKER_PROXIMITY_THRESHOLD)
        {
            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Rover has reached the target marker!");
            // Handle state transition and save the current search pattern state.
            globals::g_pStateMachineHandler->HandleEvent(Event::eReachedMarker);
            // Don't execute the rest of the state.
            return;
        }

        //////////////////////////////////////////
        /* ---  Check if the rover is stuck --- */
        //////////////////////////////////////////

        // Check if stuck.
        if (m_StuckDetector.CheckIfStuck(globals::g_pWaypointHandler->SmartRetrieveVelocity(), globals::g_pWaypointHandler->SmartRetrieveAngularVelocity()))
        {
            // Submit logger message.
            LOG_NOTICE(logging::g_qSharedLogger, "NavigatingState: Rover has become stuck!");
            // Handle state transition and save the current search pattern state.
            globals::g_pStateMachineHandler->HandleEvent(Event::eStuck, true);
            // Don't execute the rest of the state.
            return;
        }

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
        States eNextState       = States::eApproachingMarker;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eReachedMarker:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Handling ReachedMarker event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
                // Pop old waypoint out of queue.
                globals::g_pWaypointHandler->PopNextWaypoint();
                // Clear saved search pattern state.
                globals::g_pStateMachineHandler->ClearSavedState(States::eSearchPattern);
                // Submit logger message.
                LOG_NOTICE(logging::g_qSharedLogger, "ApproachingMarkerState: Cleared old search pattern state from saved states.");
                // Change states.
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
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Handling MarkerUnseen event.");
                // Change states.
                eNextState = m_eTriggeringState;
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

        if (eNextState != States::eApproachingMarker)
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
     * @param stArucoTarget - The detected target marker from OpenCV.
     * @param stTorchTarget - The detected target marker from Torch.

     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-29
     ******************************************************************************/
    void ApproachingMarkerState::IdentifyTargetMarker(tagdetectutils::ArucoTag& stArucoTarget, tagdetectutils::ArucoTag& stTorchTarget)
    {
        // Create instance variables.
        std::vector<tagdetectutils::ArucoTag> vDetectedArucoTags;
        tagdetectutils::ArucoTag stArucoBestTag;
        tagdetectutils::ArucoTag stTorchBestTag;
        std::string szIdentifiedTags = "";

        // Get the current time
        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();

        // Load all detected tags in the rover's vision.
        this->LoadDetectedTags(vDetectedArucoTags, m_vTagDetectors);
        // Find the best tag from the Aruco tags.
        for (const tagdetectutils::ArucoTag& stCandidate : vDetectedArucoTags)
        {
            // Calculate the total age of the tag.
            double dTagTotalAge = std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - stCandidate.tmCreation).count();
            // Calculate the total tag area.
            double dArea = stCandidate.pBoundingBox->area();

            // Check the tag detection method type.
            if (stCandidate.eDetectionMethod != tagdetectutils::TagDetectionMethod::eOpenCV)
            {
                // Assemble the identified tags string.
                szIdentifiedTags += "\tArUco ID: " + std::to_string(stCandidate.nID) + " Tag Age: " + std::to_string(dTagTotalAge) + "s\n";
                // Check if the tag is best.
                if (stCandidate.nID == m_nTargetTagID || m_nTargetTagID == -1)
                {
                    // Check other tag requirements.
                    if (dArea > stArucoBestTag.pBoundingBox->area() && dTagTotalAge < constants::ARUCO_MIN_LIFETIME_THRESHOLD)
                    {
                        // Set the target tag to the detected tag.
                        stArucoBestTag = stCandidate;
                    }
                }
            }
            else if (stCandidate.eDetectionMethod == tagdetectutils::TagDetectionMethod::eTorch)
            {
                // Assemble the identified tags string.
                szIdentifiedTags += "\tTorch Class: " + stCandidate.szClassName + " Tag Age: " + std::to_string(dTagTotalAge) + "s\n";
                // Check if the tag is best.
                if (dArea > stTorchBestTag.pBoundingBox->area() && dTagTotalAge < constants::ARUCO_MIN_LIFETIME_THRESHOLD)
                {
                    // Set the target tag to the detected tag.
                    stTorchBestTag = stCandidate;
                }
            }
        }

        // Set the target tag to the best tag.
        stArucoTarget = stArucoBestTag;
        stTorchTarget = stTorchBestTag;
    }

    /******************************************************************************
     * @brief Aggregates all detected tags from each provided tag detector for both OpenCV and Tensorflow detection.
     *
     * @param vDetectedArucoTags - Reference vector that will hold all of the aggregated detected Aruco tags.
     * @param vTagDetectors - Vector of pointers to tag detectors that will be used to request their detected tags.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-03-07
     ******************************************************************************/
    void ApproachingMarkerState::LoadDetectedTags(std::vector<tagdetectutils::ArucoTag>& vDetectedArucoTags,
                                                  const std::vector<std::shared_ptr<TagDetector>>& vTagDetectors)
    {
        // Number of tag detectors.
        size_t siNumTagDetectors = vTagDetectors.size();

        // Initialize vectors to store detected tags temporarily.
        std::vector<std::vector<tagdetectutils::ArucoTag>> vDetectedArucoTagBuffers(siNumTagDetectors);

        // Initialize vectors to store detected tags futures.
        std::vector<std::future<bool>> vDetectedArucoTagsFuture;

        // Request tags from each detector.
        for (size_t siIdx = 0; siIdx < siNumTagDetectors; ++siIdx)
        {
            // Check if this tag detector is ready.
            if (vTagDetectors[siIdx]->GetIsReady())
            {
                // Request detected Aruco tags from detector.
                vDetectedArucoTagsFuture.emplace_back(vTagDetectors[siIdx]->RequestDetectedArucoTags(vDetectedArucoTagBuffers[siIdx]));
            }
        }

        // Ensure all requests have been fulfilled.
        // Then transfer tags from the buffer to vDetectedArucoTags and vDetectedTensorflowTags for the user to access.
        for (size_t siIdx = 0; siIdx < vDetectedArucoTagsFuture.size(); ++siIdx)
        {
            // Wait for the request to be fulfilled.
            vDetectedArucoTagsFuture[siIdx].get();

            // Loop through the detected Aruco tags and add them to the vDetectedArucoTags vector.
            for (const tagdetectutils::ArucoTag& tTag : vDetectedArucoTagBuffers[siIdx])
            {
                vDetectedArucoTags.emplace_back(tTag);
            }
        }
    }
}    // namespace statemachine
