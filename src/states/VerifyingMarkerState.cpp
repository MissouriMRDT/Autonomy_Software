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
#include "../AutonomyNetworking.h"
#include "../util/TimeOperations.hpp"
#include "../util/states/TagDetectionChecker.hpp"
#include <filesystem>
#include <opencv2/opencv.hpp>

/// \cond
#include <tracy/Tracy.hpp>

/// \endcond

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
     * @author Eli Byrd (edbgkk@mst.edu), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void VerifyingMarkerState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Scheduling next run of state logic.");

        // Initialize member variables.
        m_stGoalWaypoint             = globals::g_pWaypointHandler->PeekNextWaypoint();
        m_tmTagVerificationStartTime = std::chrono::system_clock::now();
        m_tmTagLastSeenTime          = std::chrono::system_clock::now();

        // Get tag detectors.
        m_vTagDetectors = {globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam),
                           globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eRearCam)};
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
        ZoneScopedC(tracy::Color::Green);
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingMarkerState: Running state-specific behavior.");

        // Identify target marker.
        tagdetectutils::ArucoTag stBestArucoTag, stBestTorchTag;
        statemachine::IdentifyTargetMarker(m_vTagDetectors, stBestArucoTag, stBestTorchTag, m_stGoalWaypoint.nID);

        // Calculate how long we've been in this state.
        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
        double dElapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmTagVerificationStartTime).count() / 1000.0;
        // Calculate the time since the last time we saw a tag.
        double dTimeSinceLastSeen = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmTagLastSeenTime).count() / 1000.0;

        /*
            If we consistently detect a marker for a certain amount of time, we can assume that we are in fact in front of the marker.
            At this point, we can also assume we are close enough for the pointcloud to be usable and for aruco to pick up the tag.
        */
        // Check if ArUco tag is detected.
        if (stBestArucoTag.nID == -1 && stBestTorchTag.dConfidence == 0.0)
        {
            // Check if the time last seen is greater than the time to give up.
            if (dTimeSinceLastSeen > constants::APPROACH_MARKER_TAG_LOST_BUFFER_TIME)
            {
                // No tags are detected, trigger verify failed event.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: No tags detected. Triggering verify failed event.");
                globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingFailed);
                return;
            }
        }
        else
        {
            // Check the tags distance.
            if (stBestArucoTag.nID != -1 && stBestArucoTag.dStraightLineDistance > constants::APPROACH_MARKER_PROXIMITY_THRESHOLD)
            {
                // Tag is too far away, trigger verify failed event.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: ArUco tag detected but too far away. Triggering verify failed event.");
                globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingFailed);
                return;
            }
            else if (stBestTorchTag.dConfidence > 0.0 && stBestTorchTag.dStraightLineDistance > constants::APPROACH_MARKER_PROXIMITY_THRESHOLD)
            {
                // Tag is too far away, trigger verify failed event.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Torch tag detected but too far away. Triggering verify failed event.");
                globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingFailed);
                return;
            }

            // Update time last seen.
            m_tmTagLastSeenTime = std::chrono::system_clock::now();

            // Update best tags.
            m_stBestArucoTag = stBestArucoTag;
            m_stBestTorchTag = stBestTorchTag;

            // Check if we have been in this state long enough to verify the marker.
            if (dElapsedTime >= constants::APPROACH_MARKER_VERIFY_TIME)
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Marker verified. Triggering verify complete event.");
                // Trigger verify complete event.
                globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingComplete);
                return;
            }
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
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eVerifyingComplete:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Handling Verifying Complete event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);

                // Loop through the detectors vector and find which ones UUID matches the winning tag's UUID.
                // If a match is found, request the snapshot from that detector and save it to disk with a unique filename.
                cv::Mat cvSnapshot;
                for (const std::shared_ptr<TagDetector>& pTagDetector : m_vTagDetectors)
                {
                    if (pTagDetector->GetThreadUUID() == m_stBestArucoTag.szDetectorUUID || pTagDetector->GetThreadUUID() == m_stBestTorchTag.szDetectorUUID)
                    {
                        // Load the detector's newest last-good overlay snapshot once into a local.
                        // The TagDetectionHandler holds a Subscription to this channel for the
                        // detector's lifetime, so it is being published. This read is lock free and
                        // never blocks on the detector's loop.
                        pubsub::Publisher<cv::Mat>::SharedSnapshot pSnapshot = pTagDetector->GetLastGoodOverlayPublisher().Get();
                        if (pSnapshot != nullptr)
                        {
                            // Deep copy the immutable snapshot so we own the frame we are about to save.
                            pSnapshot->tData.copyTo(cvSnapshot);
                        }
                        else
                        {
                            // Submit logger message.
                            LOG_WARNING(logging::g_qSharedLogger, "VerifyingMarkerState: No detection overlay frame has been published yet.");
                        }
                        break;
                    }
                }

                // Make sure the snapshot is not empty before trying to save it.
                if (!cvSnapshot.empty())
                {
                    // Ensure the directory exists
                    std::string szLogDir = logging::g_szLoggingOutputPath + "/detections/";
                    if (!std::filesystem::exists(szLogDir))
                    {
                        std::filesystem::create_directories(szLogDir);
                    }

                    // Create a unique filename using the current timestamp
                    std::string szTimestamp = timeops::GetTimestamp();
                    std::string szFilename  = szLogDir + "marker_" + szTimestamp + ".png";

                    // Save the image to the disk
                    bool bSuccess = cv::imwrite(szFilename, cvSnapshot);

                    if (bSuccess)
                    {
                        LOG_NOTICE(logging::g_qSharedLogger, "VerifyingMarkerState: Saved detection snapshot to {}", szFilename);
                    }
                    else
                    {
                        LOG_ERROR(logging::g_qSharedLogger, "VerifyingMarkerState: Failed to write snapshot to disk.");
                    }
                }
                else
                {
                    LOG_WARNING(logging::g_qSharedLogger, "VerifyingMarkerState: Overlay frame was empty. No snapshot taken.");
                }

                // Pop old waypoint out of queue.
                globals::g_pWaypointHandler->PopNextWaypoint();
                // Clear saved states.
                globals::g_pStateMachineHandler->ClearSavedStates();
                // Submit logger message.
                LOG_NOTICE(logging::g_qSharedLogger, "VerifyingMarkerState: Cleared old saved states.");
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            case Event::eVerifyingFailed:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Handling Verifying Failed event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Recall the previous state.
                eNextState = globals::g_pStateMachineHandler->GetPreviousState();
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingMarkerState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eOff);
                // Change state.
                eNextState = States::eIdle;
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
}    // namespace statemachine
