/******************************************************************************
 * @brief Verifying Object State Implementation for Autonomy State Machine.
 *
 * @file VerifyingObjectState.cpp
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "VerifyingObjectState.h"
#include "../AutonomyGlobals.h"
#include "../AutonomyNetworking.h"
#include "../util/TimeOperations.hpp"
#include "../util/states/ObjectDetectionChecker.hpp"
#include <filesystem>
#include <opencv2/opencv.hpp>

// #include "../util/states/ObjectDetectionChecker.hpp"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief This method is called when the state is first started. It is used to
     *        initialize the state.
     *
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void VerifyingObjectState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Scheduling next run of state logic.");

        // Initialize member variables.
        m_stGoalWaypoint                = globals::g_pWaypointHandler->PeekNextWaypoint();
        m_tmObjectVerificationStartTime = std::chrono::system_clock::now();
        m_tmObjectLastSeenTime          = std::chrono::system_clock::now();

        // Get object detectors.
        m_vObjectDetectors = {globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eHeadMainCam)};
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void VerifyingObjectState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Exiting state.");
    }

    /******************************************************************************
     * @brief Accessor for the State private member. Returns the state as a string.
     *
     * @return std::string - The current state as a string.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    VerifyingObjectState::VerifyingObjectState() : State(States::eVerifyingObject)
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
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void VerifyingObjectState::Run()
    {
        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingObjectState: Running state-specific behavior.");

        // Identify target object.
        objectdetectutils::Object stBestObject;
        statemachine::IdentifyTargetObject(m_vObjectDetectors, stBestObject, m_stGoalWaypoint.eType);
        // Calculate how long we've been in this state.
        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
        double dElapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmObjectVerificationStartTime).count() / 1000.0;
        // Calculate the time since the last time we saw an object.
        double dTimeSinceLastSeen = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmObjectLastSeenTime).count() / 1000.0;

        /*
            If we consistently detect an object for a certain amount of time, we can assume that we are in fact in front of the object.
            At this point, we can also assume we are close enough for the pointcloud to be usable and pick up the object.
        */
        // Check if object is detected.
        if (stBestObject.dConfidence == 0.0)
        {
            // Check if the time last seen is greater than the time to give up.
            if (dTimeSinceLastSeen > constants::APPROACH_OBJECT_LOST_BUFFER_TIME)
            {
                // No objects are detected, trigger verify failed event.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: No objects detected. Triggering verify failed event.");
                globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingFailed);
                return;
            }
        }
        else
        {
            // Check the object distance.
            if (stBestObject.dConfidence > 0.0 && stBestObject.dStraightLineDistance > constants::APPROACH_OBJECT_PROXIMITY_THRESHOLD)
            {
                // Object is too far away, trigger verify failed event.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Object detected but too far away. Triggering verify failed event.");
                globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingFailed);
                return;
            }

            // Update time last seen.
            m_tmObjectLastSeenTime = std::chrono::system_clock::now();

            // Check if we have been in this state long enough to verify the object.
            if (dElapsedTime >= constants::APPROACH_OBJECT_VERIFY_TIME)
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Object verified. Triggering verify complete event.");
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
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    States VerifyingObjectState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eVerifyingObject;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eVerifyingComplete:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Handling Verifying Complete event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);

                // Request the snapshot from the object detection handler
                cv::Mat cvSnapshot = globals::g_pObjectDetectionHandler->RequestDetectionOverlayFrame();

                if (!cvSnapshot.empty())
                {
                    std::string szLogDir = logging::g_szLoggingOutputPath + "/detections/";
                    if (!std::filesystem::exists(szLogDir))
                    {
                        std::filesystem::create_directories(szLogDir);
                    }

                    // Create a unique filename using the current timestamp
                    std::string szTimestamp = timeops::GetTimestamp();
                    std::string szFilename  = szLogDir + "object_" + szTimestamp + ".png";

                    // Save the image to the disk
                    bool bSuccess = cv::imwrite(szFilename, cvSnapshot);

                    if (bSuccess)
                    {
                        LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Saved detection snapshot to {}", szFilename);
                    }
                    else
                    {
                        LOG_ERROR(logging::g_qSharedLogger, "VerifyingObjectState: Failed to write snapshot to disk.");
                    }
                }
                else
                {
                    LOG_WARNING(logging::g_qSharedLogger, "VerifyingObjectState: Overlay frame was empty. No snapshot taken.");
                }

                // Pop old waypoint out of queue.
                globals::g_pWaypointHandler->PopNextWaypoint();
                // Clear saved states.
                globals::g_pStateMachineHandler->ClearSavedStates();
                // Submit logger message.
                LOG_NOTICE(logging::g_qSharedLogger, "VerifyingObjectState: Cleared old saved states.");
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            case Event::eVerifyingFailed:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Handling Verifying Failed event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Recall the previous state.
                eNextState = globals::g_pStateMachineHandler->GetPreviousState();
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eOff);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "VerifyingObjectState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eVerifyingObject)
        {
            LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
