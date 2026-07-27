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

/// \cond
#include <tracy/Tracy.hpp>

/// \endcond

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
     * initialize the state.
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

        // Initialize time-based hit tracking
        m_tmLastRunTime   = std::chrono::system_clock::now();
        m_dTotalValidTime = 0.0;

        // Get object detectors.
        m_vObjectDetectors = {globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eHeadMainCam),
                              globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eRearCam)};
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     * the state.
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
        ZoneScopedC(tracy::Color::SpringGreen);
        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingObjectState: Running state-specific behavior.");

        // IMPORTANT: Ensure the rover is completely stopped to avoid motion blur during verification.
        globals::g_pDriveBoard->SendStop();

        // 1. Calculate delta time since the last execution of Run()
        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
        double dDeltaTime                                   = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmLastRunTime).count() / 1000.0;
        m_tmLastRunTime                                     = tmCurrentTime;    // Reset for the next loop

        // 2. Identify target object.
        objectdetectutils::Object stBestObject;
        statemachine::IdentifyTargetObject(m_vObjectDetectors, stBestObject, m_stGoalWaypoint.eType);

        // 3. Update Time-Based Hit Rate
        if (stBestObject.dConfidence > 0.0)
        {
            // Add the time that elapsed during this valid frame to our total valid time
            m_dTotalValidTime += dDeltaTime;

            m_stBestObject         = stBestObject;    // Save the best object for the snapshot later
            m_tmObjectLastSeenTime = tmCurrentTime;
        }

        double dElapsedTime       = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmObjectVerificationStartTime).count() / 1000.0;
        double dTimeSinceLastSeen = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmObjectLastSeenTime).count() / 1000.0;

        // 4. Fast-Fail if object is completely lost
        if (dTimeSinceLastSeen > constants::APPROACH_OBJECT_LOST_BUFFER_TIME)
        {
            LOG_WARNING(logging::g_qSharedLogger, "VerifyingObjectState: Lost visual entirely. Fast failing verification.");
            globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingFailed);
            return;
        }

        // 5. Final Evaluation based on elapsed verification time
        if (dElapsedTime >= constants::APPROACH_OBJECT_VERIFY_TIME)
        {
            // Calculate the percentage of time the object was validly tracked
            double dTimeHitRate = m_dTotalValidTime / constants::APPROACH_OBJECT_VERIFY_TIME;

            if (dTimeHitRate >= constants::APPROACH_OBJECT_REQUIRED_TIME_HIT_RATE)
            {
                LOG_NOTICE(logging::g_qSharedLogger,
                           "VerifyingObjectState: SUCCESS! Confirmed visually for {:.2f}s ({:.2f}% of the required window).",
                           m_dTotalValidTime,
                           dTimeHitRate * 100.0);
                globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingComplete);
            }
            else
            {
                LOG_WARNING(logging::g_qSharedLogger,
                            "VerifyingObjectState: FALSE POSITIVE. Confirmed visually for only {:.2f}s ({:.2f}% of window).",
                            m_dTotalValidTime,
                            dTimeHitRate * 100.0);
                globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingFailed);
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

                // Loop through the detectors vector and find which ones UUID matches the winning tag's UUID.
                // If a match is found, request the snapshot from that detector and save it to disk with a unique filename.
                cv::Mat cvSnapshot;
                for (const std::shared_ptr<ObjectDetector>& pObjectDetector : m_vObjectDetectors)
                {
                    if (pObjectDetector->GetThreadUUID() == m_stBestObject.szDetectorUUID)
                    {
                        // Load the detector's newest last-good overlay snapshot once into a local.
                        // The ObjectDetectionHandler holds a Subscription to this channel for the
                        // detector's lifetime, so it is being published. This read is lock free and
                        // never blocks on the detector's loop.
                        pubsub::Publisher<cv::Mat>::SharedSnapshot pSnapshot = pObjectDetector->GetLastGoodOverlayPublisher().Get();
                        if (pSnapshot != nullptr)
                        {
                            // Deep copy the immutable snapshot so we own the frame we are about to save.
                            pSnapshot->tData.copyTo(cvSnapshot);
                        }
                        else
                        {
                            // Submit logger message.
                            LOG_WARNING(logging::g_qSharedLogger, "VerifyingObjectState: No detection overlay frame has been published yet.");
                        }
                        break;
                    }
                }

                // Check if the snapshot is empty.
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
                LOG_INFO(logging::g_qSharedLogger, "VerifyingObjectState: Handling Verifying Failed/Object Unseen event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Recall the previous state.
                eNextState = globals::g_pStateMachineHandler->GetPreviousState();
                break;
            }
            case Event::eStuck:
            {
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Stuck event.");
                eNextState = States::eStuck;
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
