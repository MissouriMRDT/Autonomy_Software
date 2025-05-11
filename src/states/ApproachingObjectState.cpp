/******************************************************************************
 * @brief Approaching Object State Implementation for Autonomy State Machine.
 *
 * @file ApproachingObjectState.cpp
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "ApproachingObjectState.h"
#include "../AutonomyConstants.h"
#include "../AutonomyGlobals.h"
#include "../AutonomyNetworking.h"
#include "../util/states/ObjectDetectionChecker.hpp"

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
    void ApproachingObjectState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Scheduling next run of state logic.");

        // Initialize member variables.
        m_stGoalWaypoint = globals::g_pWaypointHandler->PeekNextWaypoint();

        // Store the state that got stuck and triggered an ObjectSeen event.
        m_eTriggeringState = globals::g_pStateMachineHandler->GetPreviousState();

        // Add the search and rover path layers to the plot.
        m_pRoverPathPlot->CreateDotLayer("DetectedObjects", "blue");
        m_pRoverPathPlot->CreateDotLayer("FinalObject", "green");
        m_pRoverPathPlot->CreatePathLayer("RoverPath", "-.r*");

        m_vObjectDetectors = {globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eHeadMainCam)};
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingObjectState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Exiting state.");
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    ApproachingObjectState::ApproachingObjectState() : State(States::eApproachingObject)
    {
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

        m_bInitialized   = false;

        m_StuckDetector  = statemachine::TimeIntervalBasedStuckDetector(constants::STUCK_CHECK_ATTEMPTS,
                                                                       constants::STUCK_CHECK_INTERVAL,
                                                                       constants::STUCK_CHECK_VEL_THRESH,
                                                                       constants::STUCK_CHECK_ROT_THRESH);
        m_pRoverPathPlot = std::make_unique<logging::graphing::PathTracer>("ApproachingObjectRoverPath");

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
    void ApproachingObjectState::Run()
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "ApproachingObjectState: Running state-specific behavior.");

        // Get the current rover pose.
        geoops::RoverPose stCurrentRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();

        // Add the current rover pose to the path plot.
        m_pRoverPathPlot->AddPathPoint(stCurrentRoverPose.GetUTMCoordinate(), "RoverPath");

        // Check Rover radius from object waypoint.
        geoops::GeoMeasurement stCurrentMeasurement = geoops::CalculateGeoMeasurement(m_stGoalWaypoint.GetGPSCoordinate(), stCurrentRoverPose.GetGPSCoordinate());
        if (stCurrentMeasurement.dDistanceMeters > m_stGoalWaypoint.dRadius)
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger,
                        "ApproachingObjectState: Rover is too far from the original waypoint! Waypoint radius is {} meters, current distance is {} meters.",
                        m_stGoalWaypoint.dRadius,
                        stCurrentMeasurement.dDistanceMeters);
            globals::g_pStateMachineHandler->HandleEvent(Event::eObjectUnseen);
            return;
        }

        // Identify target object.
        objectdetectutils::Object stBestObject;
        statemachine::IdentifyTargetObject(m_vObjectDetectors, stBestObject);

        // Check if object is unseen.
        static bool bAlreadyPrintedLost                            = false;
        static std::chrono::system_clock::time_point tLastSeenTime = std::chrono::system_clock::now();
        if (stBestObject.dConfidence == 0.0)
        {
            std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
            if ((std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - tLastSeenTime).count() / 1000.0) > constants::APPROACH_OBJECT_LOST_GIVE_UP_TIME)
            {
                // Submit logger message.
                globals::g_pStateMachineHandler->HandleEvent(Event::eObjectUnseen);
                return;
            }
            else
            {
                if (!bAlreadyPrintedLost)
                {
                    bAlreadyPrintedLost = true;
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "ApproachingObjectState: No objects detected.");

                    // If an object is good and has a valid geoposition, don't stop the drive, we can keep driving to it.
                    if (stBestObject.dConfidence != 0.0 && stBestObject.stGeolocatedPosition.eType != geoops::WaypointType::eUNKNOWN)
                    {
                        // Submit logger message.
                        LOG_NOTICE(logging::g_qSharedLogger, "ApproachingObjectState: Object is geolocated.");
                        return;
                    }

                    // Stop the drive.
                    globals::g_pDriveBoard->SendStop();
                    return;
                }
            }
        }

        else
        {
            // Submit logger message.
            if (bAlreadyPrintedLost)
            {
                LOG_NOTICE(logging::g_qSharedLogger, "ApproachingObjectState: Objects were detected again.");
            }

            // Reset the last seen time if a tag is detected.
            tLastSeenTime = std::chrono::system_clock::now();
            // Reset printed flag when tags are detected again.
            bAlreadyPrintedLost = false;
        }

        // Create instance variables.
        static double dHeadingSetPoint    = 0.0;
        static double dDistanceFromObject = 0.0;
        // Check if we got a good object.
        if (stBestObject.dConfidence != 0.0)
        {
            dDistanceFromObject = stBestObject.dStraightLineDistance;
            // Check if the object has an absolute coordinate populated.
            if (stBestObject.stGeolocatedPosition.eType != geoops::WaypointType::eUNKNOWN)
            {
                // Calculate the geomeasurement to the object.
                geoops::GeoMeasurement stObjectMeasurement =
                    geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetUTMCoordinate(), stBestObject.stGeolocatedPosition.GetUTMCoordinate());
                // Update static variables.
                dHeadingSetPoint = stObjectMeasurement.dStartRelativeBearing;
                // Add the most recent geolocated object to the path plot.
                m_pRoverPathPlot->AddDot(stBestObject.stGeolocatedPosition.GetUTMCoordinate(), "DetectedObjects");
            }
            else
            {
                dHeadingSetPoint = numops::InputAngleModulus(stBestObject.dYawAngle + stCurrentRoverPose.GetCompassHeading(), 0.0, 360.0);
            }
        }

        // Move the rover to the target's estimated position.
        diffdrive::DrivePowers stDrivePowers = globals::g_pDriveBoard->CalculateMove(constants::APPROACH_OBJECT_MOTOR_POWER,
                                                                                     dHeadingSetPoint,
                                                                                     stCurrentRoverPose.GetCompassHeading(),
                                                                                     diffdrive::DifferentialControlMethod::eArcadeDrive);
        globals::g_pDriveBoard->SendDrive(stDrivePowers);

        // Static variable to track last log time.
        static std::chrono::system_clock::time_point tmLastOLogTime = std::chrono::system_clock::now();
        std::chrono::system_clock::time_point tmCurrentTime         = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - tmLastOLogTime).count() >= 1)
        {
            // Update the last log time.
            tmLastOLogTime = tmCurrentTime;

            if (stBestObject.dConfidence != 0.0)
            {
                LOG_NOTICE(logging::g_qSharedLogger,
                           "ApproachingObjectState: Object confidence: {:.2f}, Distance: {:.2f} m, Heading: {:.2f} deg",
                           stBestObject.dConfidence,
                           dDistanceFromObject,
                           dHeadingSetPoint);
            }
        }

        // Check if tag is reached.
        if (dDistanceFromObject != 0.0 && dDistanceFromObject < constants::APPROACH_OBJECT_PROXIMITY_THRESHOLD)
        {
            // Submit logger message.
            LOG_NOTICE(logging::g_qSharedLogger, "ApproachingObjectState: Rover has reached the target object!");
            // Check if the object has a good absolute position.
            if (stBestObject.dConfidence != 0.0 && stBestObject.stGeolocatedPosition.eType == geoops::WaypointType::eObjectWaypoint)
            {
                // Add the object to the path plot.
                m_pRoverPathPlot->AddDot(stBestObject.stGeolocatedPosition.GetUTMCoordinate(), "FinalObject", 7);
            }

            // Reset the object heading and distance.
            dHeadingSetPoint    = 0.0;
            dDistanceFromObject = 0.0;

            // Handle state transition and save the current search pattern state.
            globals::g_pStateMachineHandler->HandleEvent(Event::eReachedObject, true);
            // Don't execute the rest of the state.
            return;
        }

        //////////////////////////////////////////
        // ---  Check if the rover is stuck --- //
        //////////////////////////////////////////

        // Check if stuck.
        if (m_StuckDetector.CheckIfStuck(globals::g_pWaypointHandler->SmartRetrieveVelocity(), globals::g_pWaypointHandler->SmartRetrieveAngularVelocity()))
        {
            // Submit logger message.
            LOG_NOTICE(logging::g_qSharedLogger, "ApproachingObjectState: Rover has become stuck!");
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
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    States ApproachingObjectState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eApproachingObject;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eReachedObject:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Handling ReachedObject event.");

                // Check if verifying object state is enabled.
                if (constants::APPROACH_OBJECT_VERIFY_POSITION)
                {
                    // Change states.
                    eNextState = States::eVerifyingObject;
                }
                else
                {
                    // Send multimedia command to update state display.
                    globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
                    // Pop old waypoint out of queue.
                    globals::g_pWaypointHandler->PopNextWaypoint();
                    // Clear saved search pattern state.
                    globals::g_pStateMachineHandler->ClearSavedStates();
                    // Submit logger message.
                    LOG_NOTICE(logging::g_qSharedLogger, "ApproachingObjectState: Cleared old search pattern state and approaching object state from saved states.");
                    // Change state.
                    eNextState = States::eIdle;
                }
                break;
            }
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eObjectUnseen:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Handling ObjectUnseen event.");
                // Change states.
                eNextState = m_eTriggeringState;
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "ApproachingObjectState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eApproachingMarker)
        {
            LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
