/******************************************************************************
 * @brief Search Pattern State Implementation for Autonomy State Machine.
 *
 * @file SearchPatternState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "SearchPatternState.h"
#include "../AutonomyGlobals.h"
#include "../algorithms/DifferentialDrive.hpp"
#include "../algorithms/SearchPattern.hpp"
#include "../interfaces/State.hpp"

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
    void SearchPatternState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Scheduling next run of state logic.");

        m_nMaxDataPoints             = 100;
        m_tStuckCheckTime            = time(nullptr);

        m_dStuckCheckLastPosition[0] = 0;
        m_dStuckCheckLastPosition[1] = 0;

        m_vRoverXPosition.reserve(m_nMaxDataPoints);
        m_vRoverYPosition.reserve(m_nMaxDataPoints);

        // Calculate the search path.
        geoops::GPSCoordinate stCurrentPosGPS = globals::g_pNavigationBoard->GetGPSData();
        m_vSearchPath                         = searchpattern::CalculateSearchPatternWaypoints(stCurrentPosGPS,
                                                                       constants::SEARCH_ANGULAR_STEP_DEGREES,
                                                                       constants::SEARCH_MAX_RADIUS,
                                                                       constants::SEARCH_STARTING_HEADING_DEGREES,
                                                                       constants::SEARCH_SPACING);
        m_nSearchPathIdx                      = 0;

        m_vTagDetectors                       = {globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam),
                                                 globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadLeftArucoEye),
                                                 globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadRightArucoEye)};
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void SearchPatternState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Exiting state.");

        m_vRoverXPosition.clear();
        m_vRoverYPosition.clear();
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    SearchPatternState::SearchPatternState() : State(States::eSearchPattern)
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
    void SearchPatternState::Run()
    {
        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Running state-specific behavior.");

        /*
            The overall flow of this state is as follows.
            1. Is there a tag -> MarkerSeen
            2. Is there an object -> ObjectSeen
            3. Is there an obstacle -> TBD
            4. Is the rover stuck -> Stuck
            5. Is the search pattern complete -> Abort
            6. Follow the search pattern.
        */

        /* --- Detect Tags --- */
        std::vector<arucotag::ArucoTag> vDetectedArucoTags;
        std::vector<tensorflowtag::TensorflowTag> vDetectedTensorflowTags;

        tagdetectutils::LoadDetectedArucoTags(vDetectedArucoTags, m_vTagDetectors, false);
        tagdetectutils::LoadDetectedTensorflowTags(vDetectedTensorflowTags, m_vTagDetectors, false);

        if (vDetectedArucoTags.size() || vDetectedTensorflowTags.size())
        {
            globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerSeen);
            return;
        }

        /* --- Detect Objects --- */

        // TODO: Add object detection to SearchPattern state

        /* --- Detect Obstacles --- */

        // TODO: Add obstacle detection to SearchPattern state

        /* --- Check if Stuck --- */

        // TODO: Add the ability to check if stuck

        /* --- Follow Search Pattern --- */
        // Determine the next waypoint
        geoops::GPSCoordinate stCurrentPosGPS    = globals::g_pNavigationBoard->GetGPSData();
        geoops::GPSCoordinate stCurrTargetGPS    = m_vSearchPath[m_nSearchPathIdx].GetGPSCoordinate();
        geoops::GeoMeasurement stCurrRelToTarget = geoops::CalculateGeoMeasurement(stCurrentPosGPS, stCurrTargetGPS);
        bool bReachedTarget                      = stCurrRelToTarget.dDistanceMeters <= constants::SEARCH_WAYPOINT_PROXIMITY;

        if (bReachedTarget && m_nSearchPathIdx == (int) m_vSearchPath.size())
        {
            globals::g_pStateMachineHandler->HandleEvent(Event::eSearchFailed);
            return;
        }
        else if (bReachedTarget)
        {
            ++m_nSearchPathIdx;
            stCurrTargetGPS   = m_vSearchPath[m_nSearchPathIdx].GetGPSCoordinate();
            stCurrRelToTarget = geoops::CalculateGeoMeasurement(stCurrentPosGPS, stCurrTargetGPS);
        }

        // Drive to target waypoint.
        double dCurrHeading = globals::g_pNavigationBoard->GetHeading();
        diffdrive::DrivePowers stDrivePowers =
            globals::g_pDriveBoard->CalculateMove(constants::SEARCH_MOTOR_POWER, stCurrRelToTarget.dStartRelativeBearing, dCurrHeading, diffdrive::eTankDrive);
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
    States SearchPatternState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eSearchPattern;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eMarkerSeen:
            {
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling MarkerSeen event.");
                eNextState = States::eApproachingMarker;
                break;
            }
            case Event::eObjectSeen:
            {
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling ObjectSeen event.");
                eNextState = States::eApproachingObject;
                break;
            }
            case Event::eStart:
            {
                // Submit logger message
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eSearchFailed:
            {
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling SearchFailed event.");
                eNextState = States::eIdle;
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            case Event::eStuck:
            {
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling Stuck event.");
                eNextState = States::eStuck;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "SearchPatternState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eSearchPattern)
        {
            LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
