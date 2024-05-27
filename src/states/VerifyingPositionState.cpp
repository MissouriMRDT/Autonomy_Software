/******************************************************************************
 * @brief Verifying Position State Implementation for Autonomy State Machine.
 *
 * @file VerifyingPositionState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-05-24
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "VerifyingPositionState.h"
#include "../AutonomyGlobals.h"
#include "../AutonomyNetworking.h"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-05-24
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief This method is called when the state is first started. It is used to
     *        initialize the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-05-24
     ******************************************************************************/
    void VerifyingPositionState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "VerifyingPositionState: Scheduling next run of state logic.");

        m_vCheckPoints.reserve(m_nMaxDataPoints);
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-05-24
     ******************************************************************************/
    void VerifyingPositionState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "VerifyingPositionState: Exiting state.");

        m_vCheckPoints.clear();
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-05-24
     ******************************************************************************/
    VerifyingPositionState::VerifyingPositionState() : State(States::eVerifyingPosition)
    {
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

        m_nMaxDataPoints = 100;

        m_bInitialized   = false;

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
     * @date 2024-05-24
     ******************************************************************************/
    void VerifyingPositionState::Run()
    {
        LOG_DEBUG(logging::g_qSharedLogger, "VerifyingPositionState: Running state-specific behavior.");

        if (m_vCheckPoints.size() < m_nMaxDataPoints)
        {
            if (!globals::g_pNavigationBoard->IsOutOfDate())
            {
                m_vCheckPoints.push_back(globals::g_pNavigationBoard->GetGPSData());
            }
        }
        else
        {
            // Create Average GPS Coordinate
            geoops::GPSCoordinate stAverage = geoops::GPSCoordinate();

            // Calculate Sum of GPS Coordinates
            for (geoops::GPSCoordinate& stPoint : m_vCheckPoints)
            {
                stAverage.dLatitude += stPoint.dLatitude;
                stAverage.dLongitude += stPoint.dLongitude;
            }

            // Calculate Average GPS Coordinate
            stAverage.dLatitude /= m_vCheckPoints.size();
            stAverage.dLongitude /= m_vCheckPoints.size();

            // Calculate distance and bearing from goal waypoint.
            geoops::GeoMeasurement stGoalWaypointMeasurement =
                geoops::CalculateGeoMeasurement(stAverage, globals::g_pWaypointHandler->PeekNextWaypoint().GetGPSCoordinate());

            // Check if the rover is within the goal waypoint's tolerance.
            if (stGoalWaypointMeasurement.dDistanceMeters > constants::NAVIGATING_REACHED_GOAL_RADIUS)
            {
                // Trigger event to transition to next state.
                globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingFailed, true);
            }
            else
            {
                // Trigger event to transition to next state.
                globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingComplete, true);
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
     * @date 2024-05-24
     ******************************************************************************/
    States VerifyingPositionState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eVerifyingPosition;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingPositionState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eVerifyingComplete:
            {
                LOG_INFO(logging::g_qSharedLogger, "VerifyingPositionState: Handling Verifying Complete event.");
                eNextState = States::eIdle;
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);

                // Send Reached Goal state over RoveComm.
                // Construct a RoveComm packet.
                rovecomm::RoveCommPacket<uint8_t> stPacket;
                stPacket.unDataId    = manifest::Autonomy::TELEMETRY.find("REACHEDGOAL")->second.DATA_ID;
                stPacket.unDataCount = manifest::Autonomy::TELEMETRY.find("REACHEDGOAL")->second.DATA_COUNT;
                stPacket.eDataType   = manifest::Autonomy::TELEMETRY.find("REACHEDGOAL")->second.DATA_TYPE;
                stPacket.vData.emplace_back(1);
                // Send telemetry over RoveComm to all subscribers.
                network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, "0.0.0.0", constants::ROVECOMM_OUTGOING_UDP_PORT);

                // Pop the next waypoint.
                globals::g_pWaypointHandler->PopNextWaypoint();
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            case Event::eVerifyingFailed:
            {
                LOG_INFO(logging::g_qSharedLogger, "VerifyingPositionState: Handling Verifying Failed event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eNavigating;
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "VerifyingPositionState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "VerifyingPositionState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eVerifyingPosition)
        {
            LOG_INFO(logging::g_qSharedLogger, "VerifyingPositionState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
