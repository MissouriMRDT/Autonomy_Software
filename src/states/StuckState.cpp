/******************************************************************************
 * @brief Stuck State Implementation for Autonomy State Machine.
 *
 * @file StuckState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "StuckState.h"
#include "../AutonomyConstants.h"
#include "../AutonomyGlobals.h"
#include "../algorithms/DifferentialDrive.hpp"

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
    void StuckState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "StuckState: Scheduling next run of state logic.");

        // Initialize member variables.
        m_dOriginalHeading     = 0;
        m_bIsCurrentlyAligning = false;
        m_eAttemptType         = AttemptType::eReverseCurrentHeading;

        // Store the state that got stuck and triggered a stuck event.
        m_eTriggeringState = globals::g_pStateMachineHandler->GetPreviousState();

        // Store the postion and heading where the rover get stuck.
        geoops::RoverPose stStartRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();
        m_stOriginalPosition               = stStartRoverPose.GetGPSCoordinate();
        m_dOriginalHeading                 = stStartRoverPose.GetCompassHeading();
        // Get state start time.
        m_tmStuckStartTime = std::chrono::system_clock::now();

        // Stop drivetrain.
        globals::g_pDriveBoard->SendStop();
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void StuckState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "StuckState: Exiting state.");
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    StuckState::StuckState() : State(States::eStuck)
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
     * @author Eli Byrd (edbgkk@mst.edu), Jason Pittman (jspencerpittman@gmail.com), clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void StuckState::Run()
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Running state-specific behavior.");

        // Store the current postion and heading.
        geoops::RoverPose stCurrentRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();
        // Get current time.
        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();

        // Check if we are unstuck from our starting spot.
        if (!this->SamePosition(m_stOriginalPosition, stCurrentRoverPose.GetGPSCoordinate()))
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger,
                        "StuckState: Rover has successfully unstuckith itself! A total of {} seconds was wasted being stuck.",
                        std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmStuckStartTime).count());
            // Handing unstuck event. Destroy this unstuck state.
            globals::g_pStateMachineHandler->HandleEvent(Event::eUnstuck, false);
        }
        else
        {
            // Perform unstuck logic.
            switch (m_eAttemptType)
            {
                // On the first attempt we use the rover's original heading so alignment would already be completed.
                case AttemptType::eReverseCurrentHeading:
                {
                    // Submit logger message.
                    LOG_INFO(logging::g_qSharedLogger, "StuckState: Maintaining current heading and reversing...");
                    // Update stuck type enum for if we are still stuck after reversing.
                    m_eAttemptType = AttemptType::eReverseLeft;
                    // Handle reversing event. Save current state.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReverse, true);
                    break;
                }
                    // On the second attempt align the rover constants::STUCK_ALIGN_DEGREES degrees to the right of the original heading instead.
                case AttemptType::eReverseLeft:
                {
                    // Check if we are already realigning.
                    if (!m_bIsCurrentlyAligning)
                    {
                        // Submit logger message.
                        LOG_INFO(logging::g_qSharedLogger, "StuckState: Aligning rover heading {} degrees clockwise...", constants::STUCK_ALIGN_DEGREES);
                        // Set aligning toggle.
                        m_bIsCurrentlyAligning = true;
                        // Update start heading.
                        m_dOriginalHeading = stCurrentRoverPose.GetCompassHeading();
                        // Update start time.
                        m_tmAlignStartTime = std::chrono::system_clock::now();
                    }
                    else
                    {
                        // Calculate time elapsed since realignment was started.
                        double dTimeElapsed = std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmAlignStartTime).count();
                        // Calculate the goal realignment heading.
                        double dGoalHeading = numops::InputAngleModulus<double>(m_dOriginalHeading + constants::STUCK_ALIGN_DEGREES, 0, 360);
                        // Calculate total rotation degrees so far.
                        double dRealignmentDegrees = numops::AngularDifference<double>(stCurrentRoverPose.GetCompassHeading(), dGoalHeading);

                        // Align drivetrain to a certain heading with 0 forward/reverse power.
                        diffdrive::DrivePowers stTurnPowers = globals::g_pDriveBoard->CalculateMove(0.0,
                                                                                                    dGoalHeading,
                                                                                                    stCurrentRoverPose.GetCompassHeading(),
                                                                                                    diffdrive::DifferentialControlMethod::eCurvatureDrive);
                        // Send drive powers.
                        globals::g_pDriveBoard->SendDrive(stTurnPowers);

                        // Check if we have successfully realigned.
                        if (dRealignmentDegrees <= constants::STUCK_ALIGN_TOLERANCE)
                        {
                            // Submit logger message.
                            LOG_INFO(logging::g_qSharedLogger, "StuckState: Realignment complete! Reversing...");
                            // Update stuck type enum for if we are still stuck after reversing.
                            m_eAttemptType = AttemptType::eReverseRight;
                            // Reset currently aligning toggle.
                            m_bIsCurrentlyAligning = false;
                            // Handle reversing event.
                            globals::g_pStateMachineHandler->HandleEvent(Event::eReverse, true);
                        }
                        // If not aligned yet, check if we hit the timeout.
                        else if (dTimeElapsed >= constants::STUCK_HEADING_ALIGN_TIMEOUT)
                        {
                            // Submit logger message.
                            LOG_WARNING(logging::g_qSharedLogger,
                                        "StuckState: Rotated/Realigned {} degrees in {} seconds before timeout was reached. Rover is still stuck...",
                                        constants::STUCK_ALIGN_DEGREES - dRealignmentDegrees,
                                        dTimeElapsed);
                            // Update stuck type enum for if we are still stuck after reversing.
                            m_eAttemptType = AttemptType::eReverseRight;
                            // Reset currently aligning toggle.
                            m_bIsCurrentlyAligning = false;
                            // Handle reversing event.
                            globals::g_pStateMachineHandler->HandleEvent(Event::eReverse, true);
                        }
                    }
                    break;
                }
                // For the third do it constants::STUCK_ALIGN_DEGREES degrees to the left of the original heading.
                case AttemptType::eReverseRight:
                {
                    // Check if we are already realigning.
                    if (!m_bIsCurrentlyAligning)
                    {
                        // Submit logger message.
                        LOG_INFO(logging::g_qSharedLogger, "StuckState: Aligning rover heading {} degrees counter-clockwise...", constants::STUCK_ALIGN_DEGREES);
                        // Set aligning toggle.
                        m_bIsCurrentlyAligning = true;
                        // Update start heading.
                        m_dOriginalHeading = stCurrentRoverPose.GetCompassHeading();
                        // Update start time.
                        m_tmAlignStartTime = std::chrono::system_clock::now();
                    }
                    else
                    {
                        // Calculate time elapsed since realignment was started.
                        double dTimeElapsed = std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmAlignStartTime).count();
                        // Calculate the goal realignment heading.
                        double dGoalHeading = numops::InputAngleModulus<double>(m_dOriginalHeading - constants::STUCK_ALIGN_DEGREES, 0, 360);
                        // Calculate total rotation degrees so far.
                        double dRealignmentDegrees = numops::AngularDifference<double>(stCurrentRoverPose.GetCompassHeading(), dGoalHeading);

                        // Align drivetrain to a certain heading with 0 forward/reverse power.
                        diffdrive::DrivePowers stTurnPowers = globals::g_pDriveBoard->CalculateMove(0.0,
                                                                                                    dGoalHeading,
                                                                                                    stCurrentRoverPose.GetCompassHeading(),
                                                                                                    diffdrive::DifferentialControlMethod::eCurvatureDrive);
                        // Send drive powers.
                        globals::g_pDriveBoard->SendDrive(stTurnPowers);

                        // Check if we have successfully realigned.
                        if (dRealignmentDegrees <= constants::STUCK_ALIGN_TOLERANCE)
                        {
                            // Submit logger message.
                            LOG_INFO(logging::g_qSharedLogger, "StuckState: Realignment complete! Reversing...");
                            // Update stuck type enum for if we are still stuck after reversing.
                            m_eAttemptType = AttemptType::eGiveUp;
                            // Reset currently aligning toggle.
                            m_bIsCurrentlyAligning = false;
                            // Handle reversing event.
                            globals::g_pStateMachineHandler->HandleEvent(Event::eReverse, true);
                        }
                        // If not aligned yet, check if we hit the timeout.
                        else if (dTimeElapsed >= constants::STUCK_HEADING_ALIGN_TIMEOUT)
                        {
                            // Submit logger message.
                            LOG_WARNING(logging::g_qSharedLogger,
                                        "StuckState: Rotated/Realigned {} degrees in {} seconds before timeout was reached. Rover is still stuck...",
                                        constants::STUCK_ALIGN_DEGREES - dRealignmentDegrees,
                                        dTimeElapsed);
                            // Update stuck type enum for if we are still stuck after reversing.
                            m_eAttemptType = AttemptType::eGiveUp;
                            // Reset currently aligning toggle.
                            m_bIsCurrentlyAligning = false;
                            // Handle reversing event.
                            globals::g_pStateMachineHandler->HandleEvent(Event::eReverse, true);
                        }
                    }
                    break;
                }
                case AttemptType::eGiveUp:
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "StuckState: After multiple attempts, autonomy was unable to get the rover unstuck. Giving Up...");
                    // Return to idle.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eAbort);
                    break;
                }
                default:
                {
                    // Submit logger message.
                    LOG_ERROR(logging::g_qSharedLogger, "StuckState: Unknown attempt type!");
                    // Return to idle.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eAbort);
                    break;
                }
            }
        }
    }

    /******************************************************************************
     * @brief Trigger an event in the state machine. Returns the next state.
     *
     * @param eEvent - The event to trigger.
     * @return std::shared_ptr<State> - The next state.
     *
     * @author Eli Byrd (edbgkk@mst.edu), clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    States StuckState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eStuck;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "StuckState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "StuckState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            case Event::eReverse:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "StuckState: Handling Reverse event.");
                // Change state.
                eNextState = States::eReversing;
                break;
            }
            case Event::eUnstuck:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "StuckState: Handling Unstuck event.");
                // Change state back to the state that originally got stuck.
                eNextState = m_eTriggeringState;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "StuckState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eStuck)
        {
            LOG_INFO(logging::g_qSharedLogger, "StuckState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }

    /******************************************************************************
     * @brief Checks if the rover is approximately in the same position.
     *
     * @note The threshold that defines how far away we need to be from the original point to be considered
     *  a different position is constants::STUCK_SAME_POINT_PROXIMITY.
     *
     * @param stLastPosition - Original position the rover was located.
     * @param stCurrPosition - Current position the rover is located.
     * @return true - The rover is in the same position.
     * @return false - The rover is in a different position.
     *
     * @author Jason Pittman (jspencerpittman@gmail.com)
     * @date 2024-02-14
     ******************************************************************************/
    bool StuckState::SamePosition(const geoops::GPSCoordinate& stOriginalPosition, const geoops::GPSCoordinate& stCurrPosition)
    {
        double dDistance = geoops::CalculateGeoMeasurement(stOriginalPosition, stCurrPosition).dDistanceMeters;
        return dDistance <= constants::STUCK_SAME_POINT_PROXIMITY;
    }
}    // namespace statemachine
