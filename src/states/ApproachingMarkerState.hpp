/******************************************************************************
 * @brief Approaching Marker State Implementation for Autonomy State Machine.
 *
 * @file ApproachingMarkerState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef APPROACHINGMARKERSTATE_HPP
#define APPROACHINGMARKERSTATE_HPP

#include "../AutonomyGlobals.h"
#include "../interfaces/State.hpp"
#include "../util/NumberOperations.hpp"
#include "../vision/aruco/ArucoDetection.hpp"
#include "../vision/aruco/TagDetector.h"
#include "../vision/aruco/TensorflowDetection.hpp"

#include <type_traits>

#define DETECT_ATTEMPTS_LIMIT   5
#define MOTOR_SPEED_ON_APPROACH 0.5
#define CLOSE_ENOUGH            5

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief The ApproachingMarkerState class implements the Approaching Marker
     *        state for the Autonomy State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class ApproachingMarkerState : public State
    {
        private:
            bool m_bInitialized;

            int m_nNumDetectionAttempts;

            int m_nTargetTagID;
            bool m_bDetected;
            arucotag::ArucoTag m_stTargetTagAR;
            tensorflowtag::TensorflowTag m_stTargetTagTF;

            double m_dLastTargetHeading;
            double m_dLastTargetDistance;

        protected:
            /******************************************************************************
             * @brief This method is called when the state is first started. It is used to
             *        initialize the state.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            void Start() override
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
            void Exit() override
            {
                // Clean up the state before exiting
                LOG_DEBUG(logging::g_qSharedLogger, "ApproachingMarkerState: Exiting state.");
            }

        public:
            /******************************************************************************
             * @brief Accessor for the State private member. Returns the state as a string.
             *
             * @return std::string - The current state as a string.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            ApproachingMarkerState() : State(States::eApproachingMarker)
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
            States Run() override
            {
                LOG_DEBUG(logging::g_qSharedLogger, "ApproachingMarkerState: Running state-specific behavior.");

                bool bDetectedTagAR;
                bool bDetectedTagTF;

                // Identify target
                if (!m_bDetected && m_nNumDetectionAttempts < DETECT_ATTEMPTS_LIMIT)
                {
                    bDetectedTagAR = IdentifyTargetMarker(m_stTargetTagAR);
                    if (bDetectedTagAR)
                    {
                        m_nTargetTagID          = m_stTargetTagAR.nID;
                        m_bDetected             = true;
                        m_nNumDetectionAttempts = 0;
                    }
                    else
                    {
                        bDetectedTagTF = IdentifyTargetMarker(m_stTargetTagTF);
                        if (bDetectedTagTF)
                        {
                            m_nTargetTagID          = m_stTargetTagTF.nID;
                            m_bDetected             = true;
                            m_nNumDetectionAttempts = 0;
                        }
                    }

                    if (!m_bDetected)
                    {
                        ++m_nNumDetectionAttempts;
                    }

                    return States::eApproachingMarker;
                }
                else if (!m_bDetected)
                {
                    // Abort
                    globals::g_pStateMachineHandler->HandleEvent(Event::eAbort);
                    return States::eApproachingMarker;
                }

                // Update target
                bDetectedTagAR = FindTargetMarker(m_nTargetTagID, m_stTargetTagAR);
                if (!bDetectedTagAR)
                {
                    bDetectedTagTF = FindTargetMarker(m_nTargetTagID, m_stTargetTagAR);
                }

                if (!bDetectedTagAR && !bDetectedTagTF)
                {
                    // No tag found
                    ++m_nNumDetectionAttempts;
                }
                else
                {
                    m_nNumDetectionAttempts = 0;
                }

                // Give up on target
                if (m_nNumDetectionAttempts >= DETECT_ATTEMPTS_LIMIT)
                {
                    globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerUnseen);
                    return States::eApproachingMarker;
                }

                // Find tag information
                double dTargetYaw;
                double dTargetDistance;
                if (bDetectedTagAR)
                {
                    dTargetYaw      = m_stTargetTagAR.dYawAngle;
                    dTargetDistance = m_stTargetTagAR.dStraightLineDistance;
                }
                else if (bDetectedTagTF)
                {
                    dTargetYaw      = m_stTargetTagTF.dYawAngle;
                    dTargetDistance = m_stTargetTagTF.dStraightLineDistance;
                }
                else
                {
                    dTargetYaw      = m_dLastTargetHeading;
                    dTargetDistance = m_dLastTargetDistance;
                }
                m_dLastTargetHeading  = dTargetYaw;
                m_dLastTargetDistance = dTargetDistance;

                // Close enough
                if (dTargetDistance < CLOSE_ENOUGH)
                {
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedMarker);
                    return States::eApproachingMarker;
                }

                // Move to target
                double dCurrHeading   = globals::g_pNavigationBoard->GetIMUData().dHeading;
                double dTargetHeading = numops::InputAngleModulus<double>(dCurrHeading + dTargetYaw, 0, 360);

                diffdrive::DrivePowers stDrivePowers =
                    globals::g_pDriveBoard->CalculateMove(MOTOR_SPEED_ON_APPROACH, dTargetHeading, dCurrHeading, diffdrive::eTankDrive);
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
            States TriggerEvent(Event eEvent) override
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

            template<typename T>
            bool FindTargetMarker(int nID, T& tIdentifiedTag)
            {
                std::vector<T> vDetectedTags;
                LoadDetectedTags(vDetectedTags);

                typename std::vector<T>::const_iterator itDetectedTag = vDetectedTags.begin();
                while (itDetectedTag != vDetectedTags.end())
                {
                    if (itDetectedTag->nID == nID)
                    {
                        tIdentifiedTag = *itDetectedTag;
                        return true;
                    }
                }

                return false;
            }

            template<typename T>
            bool IdentifyTargetMarker(T& tTarget)
            {
                std::vector<T> vDetectedTags;
                LoadDetectedTags(vDetectedTags);

                T tBestTag;
                tBestTag.dStraightLineDistance = std::numeric_limits<double>::max();
                tBestTag.nID                   = -1;

                typename std::vector<T>::const_iterator itCandidate;
                while (itCandidate != vDetectedTags.end())
                {
                    if (itCandidate->dStraightLineDistance < tBestTag.dStraightLineDistance)
                    {
                        tBestTag = *itCandidate;
                    }
                    ++itCandidate;
                }

                if (tBestTag.nID >= 0)
                {
                    tTarget = tBestTag;
                    return true;
                }
                else
                {
                    return false;
                }
            }

            template<typename T>
            void LoadDetectedTags(std::vector<T> vDetectedTags)
            {
                TagDetector* pTagDetectorMainCam  = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam);
                TagDetector* pTagDetectorLeftEye  = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadLeftArucoEye);
                TagDetector* pTagDetectorRightEye = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadRightArucoEye);

                std::vector<T> vDetectedTagsMain;
                std::vector<T> vDetectedTagsLeft;
                std::vector<T> vDetectedTagsRight;

                std::future<bool> fuDetectedTagsMain;
                std::future<bool> fuDetectedTagsLeft;
                std::future<bool> fuDetectedTagsRight;

                if constexpr (std::is_same_v<T, arucotag::ArucoTag>)
                {
                    fuDetectedTagsMain  = pTagDetectorMainCam->RequestDetectedArucoTags(vDetectedTagsMain);
                    fuDetectedTagsLeft  = pTagDetectorLeftEye->RequestDetectedArucoTags(vDetectedTagsLeft);
                    fuDetectedTagsRight = pTagDetectorRightEye->RequestDetectedArucoTags(vDetectedTagsRight);
                }
                else if constexpr (std::is_same_v<T, tensorflowtag::TensorflowTag>)
                {
                    fuDetectedTagsMain  = pTagDetectorMainCam->RequestDetectedTensorflowTags(vDetectedTagsMain);
                    fuDetectedTagsLeft  = pTagDetectorLeftEye->RequestDetectedTensorflowTags(vDetectedTagsLeft);
                    fuDetectedTagsRight = pTagDetectorRightEye->RequestDetectedTensorflowTags(vDetectedTagsRight);
                }
                else
                {
                    // TODO: LOG ERROR
                }

                fuDetectedTagsMain.wait();
                vDetectedTags.insert(vDetectedTags.end(), vDetectedTagsMain.begin(), vDetectedTagsMain.end());

                fuDetectedTagsLeft.wait();
                vDetectedTags.insert(vDetectedTags.end(), vDetectedTagsLeft.begin(), vDetectedTagsLeft.end());

                fuDetectedTagsRight.wait();
                vDetectedTags.insert(vDetectedTags.end(), vDetectedTagsRight.begin(), vDetectedTagsRight.end());
            }
    };    // namespace statemachine
}    // namespace statemachine

#endif    // APPROACHINGMARKERSTATE_HPP
