/******************************************************************************
 * @brief Defines and implements different structs and functions used to detect if
 *      the rover is stuck.
 *
 * @file StuckDetection.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-23
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef STUCK_DETECTION_HPP
#define STUCK_DETECTION_HPP

/// \cond
// Put implicit includes in here.
#include <chrono>
#include <cmath>

/// \endcond

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-23
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief This class should be instantiated within another state to be used
     *      for detection of if the rover is stuck. Stuck detection is solely based
     *      off of a check interval on the current velocity and rotation. If the velocity
     *      and rotation are non-moving for more then a maximum interval count, we are
     *      considered stuck.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-04-23
     ******************************************************************************/
    class TimeIntervalBasedStuckDetector
    {
        private:
            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////

            double m_dMaximumStuckCount;
            double m_dStuckCheckIntervalSeconds;
            double m_dVelocityThreshold;
            double m_dAngularVelocityThreshold;
            unsigned int m_unStuckChecksSoFar;
            std::chrono::system_clock::time_point m_tmTimeSinceLastStuckCheck;

        public:
            /////////////////////////////////////////
            // Declare public class methods.
            /////////////////////////////////////////

            /******************************************************************************
             * @brief Construct a new Stuck Detector object.
             *
             * @param dMaximumStuckCount - The maximum number of times the rover can be not moving upon check interval before it is considered stuck.
             * @param dStuckCheckIntervalSeconds - The interval in seconds that the function should check the current rover velocity and angular movement.
             * @param dVelocityThreshold - The minimum linear velocity that is considered still moving. (m/s)
             * @param dAngularVelocityThreshold - The minimum angular velocity that is considered still rotating. (deg/s)
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-04-23
             ******************************************************************************/
            TimeIntervalBasedStuckDetector(double dMaximumStuckCount         = 3,
                                           double dStuckCheckIntervalSeconds = 3,
                                           double dVelocityThreshold         = 0.3,
                                           double dAngularVelocityThreshold  = 5.0)
            {
                // Initialize member variables.
                m_dMaximumStuckCount         = dMaximumStuckCount;
                m_dStuckCheckIntervalSeconds = dStuckCheckIntervalSeconds;
                m_dVelocityThreshold         = dVelocityThreshold;
                m_dAngularVelocityThreshold  = dAngularVelocityThreshold;
                m_unStuckChecksSoFar         = 0;
                m_tmTimeSinceLastStuckCheck  = std::chrono::system_clock::now();
            }

            /******************************************************************************
             * @brief Destroy the Stuck Detector object.
             *
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-04-23
             ******************************************************************************/
            ~TimeIntervalBasedStuckDetector() {}

            /******************************************************************************
             * @brief Checks if the rover meets stuck criteria based in the given parameters.
             *
             *
             * @return true - The rover is stuck.
             * @return false - The is not stuck.
             *
             * @author clayjay3 (claytonraycowen@gmail.com), Jason Pittman (jspencerpittman@gmail.com)
             * @date 2024-04-23
             ******************************************************************************/
            bool CheckIfStuck(double dCurrentVelocity, double dCurrentAngularVelocity)
            {
                // Create instance variables.
                bool bStuck = false;

                // Time since we last checked if the rover is stuck.
                std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
                double dTimeSinceLastCheck = std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmTimeSinceLastStuckCheck).count();
                if (dTimeSinceLastCheck > m_dStuckCheckIntervalSeconds)
                {
                    // Update time since last check to now.
                    m_tmTimeSinceLastStuckCheck = tmCurrentTime;

                    // Check if the rover is rotating or moving linearly.
                    if (std::abs(dCurrentVelocity) < m_dVelocityThreshold && std::abs(dCurrentAngularVelocity) < m_dAngularVelocityThreshold)
                    {
                        ++m_unStuckChecksSoFar;
                    }
                    else
                    {
                        m_unStuckChecksSoFar = 0;
                    }
                }

                // Check if we met stuck criteria.
                if (m_unStuckChecksSoFar > m_dMaximumStuckCount)
                {
                    // Reset stuck checks.
                    m_unStuckChecksSoFar = 0;
                    // Set stuck toggle.
                    bStuck = true;
                }

                // Return if stuck or not.
                return bStuck;
            }
    };
}    // namespace statemachine

#endif
