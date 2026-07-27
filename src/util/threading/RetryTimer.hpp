/******************************************************************************
 * @brief Defines and implements the RetryTimer class: a monotonic, single-thread
 *      rate limiter for retrying an operation that has failed.
 *
 *      This replaces the wall-clock modulus idiom the camera classes previously used
 *      to pace reconnect attempts:
 *
 *          int nSecondsSinceEpoch = ...;
 *          if (nSecondsSinceEpoch % 5 == 0 && !m_bAlreadyChecked) { retry(); m_bAlreadyChecked = true; }
 *          else if (nSecondsSinceEpoch % 5 != 0) { m_bAlreadyChecked = false; }
 *
 *      That idiom has three problems this class does not: it silently skips an entire
 *      retry window if no loop iteration happens to land inside the matching second,
 *      it fires at a rate that depends on the caller's loop period, and it reads the
 *      system clock, so an NTP step or manual clock change can stall or storm the
 *      retries. RetryTimer uses steady_clock and an explicit deadline instead.
 *
 * @file RetryTimer.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-26
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#ifndef RETRY_TIMER_HPP
#define RETRY_TIMER_HPP

/// \cond
#include <chrono>

/// \endcond

/******************************************************************************
 * @brief Namespace containing the threading utilities used to structure the
 *      producer/consumer classes that derive from AutonomyThread.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-26
 ******************************************************************************/
namespace threadutils
{
    /******************************************************************************
     * @brief A monotonic rate limiter for retrying a failed operation. Ready() returns
     *      true at most once per configured interval, so a loop that calls it every
     *      iteration retries at a fixed wall-clock rate regardless of how fast that
     *      loop runs.
     *
     *      The first call to Ready() returns true, so the first retry happens
     *      immediately rather than after an initial interval of doing nothing.
     *
     * @note This class is not thread safe. It is intended to be a member of an
     *      AutonomyThread child and touched only from that thread's loop.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2026-07-26
     ******************************************************************************/
    class RetryTimer
    {
        public:
            /******************************************************************************
             * @brief Construct a new RetryTimer object.
             *
             * @param tmInterval - The minimum time between two successful Ready() calls.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            explicit RetryTimer(const std::chrono::milliseconds tmInterval) : m_tmInterval(tmInterval) {}

            /******************************************************************************
             * @brief Check whether enough time has elapsed to attempt another retry. When
             *      this returns true it also arms the next deadline, so a caller that polls
             *      every loop iteration gets exactly one true per interval.
             *
             * @return true - The caller should attempt the operation now.
             * @return false - Not enough time has elapsed; skip this iteration.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            bool Ready()
            {
                // Read the monotonic clock. Immune to wall-clock steps, unlike system_clock.
                const std::chrono::steady_clock::time_point tmNow = std::chrono::steady_clock::now();

                // The very first call always fires so the first attempt is not delayed.
                if (!m_bArmed)
                {
                    // Arm the timer and allow this attempt.
                    m_bArmed         = true;
                    m_tmNextAttempt  = tmNow + m_tmInterval;
                    return true;
                }

                // Not enough time has elapsed since the last attempt.
                if (tmNow < m_tmNextAttempt)
                {
                    // Tell the caller to skip.
                    return false;
                }

                // Deadline reached: schedule the next one and allow this attempt.
                m_tmNextAttempt = tmNow + m_tmInterval;
                return true;
            }

            /******************************************************************************
             * @brief Reset the timer so the next Ready() call fires immediately. Call this
             *      when the underlying operation succeeds, so a later failure retries at
             *      once instead of waiting out a stale deadline.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            void Reset() { m_bArmed = false; }

            /******************************************************************************
             * @brief Accessor for the configured retry interval.
             *
             * @return std::chrono::milliseconds - The minimum time between retries.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            std::chrono::milliseconds GetInterval() const { return m_tmInterval; }

        private:
            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////

            std::chrono::milliseconds m_tmInterval;                    // Minimum time between two successful Ready() calls.
            std::chrono::steady_clock::time_point m_tmNextAttempt;     // When the next attempt becomes allowed.
            bool m_bArmed = false;                                     // False until the first Ready() call arms the deadline.
    };
}    // namespace threadutils

#endif    // RETRY_TIMER_HPP
