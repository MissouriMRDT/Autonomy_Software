/******************************************************************************
 * @brief Unit tests for the threadutils::RetryTimer class.
 *
 * @file RetryTimer.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-26
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/util/threading/RetryTimer.hpp"

/// \cond
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

/// \endcond

/******************************************************************************
 * @brief The first Ready() call must fire immediately so a failed operation is retried
 *      at once instead of sitting idle for a full interval first.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-26
 ******************************************************************************/
TEST(RetryTimerTests, FirstCallFiresImmediately)
{
    // Create a timer with a long interval so only the first-call rule can fire it.
    threadutils::RetryTimer tmTimer(std::chrono::milliseconds(10000));

    // The very first call must be allowed.
    EXPECT_TRUE(tmTimer.Ready());
}

/******************************************************************************
 * @brief After firing once, Ready() must refuse until the interval has elapsed. This is
 *      the property the old wall-clock modulus could not guarantee, because its firing
 *      rate depended on the caller's loop period.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-26
 ******************************************************************************/
TEST(RetryTimerTests, SuppressesUntilIntervalElapses)
{
    // Create a timer with a long interval.
    threadutils::RetryTimer tmTimer(std::chrono::milliseconds(10000));

    // Consume the first-call allowance.
    ASSERT_TRUE(tmTimer.Ready());

    // Hammer it the way a 60 FPS producer loop would; none of these may fire.
    for (int nIter = 0; nIter < 1000; ++nIter)
    {
        // Every subsequent call inside the interval must be suppressed.
        EXPECT_FALSE(tmTimer.Ready());
    }
}

/******************************************************************************
 * @brief Once the interval elapses, exactly one call fires and the next interval is armed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-26
 ******************************************************************************/
TEST(RetryTimerTests, FiresAgainAfterIntervalElapses)
{
    // Create a timer with a short interval so the test stays fast.
    threadutils::RetryTimer tmTimer(std::chrono::milliseconds(50));

    // Consume the first-call allowance.
    ASSERT_TRUE(tmTimer.Ready());
    // Immediately after, the timer must be suppressed.
    EXPECT_FALSE(tmTimer.Ready());

    // Wait out the interval.
    std::this_thread::sleep_for(std::chrono::milliseconds(70));

    // The deadline has passed, so exactly one call fires...
    EXPECT_TRUE(tmTimer.Ready());
    // ...and the next interval is armed again.
    EXPECT_FALSE(tmTimer.Ready());
}

/******************************************************************************
 * @brief Reset() re-arms the timer so the next Ready() fires immediately. Callers use
 *      this after a success, so a later failure retries at once rather than waiting out
 *      a deadline that was armed before the operation started working.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-26
 ******************************************************************************/
TEST(RetryTimerTests, ResetReArmsImmediateFire)
{
    // Create a timer with a long interval.
    threadutils::RetryTimer tmTimer(std::chrono::milliseconds(10000));

    // Consume the first-call allowance and confirm suppression.
    ASSERT_TRUE(tmTimer.Ready());
    ASSERT_FALSE(tmTimer.Ready());

    // Reset as a caller would after the underlying operation succeeded.
    tmTimer.Reset();

    // The next call fires immediately despite the long interval.
    EXPECT_TRUE(tmTimer.Ready());
}

/******************************************************************************
 * @brief Over a fixed span of wall-clock time the timer fires a number of times bounded
 *      by that span divided by the interval, regardless of how fast it is polled. This is
 *      the guarantee the reconnect paths rely on to avoid hammering a dead device.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-26
 ******************************************************************************/
TEST(RetryTimerTests, FireRateIsIndependentOfPollRate)
{
    // Create a timer with a short interval.
    threadutils::RetryTimer tmTimer(std::chrono::milliseconds(25));

    // Count how many times it fires while polling as fast as possible for 200ms.
    int nFireCount                                = 0;
    const std::chrono::steady_clock::time_point tmDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < tmDeadline)
    {
        // Count each allowed attempt.
        if (tmTimer.Ready())
        {
            // Record the fire.
            ++nFireCount;
        }
    }

    // 200ms / 25ms is 8 intervals, plus the immediate first call. Allow slack for scheduling
    // jitter, but the count must be nowhere near the millions of polls performed.
    EXPECT_GE(nFireCount, 4);
    EXPECT_LE(nFireCount, 12);
}

/******************************************************************************
 * @brief The configured interval is reported back unchanged, so callers can log it.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-26
 ******************************************************************************/
TEST(RetryTimerTests, ReportsConfiguredInterval)
{
    // Create a timer with a known interval.
    threadutils::RetryTimer tmTimer(std::chrono::milliseconds(1234));

    // The accessor must return exactly what was configured.
    EXPECT_EQ(tmTimer.GetInterval(), std::chrono::milliseconds(1234));
}
