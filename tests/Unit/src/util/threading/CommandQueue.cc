/******************************************************************************
 * @brief Unit tests for the CommandQueue control channel.
 *
 * @file CommandQueue.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/util/threading/CommandQueue.hpp"
#include "../../../../TestingBase.hh"

/// \cond
#include <atomic>
#include <chrono>
#include <gtest/gtest.h>
#include <numeric>
#include <thread>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the CommandQueue.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
class CommandQueueTests : public TestingBase<CommandQueueTests>
{
    public:
        CommandQueueTests() {}

        ~CommandQueueTests() {}

        void TestSetup() override {}

        void TestTeardown() override {}
};

/******************************************************************************
 * @brief A posted command runs only when the owner drains, on the owner thread.
 ******************************************************************************/
TEST_F(CommandQueueTests, PostAndDrainRunsCommand)
{
    CommandQueue cq;
    int nRan = 0;
    cq.Post([&]() { nRan = 42; });
    EXPECT_EQ(nRan, 0);
    EXPECT_EQ(cq.GetPendingCount(), 1u);
    cq.DrainAll();
    EXPECT_EQ(nRan, 42);
    EXPECT_EQ(cq.GetPendingCount(), 0u);
}

/******************************************************************************
 * @brief PostWithResult delivers the command's return value via a future.
 ******************************************************************************/
TEST_F(CommandQueueTests, PostWithResultReturnsValue)
{
    CommandQueue cq;
    std::future<int> fu = cq.PostWithResult<int>([]() { return 7; });
    cq.DrainAll();
    EXPECT_EQ(fu.get(), 7);
}

/******************************************************************************
 * @brief PostWithResult<void> completes its future when drained.
 ******************************************************************************/
TEST_F(CommandQueueTests, PostWithResultVoid)
{
    CommandQueue cq;
    bool bRan            = false;
    std::future<void> fu = cq.PostWithResult<void>([&]() { bRan = true; });
    cq.DrainAll();
    fu.get();
    EXPECT_TRUE(bRan);
}

/******************************************************************************
 * @brief An exception thrown by a result command is delivered to its future.
 ******************************************************************************/
TEST_F(CommandQueueTests, CommandExceptionRoutedToFuture)
{
    CommandQueue cq;
    std::future<int> fu = cq.PostWithResult<int>([]() -> int { throw std::runtime_error("boom"); });
    cq.DrainAll();
    EXPECT_THROW(fu.get(), std::runtime_error);
}

/******************************************************************************
 * @brief A throwing fire-and-forget command is reported and swallowed, and the
 *      loop continues to the next command.
 ******************************************************************************/
TEST_F(CommandQueueTests, FireAndForgetExceptionSwallowedAndReported)
{
    CommandQueue cq;
    std::atomic<int> nHandled{0};
    cq.SetExceptionHandler([&](std::exception_ptr) { nHandled.fetch_add(1); });
    bool bSecondRan = false;
    cq.Post([]() { throw std::runtime_error("bad"); });
    cq.Post([&]() { bSecondRan = true; });
    cq.DrainAll();
    EXPECT_EQ(nHandled.load(), 1);
    EXPECT_TRUE(bSecondRan);
}

/******************************************************************************
 * @brief Commands execute in the order posted.
 ******************************************************************************/
TEST_F(CommandQueueTests, CommandsRunInFifoOrder)
{
    CommandQueue cq;
    std::vector<int> vOrder;
    for (int nIter = 0; nIter < 10; ++nIter)
    {
        cq.Post([&, nIter]() { vOrder.push_back(nIter); });
    }
    cq.DrainAll();
    std::vector<int> vExpected(10);
    std::iota(vExpected.begin(), vExpected.end(), 0);
    EXPECT_EQ(vOrder, vExpected);
}

/******************************************************************************
 * @brief Shutting down cancels pending result futures instead of stranding them.
 ******************************************************************************/
TEST_F(CommandQueueTests, ShutdownCancelsPendingFuturesNoHang)
{
    CommandQueue cq;
    std::future<int> fu = cq.PostWithResult<int>([]() { return 1; });
    cq.Shutdown();
    EXPECT_THROW(fu.get(), std::runtime_error);
}

/******************************************************************************
 * @brief After shutdown, new posts are cancelled/dropped.
 ******************************************************************************/
TEST_F(CommandQueueTests, PostAfterShutdownIsCancelled)
{
    CommandQueue cq;
    cq.Shutdown();
    std::future<int> fu = cq.PostWithResult<int>([]() { return 1; });
    EXPECT_THROW(fu.get(), std::runtime_error);
    int nRan = 0;
    cq.Post([&]() { nRan = 5; });
    cq.DrainAll();
    EXPECT_EQ(nRan, 0);
}

/******************************************************************************
 * @brief Destroying the queue with pending work cancels it (no broken promise).
 ******************************************************************************/
TEST_F(CommandQueueTests, DestructorCancelsPendingNoStrand)
{
    std::future<int> fu;
    {
        CommandQueue cq;
        fu = cq.PostWithResult<int>([]() { return 1; });
    }
    EXPECT_THROW(fu.get(), std::runtime_error);
}

/******************************************************************************
 * @brief Commands run on the draining thread, not the posting thread.
 ******************************************************************************/
TEST_F(CommandQueueTests, CommandsRunOnDrainingThread)
{
    CommandQueue cq;
    std::thread::id idPoster = std::this_thread::get_id();
    std::thread::id idRunner;
    std::thread thOwner(
        [&]()
        {
            while (cq.GetPendingCount() == 0)
            {
                std::this_thread::yield();
            }
            cq.DrainAll();
        });
    cq.Post([&]() { idRunner = std::this_thread::get_id(); });
    thOwner.join();
    EXPECT_NE(idRunner, idPoster);
}

/******************************************************************************
 * @brief Many concurrent posters and one drainer: every future resolves. Best
 *      run under ThreadSanitizer.
 ******************************************************************************/
TEST_F(CommandQueueTests, ConcurrentPostersSingleDrainerDeliversAll)
{
    CommandQueue cq;
    std::atomic<bool> abStop{false};
    std::atomic<int> nExecuted{0};
    constexpr int nPosters = 8;
    constexpr int nEach    = 400;

    std::thread thDrainer(
        [&]()
        {
            while (!abStop.load())
            {
                cq.DrainAll();
                std::this_thread::yield();
            }
            cq.DrainAll();
        });

    std::vector<std::thread> vPosters;
    std::vector<std::future<int>> vFutures;
    std::mutex muFutures;
    for (int nP = 0; nP < nPosters; ++nP)
    {
        vPosters.emplace_back(
            [&]()
            {
                for (int nIter = 0; nIter < nEach; ++nIter)
                {
                    auto fu = cq.PostWithResult<int>(
                        [&]() -> int
                        {
                            nExecuted.fetch_add(1);
                            return 1;
                        });
                    std::lock_guard<std::mutex> lk(muFutures);
                    vFutures.push_back(std::move(fu));
                }
            });
    }
    for (auto& th : vPosters)
    {
        th.join();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    abStop.store(true);
    thDrainer.join();

    int nResolved = 0;
    for (auto& fu : vFutures)
    {
        EXPECT_EQ(fu.get(), 1);
        ++nResolved;
    }
    EXPECT_EQ(nResolved, nPosters * nEach);
    EXPECT_EQ(nExecuted.load(), nPosters * nEach);
}

/******************************************************************************
 * @brief With a liveness predicate reporting a dead owner, a result-bearing command
 *      is cancelled at post time instead of sitting in the queue forever. This is the
 *      regression test for the hang that occurred when a camera thread self-stopped
 *      (camera not present) and a foreign thread then posted a control command.
 ******************************************************************************/
TEST_F(CommandQueueTests, PostWhenDrainerDeadIsCancelledNotStranded)
{
    CommandQueue cq;
    std::atomic<bool> bDrainerLive{false};
    cq.SetDrainerLivenessCheck([&]() { return bDrainerLive.load(); });

    // The owner is not running, so this must not block and must not enqueue.
    const std::chrono::steady_clock::time_point tmStart = std::chrono::steady_clock::now();
    std::future<int> fuResult                           = cq.PostAndWait<int>([]() { return 7; });
    const long long llElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tmStart).count();

    EXPECT_LT(llElapsedMs, 1000);
    EXPECT_EQ(cq.GetPendingCount(), 0u);
    EXPECT_THROW(fuResult.get(), std::runtime_error);
}

/******************************************************************************
 * @brief A caller already blocked in PostAndWait() is released when the owning
 *      thread dies mid-wait, rather than blocking forever.
 ******************************************************************************/
TEST_F(CommandQueueTests, BlockedCallerReleasedWhenDrainerDiesMidWait)
{
    CommandQueue cq;
    std::atomic<bool> bDrainerLive{true};
    std::atomic<bool> bCallerWaiting{false};
    std::atomic<bool> bCallerDone{false};
    cq.SetDrainerLivenessCheck([&]() { return bDrainerLive.load(); });

    // Block a caller on a command that will never be drained.
    std::thread thCaller(
        [&]()
        {
            bCallerWaiting = true;
            std::future<int> fuResult = cq.PostAndWait<int>([]() { return 1; });
            EXPECT_THROW(fuResult.get(), std::runtime_error);
            bCallerDone = true;
        });

    // Wait until the caller is genuinely blocked, then kill the owner.
    while (!bCallerWaiting.load())
    {
        std::this_thread::yield();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_FALSE(bCallerDone.load());
    bDrainerLive = false;

    thCaller.join();
    EXPECT_TRUE(bCallerDone.load());
}

/******************************************************************************
 * @brief A live owner still executes everything normally; the liveness gate must
 *      not cancel commands spuriously.
 ******************************************************************************/
TEST_F(CommandQueueTests, LiveDrainerStillExecutesEveryCommand)
{
    CommandQueue cq;
    std::atomic<bool> bDrainerLive{true};
    std::atomic<bool> bStopOwner{false};
    cq.SetDrainerLivenessCheck([&]() { return bDrainerLive.load(); });

    // Run an owner thread that drains continuously.
    std::thread thOwner(
        [&]()
        {
            while (!bStopOwner.load())
            {
                cq.DrainAll();
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        });

    // Round-trip a batch of commands through the owner.
    int nSum = 0;
    for (int nIter = 0; nIter < 100; ++nIter)
    {
        nSum += cq.PostAndWait<int>([nIter]() { return nIter; }).get();
    }

    bStopOwner = true;
    thOwner.join();
    EXPECT_EQ(nSum, 4950);
}

/******************************************************************************
 * @brief Fire-and-forget posts are dropped (not queued) when the owner is dead.
 ******************************************************************************/
TEST_F(CommandQueueTests, FireAndForgetDroppedWhenDrainerDead)
{
    CommandQueue cq;
    cq.SetDrainerLivenessCheck([]() { return false; });

    bool bRan = false;
    cq.Post([&]() { bRan = true; });

    EXPECT_EQ(cq.GetPendingCount(), 0u);
    cq.DrainAll();
    EXPECT_FALSE(bRan);
}

/******************************************************************************
 * @brief CancelPending() releases waiting callers without permanently shutting the
 *      queue down, so an owner that restarts can still accept new commands.
 ******************************************************************************/
TEST_F(CommandQueueTests, CancelPendingDoesNotPermanentlyShutDown)
{
    CommandQueue cq;
    std::future<int> fuCancelled = cq.PostWithResult<int>([]() { return 1; });

    cq.CancelPending();
    EXPECT_THROW(fuCancelled.get(), std::runtime_error);
    EXPECT_FALSE(cq.IsShutdown());

    // The queue is still usable afterwards.
    std::future<int> fuLater = cq.PostWithResult<int>([]() { return 5; });
    cq.DrainAll();
    EXPECT_EQ(fuLater.get(), 5);
}
