/******************************************************************************
 * @brief Unit tests for the pubsub::Publisher publish-latest data channel.
 *
 * @file Publisher.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/util/threading/Publisher.hpp"
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
 * @brief Test-local helpers, kept in an anonymous namespace so they do not
 *      collide with symbols in other test translation units.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
namespace
{
    // A payload with a buffer, so a torn read shows up as mixed element values.
    struct Payload
    {
            std::vector<int> vData;
    };
}    // namespace

/******************************************************************************
 * @brief Unit Test Class for the pubsub::Publisher.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
class PublisherTests : public TestingBase<PublisherTests>
{
    public:
        PublisherTests() {}

        ~PublisherTests() {}

        void TestSetup() override {}

        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Get() before anything is published returns null and sequence is zero.
 ******************************************************************************/
TEST_F(PublisherTests, GetBeforePublishIsNull)
{
    pubsub::Publisher<int> pub(4);
    EXPECT_EQ(pub.PeekLatest(), nullptr);
    EXPECT_EQ(pub.GetSequence(), 0ull);
}

/******************************************************************************
 * @brief Publish makes the value visible and stamps sequence/time metadata.
 ******************************************************************************/
TEST_F(PublisherTests, PublishThenGetReturnsValueAndStampsMetadata)
{
    pubsub::Publisher<int> pub(4);
    auto pSlot   = pub.Acquire();
    pSlot->tData = 123;
    pub.Publish(std::move(pSlot));

    auto pSnap = pub.PeekLatest();
    ASSERT_NE(pSnap, nullptr);
    EXPECT_EQ(pSnap->tData, 123);
    EXPECT_EQ(pSnap->ullSequence, 1ull);
    EXPECT_EQ(pub.GetSequence(), 1ull);
    EXPECT_NE(pSnap->tmPublished.time_since_epoch().count(), 0);

    auto pSlot2   = pub.Acquire();
    pSlot2->tData = 456;
    pub.Publish(std::move(pSlot2));
    EXPECT_EQ(pub.PeekLatest()->tData, 456);
    EXPECT_EQ(pub.PeekLatest()->ullSequence, 2ull);
}

/******************************************************************************
 * @brief With one live snapshot at a time and a big enough pool, misses stay 0.
 ******************************************************************************/
TEST_F(PublisherTests, SteadyStateHasZeroPoolMisses)
{
    pubsub::Publisher<int> pub(4);
    for (int nIter = 0; nIter < 1000; ++nIter)
    {
        auto pSlot   = pub.Acquire();
        pSlot->tData = nIter;
        pub.Publish(std::move(pSlot));
    }
    EXPECT_EQ(pub.GetPoolMisses(), 0u);
    EXPECT_EQ(pub.GetPoolAllocated(), 4u);
}

/******************************************************************************
 * @brief Holding more snapshots than preallocated grows the pool, never blocks.
 ******************************************************************************/
TEST_F(PublisherTests, ExhaustionGrowsAndCountsMissesNeverDeadlocks)
{
    pubsub::Publisher<int> pub(2);
    std::vector<std::shared_ptr<pubsub::Snapshot<int>>> vHeld;
    for (int nIter = 0; nIter < 50; ++nIter)
    {
        vHeld.push_back(pub.Acquire());
    }
    EXPECT_GE(pub.GetPoolMisses(), 48u);
    EXPECT_EQ(pub.GetPoolAllocated(), 50u);
    vHeld.clear();
    EXPECT_EQ(pub.GetPoolFreeCount(), 50u);
    EXPECT_FALSE(pub.GetGrowthCeilingBreached());
}

/******************************************************************************
 * @brief Passing a ceiling flags a breach (probable leak) but still grows.
 ******************************************************************************/
TEST_F(PublisherTests, GrowthCeilingBreachIsFlaggedButStillGrows)
{
    pubsub::Publisher<int> pub(2, 4);
    std::vector<std::shared_ptr<pubsub::Snapshot<int>>> vHeld;
    for (int nIter = 0; nIter < 10; ++nIter)
    {
        vHeld.push_back(pub.Acquire());
    }
    EXPECT_EQ(pub.GetPoolAllocated(), 10u);
    EXPECT_TRUE(pub.GetGrowthCeilingBreached());
    vHeld.clear();
}

/******************************************************************************
 * @brief The optional initializer pre-sizes freshly allocated slots.
 ******************************************************************************/
TEST_F(PublisherTests, SlotInitializerPreSizesBuffers)
{
    pubsub::Publisher<Payload> pub(2, 0, [](Payload& tPayload) { tPayload.vData.assign(100, 0); });
    auto pSlot = pub.Acquire();
    EXPECT_EQ(pSlot->tData.vData.size(), 100u);
}

/******************************************************************************
 * @brief The core safety property: a held snapshot is immutable while the
 *      producer churns thousands of recycling publishes underneath it.
 ******************************************************************************/
TEST_F(PublisherTests, HeldSnapshotIsImmutableAcrossManyProducerCycles)
{
    pubsub::Publisher<Payload> pub(4, 0, [](Payload& tPayload) { tPayload.vData.resize(256); });
    auto Publish = [&](int nValue)
    {
        auto pSlot = pub.Acquire();
        pSlot->tData.vData.assign(256, nValue);
        pub.Publish(std::move(pSlot));
    };

    Publish(7);
    auto pHeld = pub.PeekLatest();
    EXPECT_EQ(std::accumulate(pHeld->tData.vData.begin(), pHeld->tData.vData.end(), 0ll), 7ll * 256);

    for (int nIter = 0; nIter < 5000; ++nIter)
    {
        Publish(nIter % 100);
    }

    EXPECT_EQ(std::accumulate(pHeld->tData.vData.begin(), pHeld->tData.vData.end(), 0ll), 7ll * 256);
}

/******************************************************************************
 * @brief A snapshot handed to a consumer stays valid after its Publisher dies.
 ******************************************************************************/
TEST_F(PublisherTests, SnapshotOutlivesPublisherViaWeakPtrDeleter)
{
    pubsub::SharedSnapshot<Payload> pSurvivor;
    {
        pubsub::Publisher<Payload> pub(2);
        auto pSlot = pub.Acquire();
        pSlot->tData.vData.assign(64, 99);
        pub.Publish(std::move(pSlot));
        pSurvivor = pub.PeekLatest();
        ASSERT_NE(pSurvivor, nullptr);
    }
    ASSERT_NE(pSurvivor, nullptr);
    EXPECT_EQ(pSurvivor->tData.vData.size(), 64u);
    EXPECT_EQ(pSurvivor->tData.vData[0], 99);
    pSurvivor.reset();
}

/******************************************************************************
 * @brief HasReaders reflects the number of live Reader handles.
 ******************************************************************************/
TEST_F(PublisherTests, ReaderTracksDemand)
{
    pubsub::Publisher<int> pub(2);
    EXPECT_FALSE(pub.HasReaders());
    {
        auto sub1 = pub.CreateReader();
        EXPECT_TRUE(pub.HasReaders());
        {
            auto sub2 = pub.CreateReader();
            EXPECT_TRUE(pub.HasReaders());
        }
        EXPECT_TRUE(pub.HasReaders());
    }
    EXPECT_FALSE(pub.HasReaders());
}

/******************************************************************************
 * @brief Moving a Reader transfers its demand without dropping it.
 ******************************************************************************/
TEST_F(PublisherTests, ReaderMoveSemantics)
{
    pubsub::Publisher<int> pub(2);
    pubsub::Reader<int> rdOuter;
    EXPECT_FALSE(rdOuter.IsActive());
    {
        auto rdInner = pub.CreateReader();
        EXPECT_TRUE(pub.HasReaders());
        rdOuter = std::move(rdInner);
        EXPECT_TRUE(rdOuter.IsActive());
    }
    EXPECT_TRUE(pub.HasReaders());
    rdOuter.Release();
    EXPECT_FALSE(pub.HasReaders());
}

/******************************************************************************
 * @brief A Reader that outlives its Publisher must stay usable, not dangle. The
 *      channel is kept alive by the Reader, so Get() keeps returning the last value
 *      that was published rather than reading freed memory.
 ******************************************************************************/
TEST_F(PublisherTests, ReaderOutlivingPublisherStaysUsable)
{
    pubsub::Reader<int> rd;
    {
        pubsub::Publisher<int> pub(2);
        rd         = pub.CreateReader();
        auto pSlot = pub.Acquire();
        pSlot->tData = 77;
        pub.Publish(std::move(pSlot));
    }

    // The publisher is gone, but the channel and its last value survive through the Reader.
    auto pSnapshot = rd.Get();
    ASSERT_NE(pSnapshot, nullptr);
    EXPECT_EQ(pSnapshot->tData, 77);

    // Releasing afterwards must not crash either.
    rd.Release();
    EXPECT_EQ(rd.Get(), nullptr);
    SUCCEED();
}

/******************************************************************************
 * @brief A default-constructed Reader is inactive and reads as empty rather than
 *      crashing, so it is safe as a not-yet-assigned member.
 ******************************************************************************/
TEST_F(PublisherTests, InactiveReaderReadsEmpty)
{
    pubsub::Reader<int> rd;
    EXPECT_FALSE(rd.IsActive());
    EXPECT_EQ(rd.Get(), nullptr);
    // Releasing an inactive reader is a no-op.
    rd.Release();
    EXPECT_EQ(rd.Get(), nullptr);
}

/******************************************************************************
 * @brief One producer and several consumers: reads are never torn, growth is
 *      bounded. Best run under ThreadSanitizer.
 ******************************************************************************/
TEST_F(PublisherTests, ConcurrentPublishAndGetNoTears)
{
    pubsub::Publisher<Payload> pub(8, 0, [](Payload& tPayload) { tPayload.vData.resize(512); });
    std::atomic<bool> abStop{false};
    std::atomic<long long> allReads{0};
    std::atomic<long long> allTears{0};

    std::thread thProducer(
        [&]()
        {
            for (int nIter = 1; !abStop.load(); ++nIter)
            {
                auto pSlot = pub.Acquire();
                pSlot->tData.vData.assign(512, nIter);
                pub.Publish(std::move(pSlot));
            }
        });

    std::vector<std::thread> vConsumers;
    for (int nConsumer = 0; nConsumer < 6; ++nConsumer)
    {
        vConsumers.emplace_back(
            [&]()
            {
                while (!abStop.load())
                {
                    auto pSnap = pub.PeekLatest();
                    if (pSnap == nullptr)
                    {
                        continue;
                    }
                    const int nFirst = pSnap->tData.vData.front();
                    for (int nVal : pSnap->tData.vData)
                    {
                        if (nVal != nFirst)
                        {
                            allTears.fetch_add(1);
                            break;
                        }
                    }
                    allReads.fetch_add(1);
                }
            });
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    abStop.store(true);
    thProducer.join();
    for (auto& th : vConsumers)
    {
        th.join();
    }

    EXPECT_GT(allReads.load(), 0);
    EXPECT_EQ(allTears.load(), 0);
    EXPECT_LT(pub.GetPoolAllocated(), 100u);
}

/******************************************************************************
 * @brief A recycled slot still carries the previous publish's data. This is
 *      deliberate - it is what lets copyTo() and std::vector reuse capacity - but it
 *      is the single sharpest edge on this API, so pin it down: a producer that
 *      writes only some fields republishes the rest of the last frame.
 ******************************************************************************/
TEST_F(PublisherTests, AcquireReturnsRecycledSlotStillHoldingPreviousData)
{
    // Enough slots that we recycle rather than allocate fresh ones.
    pubsub::Publisher<Payload> pubChannel(4, 0);

    // Publish twice so the first slot is definitely back on the free list.
    {
        std::shared_ptr<pubsub::Snapshot<Payload>> pSlot = pubChannel.Acquire();
        pSlot->tData.vData                               = {9, 9, 9, 9};
        pubChannel.Publish(std::move(pSlot));
    }
    {
        std::shared_ptr<pubsub::Snapshot<Payload>> pSlot = pubChannel.Acquire();
        pSlot->tData.vData                               = {8, 8, 8, 8};
        pubChannel.Publish(std::move(pSlot));
    }

    // The third Acquire() reuses the first slot, contents and all.
    std::shared_ptr<pubsub::Snapshot<Payload>> pRecycled = pubChannel.Acquire();
    EXPECT_FALSE(pRecycled->tData.vData.empty()) << "Acquire() is documented to hand back a DIRTY slot; if this is ever "
                                                    "empty the pooling contract changed and every publish site needs re-auditing.";
    EXPECT_EQ(pRecycled->tData.vData.size(), 4U);
    EXPECT_EQ(pRecycled->tData.vData.front(), 9);
    // No pool misses: this really was a recycled slot, not a fresh allocation.
    EXPECT_EQ(pubChannel.GetPoolMisses(), 0U);
}

/******************************************************************************
 * @brief WaitForNewer() returns as soon as the producer publishes past the given
 *      sequence, so consumers never have to spin on Get().
 ******************************************************************************/
TEST_F(PublisherTests, WaitForNewerWakesOnPublish)
{
    pubsub::Publisher<int> pubChannel(4, 0);
    pubsub::Reader<int> rdReader = pubChannel.CreateReader();

    // Publish one value so the reader has a sequence to wait past.
    std::shared_ptr<pubsub::Snapshot<int>> pFirst = pubChannel.Acquire();
    pFirst->tData                                 = 1;
    pubChannel.Publish(std::move(pFirst));
    const unsigned long long ullFirstSequence = rdReader.Get()->ullSequence;

    // Publish a second value from another thread after a short delay.
    std::thread thProducer(
        [&pubChannel]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
            std::shared_ptr<pubsub::Snapshot<int>> pSecond = pubChannel.Acquire();
            pSecond->tData                                 = 2;
            pubChannel.Publish(std::move(pSecond));
        });

    // Block for it. This must return the new value, not time out.
    const std::chrono::steady_clock::time_point tmStart = std::chrono::steady_clock::now();
    pubsub::SharedSnapshot<int> pWoken                  = rdReader.WaitForNewer(ullFirstSequence, std::chrono::milliseconds(2000));
    const std::chrono::milliseconds tmElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tmStart);
    thProducer.join();

    ASSERT_NE(pWoken, nullptr);
    EXPECT_EQ(pWoken->tData, 2);
    EXPECT_GT(pWoken->ullSequence, ullFirstSequence);
    // It woke on the publish, not on the timeout.
    EXPECT_LT(tmElapsed.count(), 1000);
}

/******************************************************************************
 * @brief WaitForNewer() returns null (rather than hanging) when nothing new is
 *      published inside the timeout, so a stopped producer cannot pin a consumer's
 *      shutdown open.
 ******************************************************************************/
TEST_F(PublisherTests, WaitForNewerTimesOutWhenProducerIsSilent)
{
    pubsub::Publisher<int> pubChannel(4, 0);
    pubsub::Reader<int> rdReader = pubChannel.CreateReader();

    // Publish once, then go quiet.
    std::shared_ptr<pubsub::Snapshot<int>> pSlot = pubChannel.Acquire();
    pSlot->tData                                 = 7;
    pubChannel.Publish(std::move(pSlot));
    const unsigned long long ullSequence = rdReader.Get()->ullSequence;

    // Waiting past the newest sequence must time out and report nothing.
    EXPECT_EQ(rdReader.WaitForNewer(ullSequence, std::chrono::milliseconds(60)), nullptr);
    // Waiting past an older sequence returns immediately without blocking.
    EXPECT_NE(rdReader.WaitForNewer(ullSequence - 1, std::chrono::milliseconds(60)), nullptr);
}

/******************************************************************************
 * @brief The growth ceiling handler fires on the producer thread at the exact
 *      allocation that breaches it, and exactly once. The old design only set a flag
 *      that a periodic sweep read seconds later, by which point a leaking consumer of
 *      full resolution frames has already allocated gigabytes.
 ******************************************************************************/
TEST_F(PublisherTests, GrowthCeilingHandlerFiresOnceAtTheBreach)
{
    // Prealloc 2, ceiling 4: the 5th live slot is the breach.
    pubsub::Publisher<int> pubChannel(2, 4);

    // Record every breach report.
    std::vector<std::pair<size_t, size_t>> vReports;
    pubChannel.SetGrowthCeilingHandler([&vReports](size_t siAllocated, size_t siCeiling) { vReports.emplace_back(siAllocated, siCeiling); });

    // Hold every slot so the pool is forced to grow.
    std::vector<std::shared_ptr<pubsub::Snapshot<int>>> vHeld;
    for (int nIter = 0; nIter < 8; ++nIter)
    {
        vHeld.push_back(pubChannel.Acquire());
    }

    // Reported exactly once, with the numbers needed to log something actionable.
    ASSERT_EQ(vReports.size(), 1U);
    EXPECT_EQ(vReports.front().first, 5U);
    EXPECT_EQ(vReports.front().second, 4U);
    // The flag is still raised for the periodic sweep, and the pool still grew.
    EXPECT_TRUE(pubChannel.GetGrowthCeilingBreached());
    EXPECT_EQ(pubChannel.GetPoolAllocated(), 8U);
}
