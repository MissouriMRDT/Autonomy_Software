/******************************************************************************
 * @brief Defines and implements the pubsub::Publisher class and its supporting
 *      pubsub::Snapshot and pubsub::Subscription types. Together these implement
 *      a lock-free "publish-latest" data channel: a producer thread copies each
 *      new value into a pooled, reference-counted, immutable snapshot and stores
 *      it atomically; any number of consumer threads read the newest snapshot
 *      without blocking the producer and without blocking each other.
 *
 *      This single primitive replaces the per-consumer request/queue/promise
 *      fan-out that the camera and detector classes previously reimplemented.
 *      It is intentionally dependency-free (standard library only) so it can be
 *      unit tested in isolation and reused anywhere.
 *
 * @file Publisher.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

/// \cond
#include <atomic>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <version>

/// \endcond

// Decide exactly once, here, whether the toolchain provides a real
// std::atomic<std::shared_ptr<T>> specialization (C++20 libraries). If it does
// we use it; otherwise we fall back to the pre-C++20 free-function atomics on a
// plain shared_ptr member. Those free functions are deprecated in C++20 and
// removed in C++26, but they are the only portable option on libstdc++ < 12
// (the GCC 10.x toolchain this project currently builds with). No other file in
// the codebase needs to care which path is active.
#if defined(__cpp_lib_atomic_shared_ptr) && (__cpp_lib_atomic_shared_ptr >= 201711L)
#define PUBSUB_HAS_ATOMIC_SHARED_PTR 1
#else
#define PUBSUB_HAS_ATOMIC_SHARED_PTR 0
#endif

/******************************************************************************
 * @brief Namespace containing the publish-latest data channel primitives used
 *      to hand immutable snapshots of a value from one producer thread to many
 *      consumer threads without locking or blocking.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
namespace pubsub
{
    /******************************************************************************
     * @brief An immutable, reference-counted snapshot of one published value.
     *      The sequence number and publish time travel inside the object so they
     *      are always atomic with the data itself. Consumers use the sequence to
     *      detect staleness and to skip redundant work when the value has not
     *      changed.
     *
     * @tparam T - The type of value being published.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2026-07-24
     ******************************************************************************/
    template<typename T>
    struct Snapshot
    {
        public:
            // Declare and define public struct member variables.
            T tData;                                             // The published value. Written by the producer before Publish().
            unsigned long long ullSequence = 0;                  // Monotonic publish sequence number. Stamped by Publish().
            std::chrono::system_clock::time_point tmPublished;    // Wall-clock time the value was published. Stamped by Publish().
    };

    /******************************************************************************
     * @brief A move-only RAII handle that expresses a consumer's demand for a
     *      Publisher's data. Construction increments the owning Publisher's
     *      subscriber count; destruction decrements it. The producer retrieves
     *      and publishes a data type only while at least one Subscription for it
     *      is alive, so demand tracks reality instead of a wall clock.
     *
     *      The count is held through a shared_ptr so a Subscription that outlives
     *      its Publisher decrements a still-valid (if now unread) counter rather
     *      than touching freed memory.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2026-07-24
     ******************************************************************************/
    class Subscription
    {
        public:
            /******************************************************************************
             * @brief Construct a new, inactive Subscription. Holds no demand.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            Subscription() = default;

            /******************************************************************************
             * @brief Construct a new active Subscription and register one unit of demand.
             *
             * @param pSubscriberCount - The shared subscriber counter to increment.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            explicit Subscription(std::shared_ptr<std::atomic<long>> pSubscriberCount) : m_pSubscriberCount(std::move(pSubscriberCount))
            {
                // Register demand if the counter is valid.
                if (m_pSubscriberCount != nullptr)
                {
                    // Increment the subscriber count.
                    m_pSubscriberCount->fetch_add(1, std::memory_order_acq_rel);
                }
            }

            /******************************************************************************
             * @brief Destroy the Subscription object and release its demand.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            ~Subscription() { this->Release(); }

            /******************************************************************************
             * @brief Move construct a Subscription, transferring its demand.
             *
             * @param stOther - The Subscription to move from.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            Subscription(Subscription&& stOther) noexcept : m_pSubscriberCount(std::move(stOther.m_pSubscriberCount)) { stOther.m_pSubscriberCount = nullptr; }

            /******************************************************************************
             * @brief Move assign a Subscription, releasing our demand and taking theirs.
             *
             * @param stOther - The Subscription to move from.
             * @return Subscription& - A reference to this object.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            Subscription& operator=(Subscription&& stOther) noexcept
            {
                // Guard against self-assignment.
                if (this != &stOther)
                {
                    // Release our current demand before taking the other's.
                    this->Release();
                    // Transfer ownership of the counter.
                    m_pSubscriberCount        = std::move(stOther.m_pSubscriberCount);
                    stOther.m_pSubscriberCount = nullptr;
                }

                // Return a reference to this object.
                return *this;
            }

            // A Subscription represents unique demand and must not be copied.
            Subscription(const Subscription&)            = delete;
            Subscription& operator=(const Subscription&) = delete;

            /******************************************************************************
             * @brief Check whether this Subscription currently holds demand.
             *
             * @return true - This Subscription is active and counted.
             * @return false - This Subscription is inactive/empty.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            bool IsActive() const { return m_pSubscriberCount != nullptr; }

            /******************************************************************************
             * @brief Release this Subscription's demand early, before destruction.
             *      Safe to call multiple times.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            void Release()
            {
                // Only decrement if we currently hold demand.
                if (m_pSubscriberCount != nullptr)
                {
                    // Decrement the subscriber count.
                    m_pSubscriberCount->fetch_sub(1, std::memory_order_acq_rel);
                    // Drop the counter so we never decrement twice.
                    m_pSubscriberCount = nullptr;
                }
            }

        private:
            // Declare private member variables.
            std::shared_ptr<std::atomic<long>> m_pSubscriberCount;    // Shared demand counter. Null when inactive.
    };

    /******************************************************************************
     * @brief A publish-latest data channel for a single value type T.
     *
     *      Producer side: Acquire() hands out a pooled, default-constructed
     *      snapshot slot (recycled buffers, never blocks, grows if the pool is
     *      empty). The producer writes into the slot's tData and calls Publish(),
     *      which stamps the sequence and time and atomically stores the slot as
     *      the newest snapshot.
     *
     *      Consumer side: Get() returns the newest snapshot with a lock-free
     *      atomic load (may be null if nothing has been published yet). The
     *      returned shared_ptr keeps that snapshot's buffer alive for as long as
     *      the consumer holds it, so a slow reader can never be torn by the
     *      producer recycling the buffer underneath it.
     *
     * @tparam T - The value type to publish. Must be default-constructible.
     *
     * @note This class is neither copyable nor movable; hold it as a member and
     *      hand out references via an accessor.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2026-07-24
     ******************************************************************************/
    template<typename T>
    class Publisher
    {
        public:
            // Public type alias for the immutable snapshot handle consumers receive.
            using SharedSnapshot = std::shared_ptr<const Snapshot<T>>;

            /******************************************************************************
             * @brief Construct a new Publisher object.
             *
             * @param siPrealloc - Number of snapshot slots to pre-allocate so steady
             *                  state performs no allocation. A snapshot is live only
             *                  while a consumer is mid-read, so a good value is
             *                  (2 + expected concurrent readers + slack).
             * @param siGrowthCeiling - Soft cap on total slots allocated. 0 means
             *                  unlimited. When exceeded the pool still grows (the
             *                  producer never blocks) but a breach flag is raised so a
             *                  genuine snapshot leak surfaces as an error, not an OOM.
             * @param fnSlotInitializer - Optional callback invoked on each freshly
             *                  allocated slot's tData so buffers can be sized up front
             *                  and even the first frames avoid page faults.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            explicit Publisher(const size_t siPrealloc = 4, const size_t siGrowthCeiling = 0, std::function<void(T&)> fnSlotInitializer = nullptr)
            {
                // Allocate the shared slot pool and the shared subscriber counter.
                m_pPool            = std::make_shared<Pool>();
                m_pSubscriberCount = std::make_shared<std::atomic<long>>(0);

                // Store pool configuration.
                m_pPool->siGrowthCeiling   = siGrowthCeiling;
                m_pPool->fnSlotInitializer = std::move(fnSlotInitializer);

                // Pre-allocate the requested number of slots so steady state is allocation free.
                std::lock_guard<std::mutex> lkFreeList(m_pPool->muFreeList);
                for (size_t siIter = 0; siIter < siPrealloc; ++siIter)
                {
                    // Allocate a slot, run the initializer, and place it on the free list.
                    m_pPool->vFreeList.push_back(m_pPool->AllocateSlotLocked());
                }
            }

            /******************************************************************************
             * @brief Destroy the Publisher object. Slots still held by consumers stay
             *      alive; their custom deleters see the pool is gone (via weak_ptr) and
             *      simply delete them, so there is no use-after-free.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            ~Publisher() = default;

            // A Publisher owns atomic state and must not be copied or moved.
            Publisher(const Publisher&)            = delete;
            Publisher& operator=(const Publisher&) = delete;
            Publisher(Publisher&&)                 = delete;
            Publisher& operator=(Publisher&&)      = delete;

            /******************************************************************************
             * @brief Acquire a pooled snapshot slot for the producer to write into.
             *      Returns a recycled slot from the free list, or allocates a new one
             *      if the free list is empty. NEVER blocks and never waits on a
             *      consumer. When the last holder of the returned shared_ptr drops it,
             *      the slot is returned to the free list instead of being freed.
             *
             * @return std::shared_ptr<Snapshot<T>> - A writable, pooled snapshot slot.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            std::shared_ptr<Snapshot<T>> Acquire()
            {
                // The raw slot pointer we will hand out.
                Snapshot<T>* pSlot = nullptr;

                // Try to pop a recycled slot off the free list.
                {
                    // Lock the free list only long enough to pop.
                    std::lock_guard<std::mutex> lkFreeList(m_pPool->muFreeList);
                    if (!m_pPool->vFreeList.empty())
                    {
                        // Reuse the most recently returned slot (warmest in cache).
                        pSlot = m_pPool->vFreeList.back();
                        m_pPool->vFreeList.pop_back();
                    }
                    else
                    {
                        // Free list is empty: allocate a fresh slot and count the miss.
                        pSlot = m_pPool->AllocateSlotLocked();
                        m_pPool->siPoolMisses.fetch_add(1, std::memory_order_relaxed);
                    }
                }

                // Wrap the slot in a shared_ptr whose deleter recycles it into the pool.
                // The deleter captures a weak_ptr, never a raw this, so a consumer that
                // outlives the Publisher deletes the slot instead of dereferencing a
                // dangling pool. The deleter is noexcept.
                std::weak_ptr<Pool> wpPool = m_pPool;
                return std::shared_ptr<Snapshot<T>>(pSlot,
                                                    [wpPool](Snapshot<T>* pReturned) noexcept
                                                    {
                                                        // If the pool still exists, return the slot to it; otherwise delete.
                                                        if (std::shared_ptr<Pool> pLockedPool = wpPool.lock())
                                                        {
                                                            // Recycle the slot for reuse.
                                                            pLockedPool->Return(pReturned);
                                                        }
                                                        else
                                                        {
                                                            // Pool is gone; free the slot outright.
                                                            delete pReturned;
                                                        }
                                                    });
            }

            /******************************************************************************
             * @brief Publish a slot as the newest snapshot. Stamps the sequence number
             *      and publish time, then atomically stores the snapshot with release
             *      ordering so consumers see the data and its metadata together.
             *
             * @param pSnapshot - The slot (from Acquire()) whose tData has been written.
             *
             * @note Never publish a slot that has already been published. Acquire a
             *      fresh slot per publish.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            void Publish(std::shared_ptr<Snapshot<T>> pSnapshot)
            {
                // Ignore a null publish rather than crashing.
                if (pSnapshot == nullptr)
                {
                    // Nothing to publish.
                    return;
                }

                // Stamp the sequence number and publish time inside the snapshot so they
                // are atomic with the data.
                pSnapshot->ullSequence  = m_ullSequence.fetch_add(1, std::memory_order_relaxed) + 1;
                pSnapshot->tmPublished  = std::chrono::system_clock::now();

                // Atomically store the (now immutable) snapshot as the newest value.
                this->StoreLatest(SharedSnapshot(std::move(pSnapshot)));
            }

            /******************************************************************************
             * @brief Get the newest published snapshot with a lock-free atomic load.
             *
             * @return SharedSnapshot - The newest immutable snapshot, or nullptr if
             *                  nothing has been published yet.
             *
             * @note Load once into a local and work from that local. Calling Get()
             *      repeatedly returns whatever is newest each time, which is not a
             *      stable value.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            SharedSnapshot Get() const { return this->LoadLatest(); }

            /******************************************************************************
             * @brief Register demand for this Publisher's data. Hold the returned
             *      handle for as long as you want the producer to keep publishing.
             *
             * @return Subscription - A move-only RAII demand handle.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            Subscription Subscribe() { return Subscription(m_pSubscriberCount); }

            /******************************************************************************
             * @brief Check whether any consumer currently has demand for this data.
             *
             * @return true - At least one Subscription is alive.
             * @return false - No consumer is subscribed.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            bool HasSubscribers() const { return m_pSubscriberCount->load(std::memory_order_acquire) > 0; }

            /******************************************************************************
             * @brief Accessor for the number of pool misses (slots allocated because
             *      the free list was empty). Steady-state value should be flat after
             *      warmup; a steadily rising value indicates a snapshot leak.
             *
             * @return size_t - The cumulative number of pool misses.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            size_t GetPoolMisses() const { return m_pPool->siPoolMisses.load(std::memory_order_relaxed); }

            /******************************************************************************
             * @brief Accessor for the current publish sequence number (the sequence of
             *      the most recently published snapshot, or 0 if none).
             *
             * @return unsigned long long - The newest published sequence number.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            unsigned long long GetSequence() const { return m_ullSequence.load(std::memory_order_relaxed); }

            /******************************************************************************
             * @brief Accessor for the total number of slots this pool has ever
             *      allocated (free list size plus slots currently in use).
             *
             * @return size_t - The total number of allocated slots.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            size_t GetPoolAllocated() const { return m_pPool->siAllocated.load(std::memory_order_relaxed); }

            /******************************************************************************
             * @brief Accessor for the number of slots currently sitting in the free
             *      list (allocated but not in use).
             *
             * @return size_t - The current free list size.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            size_t GetPoolFreeCount() const
            {
                // Lock the free list to read its size consistently.
                std::lock_guard<std::mutex> lkFreeList(m_pPool->muFreeList);
                return m_pPool->vFreeList.size();
            }

            /******************************************************************************
             * @brief Accessor for whether the pool's growth ceiling has ever been
             *      breached. A breach means more slots are live than expected, which
             *      almost always means a consumer is leaking snapshots.
             *
             * @return true - The growth ceiling was breached at least once.
             * @return false - The pool has stayed within its ceiling.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            bool GetGrowthCeilingBreached() const { return m_pPool->bCeilingBreached.load(std::memory_order_relaxed); }

        private:
            /******************************************************************************
             * @brief The internal slot pool shared (via shared_ptr) between the
             *      Publisher and every outstanding snapshot's recycling deleter. Held by
             *      shared_ptr so the deleters can weak_ptr to it and outlive the
             *      Publisher safely.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            struct Pool
            {
                public:
                    std::mutex muFreeList;                          // Guards the free list.
                    std::vector<Snapshot<T>*> vFreeList;            // Recycled slots not currently in use. Owns them.
                    std::atomic<size_t> siPoolMisses{0};           // Slots allocated because the free list was empty.
                    std::atomic<size_t> siAllocated{0};            // Total slots ever allocated (free + in use).
                    std::atomic<bool> bCeilingBreached{false};     // Set once total allocation passes the ceiling.
                    size_t siGrowthCeiling = 0;                    // Soft cap on total allocation (0 = unlimited).
                    std::function<void(T&)> fnSlotInitializer;     // Optional per-slot data initializer.

                    /******************************************************************************
                     * @brief Destroy the Pool object, freeing every slot still on the free
                     *      list. Slots currently in use are not here; their deleters free them.
                     *
                     * @author clayjay3 (claytonraycowen@gmail.com)
                     * @date 2026-07-24
                     ******************************************************************************/
                    ~Pool()
                    {
                        // Free every slot remaining on the free list.
                        for (Snapshot<T>* pSlot : vFreeList)
                        {
                            // Delete the recycled slot.
                            delete pSlot;
                        }
                        // Clear the free list.
                        vFreeList.clear();
                    }

                    /******************************************************************************
                     * @brief Allocate one fresh slot, run the optional initializer on it, and
                     *      account for it. Must be called with muFreeList held (or during
                     *      construction) because it touches siAllocated/bCeilingBreached and the
                     *      initializer.
                     *
                     * @return Snapshot<T>* - The newly allocated slot.
                     *
                     * @author clayjay3 (claytonraycowen@gmail.com)
                     * @date 2026-07-24
                     ******************************************************************************/
                    Snapshot<T>* AllocateSlotLocked()
                    {
                        // Allocate a default-constructed slot.
                        Snapshot<T>* pSlot = new Snapshot<T>();
                        // Count it toward total allocation.
                        const size_t siNowAllocated = siAllocated.fetch_add(1, std::memory_order_relaxed) + 1;
                        // Raise the breach flag if we have grown past the soft ceiling.
                        if (siGrowthCeiling > 0 && siNowAllocated > siGrowthCeiling)
                        {
                            // Surface the (probable) leak to diagnostics without blocking.
                            bCeilingBreached.store(true, std::memory_order_relaxed);
                        }
                        // Pre-size the slot's data if an initializer was provided.
                        if (fnSlotInitializer)
                        {
                            // Run the initializer on the fresh slot's data.
                            fnSlotInitializer(pSlot->tData);
                        }
                        // Return the ready slot.
                        return pSlot;
                    }

                    /******************************************************************************
                     * @brief Return an in-use slot to the free list for reuse. Called from a
                     *      snapshot's deleter, so it must be noexcept: if pushing onto the free
                     *      list throws (out of memory), the slot is deleted instead of leaked.
                     *
                     * @param pSlot - The slot to recycle.
                     *
                     * @author clayjay3 (claytonraycowen@gmail.com)
                     * @date 2026-07-24
                     ******************************************************************************/
                    void Return(Snapshot<T>* pSlot) noexcept
                    {
                        try
                        {
                            // Lock the free list and push the slot back for reuse.
                            std::lock_guard<std::mutex> lkFreeList(muFreeList);
                            vFreeList.push_back(pSlot);
                        }
                        catch (...)
                        {
                            // Could not recycle (allocation failure); free the slot to stay leak free.
                            delete pSlot;
                            siAllocated.fetch_sub(1, std::memory_order_relaxed);
                        }
                    }
            };

            /////////////////////////////////////////
            // Atomic newest-snapshot storage. The member type and the load/store
            // helpers are selected once, at compile time, based on library support.
            /////////////////////////////////////////

#if PUBSUB_HAS_ATOMIC_SHARED_PTR
            // Modern path: a real atomic shared_ptr.
            std::atomic<SharedSnapshot> m_atomLatest;

            /******************************************************************************
             * @brief Atomically load the newest snapshot with acquire ordering.
             ******************************************************************************/
            SharedSnapshot LoadLatest() const { return m_atomLatest.load(std::memory_order_acquire); }

            /******************************************************************************
             * @brief Atomically store the newest snapshot with release ordering.
             ******************************************************************************/
            void StoreLatest(SharedSnapshot pSnapshot) { m_atomLatest.store(std::move(pSnapshot), std::memory_order_release); }
#else
            // Fallback path: a plain shared_ptr accessed through the deprecated
            // free-function atomics (the only option on libstdc++ < 12).
            SharedSnapshot m_pLatest;

            /******************************************************************************
             * @brief Atomically load the newest snapshot with acquire ordering.
             ******************************************************************************/
            SharedSnapshot LoadLatest() const { return std::atomic_load_explicit(&m_pLatest, std::memory_order_acquire); }

            /******************************************************************************
             * @brief Atomically store the newest snapshot with release ordering.
             ******************************************************************************/
            void StoreLatest(SharedSnapshot pSnapshot) { std::atomic_store_explicit(&m_pLatest, std::move(pSnapshot), std::memory_order_release); }
#endif

            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////

            std::shared_ptr<Pool> m_pPool;                          // The slot pool, shared with snapshot deleters via weak_ptr.
            std::shared_ptr<std::atomic<long>> m_pSubscriberCount;    // Demand counter, shared with Subscription handles.
            std::atomic<unsigned long long> m_ullSequence{0};       // Monotonic publish sequence source.
    };
}    // namespace pubsub

#endif    // PUBLISHER_HPP
