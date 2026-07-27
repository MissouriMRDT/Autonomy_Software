/******************************************************************************
 * @brief Defines and implements the publish-latest data channel: pubsub::Snapshot,
 *      pubsub::Publisher (the write face) and pubsub::Reader (the read face).
 *
 *      A producer thread copies each new value into a pooled, reference-counted,
 *      immutable snapshot and stores it atomically; any number of consumer threads read
 *      the newest snapshot without blocking the producer and without blocking each other.
 *      This single primitive replaces the per-consumer request/queue/promise fan-out that
 *      the camera and detector classes previously reimplemented.
 *
 *      The write and read faces are deliberately separate types over one shared channel:
 *
 *        Publisher<T>  Acquire() / Publish() / HasSubscribers(). Held privately by the
 *                      producer that owns the data. Never handed out.
 *        Reader<T>     Get(), plus RAII demand. This is what producers hand to consumers.
 *
 *      That split buys two invariants that documentation alone cannot:
 *
 *        1. A consumer physically cannot publish into a channel it only reads. Handing out
 *           the Publisher would let any consumer inject fabricated values into every other
 *           consumer of that channel.
 *        2. A consumer physically cannot read without registering demand, because Get()
 *           lives on the same object that holds the demand count. Reads that "work" only
 *           because some unrelated consumer happened to be subscribed are unrepresentable.
 *
 *      A Reader owns a strong reference to the channel, so it stays valid even if the
 *      producer that created it is destroyed first; Get() simply keeps returning the last
 *      published value (or nullptr).
 *
 *      It is intentionally dependency-free (standard library plus Tracy) so it can be unit
 *      tested in isolation and reused anywhere.
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
#include <tracy/Tracy.hpp>
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
     * @brief Namespace containing implementation details of the publish-latest
     *      channel. Nothing in here is part of the public interface.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2026-07-26
     ******************************************************************************/
    namespace internal
    {
        /******************************************************************************
         * @brief The shared control block for one publish-latest channel. Owns the slot
         *      pool, the newest published snapshot, the demand count and the sequence
         *      source. Held through a shared_ptr by the Publisher, by every Reader, and
         *      (weakly) by every outstanding snapshot's recycling deleter.
         *
         *      Holding this separately from the Publisher is what lets a Reader outlive
         *      the producer that created it: the channel survives as long as anything
         *      still references it, so a late Get() is well defined rather than a
         *      dangling read.
         *
         * @tparam T - The value type carried by this channel.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-26
         ******************************************************************************/
        template<typename T>
        class Channel
        {
            public:
                // Public type alias for the immutable snapshot handle consumers receive.
                using SharedSnapshot = std::shared_ptr<const Snapshot<T>>;

                /////////////////////////////////////////
                // Declare public member variables.
                /////////////////////////////////////////

                std::mutex muFreeList;                             // Guards the free list.
                std::vector<Snapshot<T>*> vFreeList;               // Recycled slots not currently in use. Owns them.
                std::atomic<size_t> siPoolMisses{0};               // Slots allocated because the free list was empty.
                std::atomic<size_t> siAllocated{0};                // Total slots ever allocated (free + in use).
                std::atomic<bool> bCeilingBreached{false};         // Set once total allocation passes the ceiling.
                std::atomic<long> nSubscribers{0};                 // Number of live Readers expressing demand.
                std::atomic<unsigned long long> ullSequence{0};    // Monotonic publish sequence source.
                size_t siGrowthCeiling = 0;                        // Soft cap on total allocation (0 = unlimited).
                std::function<void(T&)> fnSlotInitializer;         // Optional per-slot data initializer.

                /******************************************************************************
                 * @brief Destroy the Channel object, freeing every slot still on the free
                 *      list. Slots currently in use are not here; their deleters free them.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2026-07-24
                 ******************************************************************************/
                ~Channel()
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

#if PUBSUB_HAS_ATOMIC_SHARED_PTR
                // Modern path: a real atomic shared_ptr.
                std::atomic<SharedSnapshot> atomLatest;

                /******************************************************************************
                 * @brief Atomically load the newest snapshot with acquire ordering.
                 ******************************************************************************/
                SharedSnapshot LoadLatest() const { return atomLatest.load(std::memory_order_acquire); }

                /******************************************************************************
                 * @brief Atomically store the newest snapshot with release ordering.
                 ******************************************************************************/
                void StoreLatest(SharedSnapshot pSnapshot) { atomLatest.store(std::move(pSnapshot), std::memory_order_release); }
#else
                // Fallback path: a plain shared_ptr accessed through the deprecated
                // free-function atomics (the only option on libstdc++ < 12).
                SharedSnapshot pLatest;

                /******************************************************************************
                 * @brief Atomically load the newest snapshot with acquire ordering.
                 ******************************************************************************/
                SharedSnapshot LoadLatest() const { return std::atomic_load_explicit(&pLatest, std::memory_order_acquire); }

                /******************************************************************************
                 * @brief Atomically store the newest snapshot with release ordering.
                 ******************************************************************************/
                void StoreLatest(SharedSnapshot pSnapshot) { std::atomic_store_explicit(&pLatest, std::move(pSnapshot), std::memory_order_release); }
#endif
        };
    }    // namespace internal

    /******************************************************************************
     * @brief The read face of a publish-latest channel, and the only thing a producer
     *      hands to consumers.
     *
     *      A Reader is a move-only RAII handle that does two things at once:
     *
     *        - It expresses demand. Construction increments the channel's subscriber
     *          count and destruction decrements it, so a producer retrieves and
     *          publishes a data type only while at least one Reader for it is alive.
     *          Demand therefore tracks reality with no manual unsubscribe to forget.
     *        - It reads. Get() returns the newest immutable snapshot with a non-blocking
     *          atomic load that never waits on the producer's loop.
     *
     *      Because Get() and the demand count live on the same object, it is impossible
     *      to read a channel without registering demand for it, and impossible to publish
     *      into a channel you only consume.
     *
     *      A Reader keeps its channel alive, so it remains safe to use even after the
     *      producer that created it has been destroyed. Get() then simply keeps returning
     *      the last value that was published.
     *
     * @tparam T - The value type carried by the channel.
     *
     * @note Load a snapshot ONCE into a local and work from that local. Calling Get()
     *      repeatedly returns whatever is newest each time, which is not a stable value.
     *      Published snapshots are immutable and shared; clone before modifying.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2026-07-26
     ******************************************************************************/
    template<typename T>
    class Reader
    {
        public:
            // Public type alias for the immutable snapshot handle this reader returns.
            using SharedSnapshot = std::shared_ptr<const Snapshot<T>>;

            /******************************************************************************
             * @brief Construct a new, inactive Reader. Holds no demand and Get() returns
             *      nullptr. Exists so a Reader can be a default-constructed member that is
             *      move-assigned later.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            Reader() = default;

            /******************************************************************************
             * @brief Construct a new active Reader on the given channel and register one
             *      unit of demand.
             *
             * @param pChannel - The channel to read from and express demand on.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            explicit Reader(std::shared_ptr<internal::Channel<T>> pChannel) : m_pChannel(std::move(pChannel))
            {
                // Register demand if the channel is valid.
                if (m_pChannel != nullptr)
                {
                    // Increment the subscriber count.
                    m_pChannel->nSubscribers.fetch_add(1, std::memory_order_acq_rel);
                }
            }

            /******************************************************************************
             * @brief Destroy the Reader object and release its demand.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            ~Reader() { this->Release(); }

            /******************************************************************************
             * @brief Move construct a Reader, transferring its demand.
             *
             * @param stOther - The Reader to move from.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            Reader(Reader&& stOther) noexcept : m_pChannel(std::move(stOther.m_pChannel)) { stOther.m_pChannel = nullptr; }

            /******************************************************************************
             * @brief Move assign a Reader, releasing our demand and taking theirs.
             *
             * @param stOther - The Reader to move from.
             * @return Reader& - A reference to this object.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            Reader& operator=(Reader&& stOther) noexcept
            {
                // Guard against self-assignment.
                if (this != &stOther)
                {
                    // Release our current demand before taking the other's.
                    this->Release();
                    // Transfer ownership of the channel.
                    m_pChannel         = std::move(stOther.m_pChannel);
                    stOther.m_pChannel = nullptr;
                }

                // Return a reference to this object.
                return *this;
            }

            // A Reader represents unique demand and must not be copied. Ask the producer for
            // another one instead; each carries its own demand.
            Reader(const Reader&)            = delete;
            Reader& operator=(const Reader&) = delete;

            /******************************************************************************
             * @brief Get the newest published snapshot with a non-blocking atomic load.
             *
             * @return SharedSnapshot - The newest immutable snapshot, or nullptr if nothing
             *                  has been published yet or this Reader is inactive.
             *
             * @note Load once into a local and work from that local. Calling Get()
             *      repeatedly returns whatever is newest each time, which is not a stable
             *      value.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            SharedSnapshot Get() const
            {
                ZoneScoped;

                // An inactive reader has nothing to read.
                if (m_pChannel == nullptr)
                {
                    // Report that no value is available.
                    return nullptr;
                }

                // Atomically load the newest published snapshot.
                return m_pChannel->LoadLatest();
            }

            /******************************************************************************
             * @brief Check whether this Reader is attached to a channel and holding demand.
             *
             * @return true - This Reader is active and counted.
             * @return false - This Reader is inactive/empty.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            bool IsActive() const { return m_pChannel != nullptr; }

            /******************************************************************************
             * @brief Release this Reader's demand early, before destruction. After this the
             *      Reader is inactive and Get() returns nullptr. Safe to call repeatedly.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            void Release()
            {
                // Only decrement if we currently hold demand.
                if (m_pChannel != nullptr)
                {
                    // Decrement the subscriber count.
                    m_pChannel->nSubscribers.fetch_sub(1, std::memory_order_acq_rel);
                    // Drop the channel so we never decrement twice and Get() reports nothing.
                    m_pChannel = nullptr;
                }
            }

        private:
            // Declare private member variables.
            std::shared_ptr<internal::Channel<T>> m_pChannel;    // The channel we read and hold demand on. Null when inactive.
    };

    /******************************************************************************
     * @brief The write face of a publish-latest channel for a single value type T.
     *      Held privately by the producer that owns the data; never handed to consumers.
     *
     *      Acquire() hands out a pooled, default-constructed snapshot slot (recycled
     *      buffers, never blocks, grows if the pool is empty). The producer writes into
     *      the slot's tData and calls Publish(), which stamps the sequence and time and
     *      atomically stores the slot as the newest snapshot.
     *
     *      Consumers are served through CreateReader(), which returns a Reader<T>: the
     *      read-only, demand-carrying half of this same channel.
     *
     * @tparam T - The value type to publish. Must be default-constructible.
     *
     * @note This class is neither copyable nor movable; hold it as a member and hand out
     *      Readers via an accessor. Do NOT expose it publicly: a consumer holding a
     *      Publisher could inject values into every other consumer of the channel.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2026-07-24
     ******************************************************************************/
    template<typename T>
    class Publisher
    {
        public:
            // Public type alias for the immutable snapshot handle this channel carries.
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
                // Allocate the shared channel control block.
                m_pChannel = std::make_shared<internal::Channel<T>>();

                // Store pool configuration.
                m_pChannel->siGrowthCeiling   = siGrowthCeiling;
                m_pChannel->fnSlotInitializer = std::move(fnSlotInitializer);

                // Pre-allocate the requested number of slots so steady state is allocation free.
                std::lock_guard<std::mutex> lkFreeList(m_pChannel->muFreeList);
                for (size_t siIter = 0; siIter < siPrealloc; ++siIter)
                {
                    // Allocate a slot, run the initializer, and place it on the free list.
                    m_pChannel->vFreeList.push_back(m_pChannel->AllocateSlotLocked());
                }
            }

            /******************************************************************************
             * @brief Destroy the Publisher object. The channel itself survives as long as
             *      any Reader or outstanding snapshot still references it, so consumers
             *      that outlive their producer keep working instead of dangling.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            ~Publisher() = default;

            // A Publisher owns the write end of a channel and must not be copied or moved.
            Publisher(const Publisher&)            = delete;
            Publisher& operator=(const Publisher&) = delete;
            Publisher(Publisher&&)                 = delete;
            Publisher& operator=(Publisher&&)      = delete;

            /******************************************************************************
             * @brief Create a read-only handle to this channel for a consumer, registering
             *      one unit of demand for as long as that handle lives.
             *
             *      This is the ONLY thing a producer should expose publicly. Handing out the
             *      Publisher itself would let a consumer publish into the channel.
             *
             * @return Reader<T> - A move-only, demand-carrying read handle.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            Reader<T> CreateReader() { return Reader<T>(m_pChannel); }

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
                ZoneScoped;
                // The raw slot pointer we will hand out.
                Snapshot<T>* pSlot = nullptr;

                // Try to pop a recycled slot off the free list.
                {
                    // Lock the free list only long enough to pop.
                    std::lock_guard<std::mutex> lkFreeList(m_pChannel->muFreeList);
                    if (!m_pChannel->vFreeList.empty())
                    {
                        // Reuse the most recently returned slot (warmest in cache).
                        pSlot = m_pChannel->vFreeList.back();
                        m_pChannel->vFreeList.pop_back();
                    }
                    else
                    {
                        // Free list is empty: allocate a fresh slot and count the miss.
                        pSlot = m_pChannel->AllocateSlotLocked();
                        m_pChannel->siPoolMisses.fetch_add(1, std::memory_order_relaxed);
                    }
                }

                // Wrap the slot in a shared_ptr whose deleter recycles it into the channel.
                // The deleter captures a weak_ptr, never a raw this, so a consumer that
                // outlives the channel deletes the slot instead of dereferencing freed
                // memory. The deleter is noexcept.
                std::weak_ptr<internal::Channel<T>> wpChannel = m_pChannel;
                return std::shared_ptr<Snapshot<T>>(pSlot,
                                                    [wpChannel](Snapshot<T>* pReturned) noexcept
                                                    {
                                                        // If the channel still exists, return the slot to it; otherwise delete.
                                                        if (std::shared_ptr<internal::Channel<T>> pLockedChannel = wpChannel.lock())
                                                        {
                                                            // Recycle the slot for reuse.
                                                            pLockedChannel->Return(pReturned);
                                                        }
                                                        else
                                                        {
                                                            // Channel is gone; free the slot outright.
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
                ZoneScoped;
                // Ignore a null publish rather than crashing.
                if (pSnapshot == nullptr)
                {
                    // Nothing to publish.
                    return;
                }

                // Stamp the sequence number and publish time inside the snapshot so they
                // are atomic with the data.
                pSnapshot->ullSequence = m_pChannel->ullSequence.fetch_add(1, std::memory_order_relaxed) + 1;
                pSnapshot->tmPublished = std::chrono::system_clock::now();

                // Atomically store the (now immutable) snapshot as the newest value.
                m_pChannel->StoreLatest(SharedSnapshot(std::move(pSnapshot)));
            }

            /******************************************************************************
             * @brief Read the newest published snapshot WITHOUT registering demand. This
             *      exists for the producer that owns this Publisher to read back its own
             *      channel (for example a camera answering GetCameraIsOpen() from the
             *      status snapshot it just published).
             *
             * @return SharedSnapshot - The newest immutable snapshot, or nullptr if
             *                  nothing has been published yet.
             *
             * @note Consumers must never use this; they hold a Reader, which couples the
             *      read to the demand that makes the value get produced in the first
             *      place. This is only correct for the owner of the channel, whose
             *      production does not depend on its own demand.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            SharedSnapshot PeekLatest() const
            {
                ZoneScoped;
                return m_pChannel->LoadLatest();
            }

            /******************************************************************************
             * @brief Check whether any consumer currently has demand for this data.
             *
             * @return true - At least one Reader is alive.
             * @return false - No consumer is reading this channel.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            bool HasSubscribers() const { return m_pChannel->nSubscribers.load(std::memory_order_acquire) > 0; }

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
            size_t GetPoolMisses() const { return m_pChannel->siPoolMisses.load(std::memory_order_relaxed); }

            /******************************************************************************
             * @brief Accessor for the current publish sequence number (the sequence of
             *      the most recently published snapshot, or 0 if none).
             *
             * @return unsigned long long - The newest published sequence number.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            unsigned long long GetSequence() const { return m_pChannel->ullSequence.load(std::memory_order_relaxed); }

            /******************************************************************************
             * @brief Accessor for the total number of slots this pool has ever
             *      allocated (free list size plus slots currently in use).
             *
             * @return size_t - The total number of allocated slots.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-24
             ******************************************************************************/
            size_t GetPoolAllocated() const { return m_pChannel->siAllocated.load(std::memory_order_relaxed); }

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
                std::lock_guard<std::mutex> lkFreeList(m_pChannel->muFreeList);
                return m_pChannel->vFreeList.size();
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
            bool GetGrowthCeilingBreached() const { return m_pChannel->bCeilingBreached.load(std::memory_order_relaxed); }

            /******************************************************************************
             * @brief Accessor for the current number of live Readers on this channel.
             *      Diagnostics only.
             *
             * @return long - The number of consumers currently expressing demand.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-07-26
             ******************************************************************************/
            long GetSubscriberCount() const { return m_pChannel->nSubscribers.load(std::memory_order_acquire); }

        private:
            // Declare private member variables.
            std::shared_ptr<internal::Channel<T>> m_pChannel;    // Shared control block: pool, latest value, demand, sequence.
    };
}    // namespace pubsub

#endif    // PUBLISHER_HPP
