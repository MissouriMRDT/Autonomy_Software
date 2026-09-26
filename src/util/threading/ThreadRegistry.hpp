/******************************************************************************
 * @brief Defines and implements the process-wide thread registry: a list of every
 *      started AutonomyThread and a lock-free iteration counter for each one, so
 *      tools like the web visualizer can graph per-thread FPS.
 *
 *      Each thread only does a relaxed atomic increment per iteration. Readers sample
 *      the raw counters and compute FPS themselves from the change between two
 *      samples, so the registry keeps no timing state and any number of readers can
 *      sample it without affecting each other.
 *
 * @file ThreadRegistry.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-09-23
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#ifndef THREAD_REGISTRY_HPP
#define THREAD_REGISTRY_HPP

/// \cond
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

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
     * @brief The counters one thread publishes to the registry. Owned by the thread
     *      object; written only by that thread (iterations) or its owner (max IPS).
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2026-09-23
     ******************************************************************************/
    struct ThreadTelemetry
    {
        public:
            std::atomic<uint64_t> aullIterations{0};    // Completed main loop iterations since construction.
            std::atomic<int> anMaxIPS{0};               // The thread's IPS limit. 0 means unlimited.
    };

    /******************************************************************************
     * @brief A point-in-time copy of one registered thread's telemetry.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2026-09-23
     ******************************************************************************/
    struct ThreadTelemetrySample
    {
        public:
            uint32_t unID;             // Unique for the life of the process, so readers can key series on it.
            std::string szName;        // Display name.
            uint64_t ullIterations;    // Completed main loop iterations.
            int nMaxIPS;               // IPS limit. 0 means unlimited.
    };

    /******************************************************************************
     * @brief The process-wide list of registered threads. Registration and sampling
     *      take one mutex; the per-iteration counter update never does.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2026-09-23
     ******************************************************************************/
    class ThreadRegistry
    {
        public:
            /******************************************************************************
             * @brief Get the single process-wide registry.
             *
             * @return ThreadRegistry& - The registry.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-09-23
             ******************************************************************************/
            static ThreadRegistry& Instance()
            {
                static ThreadRegistry stRegistry;
                return stRegistry;
            }

            /******************************************************************************
             * @brief Add a thread to the registry, or rename it if it is already there.
             *
             * @param pTelemetry - The thread's counters. Must stay valid until Unregister().
             * @param szName - The display name.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-09-23
             ******************************************************************************/
            void Register(const ThreadTelemetry* pTelemetry, const std::string& szName)
            {
                // Acquire the registry lock.
                std::lock_guard lkRegistry(m_muEntries);
                // Rename in place if this thread was started before.
                for (Entry& stEntry : m_vEntries)
                {
                    if (stEntry.pTelemetry == pTelemetry)
                    {
                        stEntry.szName = szName;
                        return;
                    }
                }
                // Otherwise add it with a fresh ID.
                m_vEntries.push_back({++m_unLastID, szName, pTelemetry});
            }

            /******************************************************************************
             * @brief Remove a thread from the registry. Safe to call for a thread that was
             *      never registered. After this returns, no sampler touches the counters.
             *
             * @param pTelemetry - The thread's counters.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-09-23
             ******************************************************************************/
            void Unregister(const ThreadTelemetry* pTelemetry)
            {
                // Acquire the registry lock.
                std::lock_guard lkRegistry(m_muEntries);
                // Drop the entry for this thread, if any.
                std::erase_if(m_vEntries, [pTelemetry](const Entry& stEntry) { return stEntry.pTelemetry == pTelemetry; });
            }

            /******************************************************************************
             * @brief Copy the current counters of every registered thread.
             *
             * @return std::vector<ThreadTelemetrySample> - One sample per thread, in registration order.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-09-23
             ******************************************************************************/
            std::vector<ThreadTelemetrySample> Sample() const
            {
                // Acquire the registry lock so no entry is unregistered (and destroyed) mid-read.
                std::lock_guard lkRegistry(m_muEntries);
                std::vector<ThreadTelemetrySample> vSamples;
                vSamples.reserve(m_vEntries.size());
                for (const Entry& stEntry : m_vEntries)
                {
                    vSamples.push_back({stEntry.unID,
                                        stEntry.szName,
                                        stEntry.pTelemetry->aullIterations.load(std::memory_order_relaxed),
                                        stEntry.pTelemetry->anMaxIPS.load(std::memory_order_relaxed)});
                }
                return vSamples;
            }

        private:
            /******************************************************************************
             * @brief One registered thread.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2026-09-23
             ******************************************************************************/
            struct Entry
            {
                public:
                    uint32_t unID;
                    std::string szName;
                    const ThreadTelemetry* pTelemetry;
            };

            ThreadRegistry() = default;

            mutable std::mutex m_muEntries;
            std::vector<Entry> m_vEntries;
            uint32_t m_unLastID = 0;
    };
}    // namespace threadutils

#endif
