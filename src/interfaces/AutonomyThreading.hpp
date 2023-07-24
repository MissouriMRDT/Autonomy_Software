/******************************************************************************
 * @brief This interface defines the base functions needed to multi-thread a
 *      class in Autonomy_Software. Some methods contain default implementations
 *      and others are pure virtual methods that need to be implemented by the
 *      inheritor.
 *
 * @file AutonomyThreading.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0716
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/
#include <atomic>
#include <vector>

#include "../../external/threadpool/BSThreadPool.hpp"

#ifndef AUTONOMYTHREAD_H
#define AUTONOMYTHREAD_H

template<class T>
class AutonomyThreading
{
    private:
        // Declare and define interface class private member variables.
        std::atomic_bool m_bStopThreads = false;
        BS::thread_pool m_thPool        = BS::thread_pool(1);
        std::vector<void*> m_vPoolReturns;

        // Declare interface class pure virtual functions. (These must be overriden be inheritor.)
        virtual T ThreadedCode() = 0;

        // Declare and define private interface methods.
        /******************************************************************************
         * @brief This method is ran in a seperate thread. It is a middleware between the
         *      class member thread and the user code that handles graceful stopping of
         *      user code.
         *
         * @param bStopThread - Atomic shared variable that signals the thread to stop interating.
         * @return T - Variable return type from user code.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0724
         ******************************************************************************/
        T RunThread(std::atomic_bool& bStopThread)
        {
            while (!bStopThread)
            {
                // Call method containing user code.
                this->ThreadedCode();
            }

            // Do one more iteration to get return type from user code.
            return this->ThreadedCode();
        }

    public:
        /******************************************************************************
         * @brief Destroy the Autonomy Thread object. If the parent object or main thread
         *      is destroyed or exited while this thread is still running, a race condition
         *      will occur. Stopping and joining the thread here insures that the main
         *      program can't exit if the user forgot to stop and join the thread.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0723
         ******************************************************************************/
        virtual ~AutonomyThreading()
        {
            // Tell all threads to stop executing user code.
            m_bStopThreads = true;

            // Wait for all threads to finish.
            m_thPool.wait_for_tasks();
        }

        /******************************************************************************
         * @brief When this method is called, it starts a new thread that runs the
         *      code within the RunThread and ThreadedCode methods.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        void Start()
        {
            // Tell any open thread to stop.
            m_bStopThreads = true;

            // Pause queuing of new tasks to the threads, then purge them.
            m_thPool.pause();
            m_thPool.purge();
            // Wait for open threads to terminate, then resize the pool to only 1 thread.
            m_thPool.reset(1);

            // Reset thread stop toggle.
            m_bStopThreads = false;

            // Submit single task to pool queue.
            std::future<T> future = m_thPool.submit(&AutonomyThreading::RunThread, this, std::ref(m_bStopThreads));
            // Unpause queue.
            m_thPool.unpause();
        }

        /******************************************************************************
         * @brief When this method is called, it starts a thread pool that runs nNumThreads
         *      copies of the code withing the RunThread and ThreadedCode methods.
         *      Default value for nNumThreads is 1.
         *
         * @param nNumThreads - The number of threads to run user code in.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0723
         ******************************************************************************/
        void StartPool(const int nNumThreads = 2)
        {
            // Tell any open thread to stop.
            m_bStopThreads = true;

            // Pause queuing of new tasks to the threads, then purge them.
            m_thPool.pause();
            m_thPool.purge();
            // Wait for open threads to terminate, then resize the pool.
            m_thPool.reset(nNumThreads);
        }

        /******************************************************************************
         * @brief When this method is called, it starts a thread pool full of threads that
         *      don't return std::futures (like a placeholder for the thread return type). This
         *      means the thread will not have a return type and there is not way to determine
         *      if the thread has finished. Only use this if you want to 'set and forget'.
         *
         *      If this method is called directly after Start() or StartPool(), it will signal
         *      for those threads to stop and wait until they exit on their next iteration. Any
         *      number of tasks that are still queued will be cleared.
         *      If you want to wait until they fully execute their code, then call the Join()
         *      method before this one.
         *
         * @param nNumThreads - The number of threads to run user code in.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0723
         ******************************************************************************/
        void StartDetachedPool(const int nNumThreads = 2)
        {
            // Tell any open thread to stop.
            m_bStopThreads = true;

            // Pause queuing of new tasks to the threads, then purge them.
            m_thPool.pause();
            m_thPool.purge();
            // Wait for open threads to terminate, then resize the pool.
            m_thPool.reset(nNumThreads);
        }

        /******************************************************************************
         * @brief Signals threads to stop executing user code, terminate. DOES NOT JOIN.
         *      This method will not force the thread to exit, if the user code is not
         *      written properly and contains WHILE statement or any other long-executing
         *      or blocking code, then the thread will not exit until the next iteration.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        void RequestStop()
        {
            // Set the stop toggle.
            m_bStopThreads = true;
        }

        /******************************************************************************
         * @brief Waits for thread to finish executing and then closes thread.
         *      This method will block the calling code until thread is finished.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        void Join()
        {
            // Wait for pool to finish all tasks.
            m_thPool.wait_for_tasks();
        }

        /******************************************************************************
         * @brief Check if the code within the thread is finished executing and the
         *      thread is ready to be closed.
         *
         * @return true - The thread is finished and joinable.
         * @return false - The thread is still running code.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        bool Joinable()
        {
            // Check current number of running and queued tasks.
            if (m_thPool.get_tasks_total() <= 0)
            {
                // Threads are joinable.
                return true;
            }
            else
            {
                // Threads are still running.
                return false;
            }
        }
};

#endif
