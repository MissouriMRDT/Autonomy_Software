/******************************************************************************
 * @brief This interface defines the base functions needed to multi-thread a
 *      class in Autonomy_Software. Some methods contain default implementations
 *      and others are pure virtual methods that need to be implemented by the
 *      inheritor.
 *
 * @file AutonomyThread.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-16
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/
#ifndef AUTONOMYTHREAD_H
#define AUTONOMYTHREAD_H

#include <atomic>
#include <vector>

#include "../../external/threadpool/BSThreadPool.hpp"

/******************************************************************************
 * @brief Interface class used to easily multithread a child class.
 *
 * @tparam T - Variable return type of internal pooled code.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-27
 ******************************************************************************/
template<class T>
class AutonomyThread
{
    private:
        // Declare and define interface class private member variables.
        std::atomic_bool m_bStopThreads = false;
        BS::thread_pool m_thMainThread  = BS::thread_pool(1);
        BS::thread_pool m_thPool        = BS::thread_pool(2);
        std::future<void> m_fuMainReturn;
        std::vector<std::future<T>> m_vPoolReturns;

        // Declare interface class pure virtual functions. (These must be overriden by inheritor.)
        virtual void ThreadedContinuousCode() = 0;    // This is where user's main single threaded and continuously looping code will go.
        virtual T PooledLinearCode()          = 0;    // This is where user's offshoot, highly parallelizable code will go. Helpful for intensive short-lived tasks.
                                                      // Can be ran from inside the ThreadedContinuousCode() method.

        // Declare and define private interface methods.
        /******************************************************************************
         * @brief This method is ran in a separate thread. It is a middleware between the
         *      class member thread and the user code that handles graceful stopping of
         *      user code. This method is intentionally designed to not return anything.
         *
         * @param bStopThread - Atomic shared variable that signals the thread to stop iterating.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-24
         ******************************************************************************/
        void RunThread(std::atomic_bool& bStopThread)
        {
            // Loop until stop flag is set.
            while (!bStopThread)
            {
                // Call method containing user code.
                this->ThreadedContinuousCode();
            }
        }

    protected:
        /******************************************************************************
         * @brief When this method is called, it starts/adds tasks to a thread pool that runs nNumTasksToQueue
         *      copies of the code within the PooledLinearCode() method using nNumThreads number of threads. This is meant to be
         *      used as an internal utility of the child class to further improve parallelization.
         *      Default value for nNumThreads is 2.
         *
         *      If this method is called directly after itself or RunDetachedPool(), it will just add more
         *      tasks to the queue. If the bForceStopCurrentThreads is enabled, it will signal
         *      for those threads to stop and wait until they exit on their next iteration. Any number of
         *      tasks that are still queued will be cleared. Old results will be destroyed. If you want
         *      to wait until they fully execute their code, then call the Join() method before this one.
         *
         *      Once the pool is created it stays alive for as long as the program runs or until
         *      a different threading method is called. So there's no overhead with starting and
         *      stopping threads or queueing more tasks.
         *
         *      YOU MUST HANDLE MUTEX LOCKS AND ATOMICS. It is impossible for this class to handle
         *      locks as all possible solutions lead to a solution that only lets one thread run at
         *      a time, essentially canceling out the parallelism.
         *
         * @param nNumTasksToQueue - The number of tasks running PooledLinearCode() to queue.
         * @param nNumThreads - The number of threads to run user code in.
         * @param bForceStopCurrentThreads - Clears the current tasks queue then signals and waits for existing
         *                                  tasks to stop before queueing more.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-23
         ******************************************************************************/
        void RunPool(const unsigned int nNumTasksToQueue, const unsigned int nNumThreads = 2, const bool bForceStopCurrentThreads = false)
        {
            // Check if the pools need to be resized.
            if (m_thPool.get_thread_count() != nNumThreads)
            {
                // Tell any open thread to stop.
                m_bStopThreads = true;

                // Pause queuing of new tasks to the threads, then purge them.
                m_thPool.pause();
                m_thPool.purge();
                // Wait for open threads to terminate, then resize the pool.
                m_thPool.reset(nNumThreads);
                // Unpause queue.
                m_thPool.unpause();

                // Clear results vector.
                m_vPoolReturns.clear();
                // Reset thread stop toggle.
                m_bStopThreads = false;
            }
            // Check if the current pool tasks should be stopped before queueing more tasks.
            else if (bForceStopCurrentThreads)
            {
                // Tell any open thread to stop.
                m_bStopThreads = true;

                // Pause queuing of new tasks to the threads, then purge them.
                m_thPool.pause();
                m_thPool.purge();
                // Wait for threadpool to join.
                m_thPool.wait_for_tasks();
                // Unpause queue.
                m_thPool.unpause();

                // Reset stop toggle.
                m_bStopThreads = false;
            }

            // Loop nNumThreads times and queue tasks.
            for (int i = 0; i < nNumTasksToQueue; ++i)
            {
                // Submit single task to pool queue.
                m_vPoolReturns.emplace_back(m_thPool.submit(
                    [this]()
                    {
                        // Run user pool code without lock.
                        this->PooledLinearCode();
                    }));
            }
        }

        /******************************************************************************
         * @brief When this method is called, it starts a thread pool full of threads that
         *      don't return std::futures (like a placeholder for the thread return type). This
         *      means the thread will not have a return type and there is no way to determine
         *      if the thread has finished other than calling the Join() method.
         *      Only use this if you want to 'set and forget'. It will be faster as it doesn't
         *      return futures. Runs PooledLinearCode() method code. This is meant to be
         *      used as an internal utility of the child class to further improve parallelization.
         *
         *      If this method is called directly after itself or RunPool(), it will just add more
         *      tasks to the queue. If the bForceStopCurrentThreads is enabled, it will signal
         *      for those threads to stop and wait until they exit on their next iteration.
         *      Any number of tasks that are still queued will be cleared. Old results will be destroyed.
         *      If you want to wait until they fully execute their code, then call the Join() method before this one.
         *
         *      Once the pool is created it stays alive for as long as the program runs or until
         *      a different threading method is called. So there's no overhead with starting and
         *      stopping threads or queueing more tasks.
         *
         *      YOU MUST HANDLE MUTEX LOCKS AND ATOMICS. It is impossible for this class to handle
         *      locks as all possible solutions lead to a solution that only lets one thread run at
         *      a time, essentially canceling out the parallelism.
         *
         * @param nNumTasksToQueue - The number of tasks running PooledLinearCode() to queue.
         * @param nNumThreads - The number of threads to run user code in.
         * @param bForceStopCurrentThreads - Clears the current tasks queue then signals and waits for existing
         *                                  tasks to stop before queueing more.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-23
         ******************************************************************************/
        void RunDetachedPool(const unsigned int nNumTasksToQueue, const unsigned int nNumThreads = 2, const bool bForceStopCurrentThreads = false)
        {
            // Check if the pools need to be resized.
            if (m_thPool.get_thread_count() != nNumThreads)
            {
                // Tell any open thread to stop.
                m_bStopThreads = true;

                // Pause queuing of new tasks to the threads, then purge them.
                m_thPool.pause();
                m_thPool.purge();
                // Wait for open threads to terminate, then resize the pool.
                m_thPool.reset(nNumThreads);
                // Unpause queue.
                m_thPool.unpause();

                // Clear results vector.
                m_vPoolReturns.clear();
                // Reset thread stop toggle.
                m_bStopThreads = false;
            }
            // Check if the current pool tasks should be stopped before queueing more tasks.
            else if (bForceStopCurrentThreads)
            {
                // Tell any open thread to stop.
                m_bStopThreads = true;

                // Pause queuing of new tasks to the threads, then purge them.
                m_thPool.pause();
                m_thPool.purge();
                // Wait for threadpool to join.
                m_thPool.wait_for_tasks();
                // Unpause queue.
                m_thPool.unpause();

                // Reset stop toggle.
                m_bStopThreads = false;
            }

            // Loop nNumThreads times and queue tasks.
            for (unsigned int i = 0; i < nNumTasksToQueue; ++i)
            {
                // Push single task to pool queue. No return value no control.
                m_thPool.push_task(
                    [this]()
                    {
                        // Run user code without lock.
                        this->PooledLinearCode();
                    });
            }
        }

        /******************************************************************************
         * @brief Given a ref-qualified looping function and an arbitrary number of iterations,
         *      this method will divide up the loop and run each section in a thread pool.
         *      This function must not return anything. This method will block until the
         *      loop has completed.
         *
         *      To see an example of how to use this function, check out ArucoGenerateTags in
         *      the threads example folder.
         *
         *      YOU MUST HANDLE MUTEX LOCKS AND ATOMICS. It is impossible for this class to handle
         *      locks as all possible solutions lead to a solution that only lets one thread run at
         *      a time, essentially canceling out the parallelism.
         *
         * @tparam N - Template argument for the nTotalIterations type.
         * @tparam F - Template argument for the given function reference.
         * @param nNumThreads - The number of threads to use for the thread pool.
         * @param nTotalIterations - The total iterations to loop for.
         * @param tLoopFunction - Ref-qualified function to run.
         *                       MUST ACCEPT TWO ARGS: const int a, const int b.
         *                       a - loop start
         *                       b - loop end
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-26
         ******************************************************************************/
        template<typename N, typename F>
        void ParallelizeLoop(const int nNumThreads, const N tTotalIterations, F&& tLoopFunction)
        {
            // Create new thread pool.
            BS::thread_pool m_thLoopPool = BS::thread_pool(nNumThreads);

            m_thLoopPool.push_loop(tTotalIterations,
                                   [&tLoopFunction](const int a, const int b)
                                   {
                                       // Call loop function without lock.
                                       tLoopFunction(a, b);
                                   });

            // Wait for loop to finish.
            m_thLoopPool.wait_for_tasks();
        }

        /******************************************************************************
         * @brief Clears any tasks waiting to be ran in the queue, tasks currently
         *      running will remain running.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-09-09
         ******************************************************************************/
        void ClearPoolQueue() { m_thPool.purge(); }

        /******************************************************************************
         * @brief Waits for pool to finish executing tasks. This method will block
         *      the calling code until thread is finished.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-22
         ******************************************************************************/
        void JoinPool() { m_thPool.wait_for_tasks(); }

        /******************************************************************************
         * @brief Check if the internal pool threads are done executing code and the
         *      queue is empty.
         *
         * @return true - The thread is finished and joinable.
         * @return false - The thread is still running code.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-22
         ******************************************************************************/
        bool PoolJoinable() const
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

        /******************************************************************************
         * @brief Accessor for the Pool Num Of Threads private member.
         *
         * @return int - The number of threads available to the pool.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-09-09
         ******************************************************************************/
        int GetPoolNumOfThreads() { return m_thPool.get_thread_count(); }

        /******************************************************************************
         * @brief Accessor for the Pool Results private member. The action of getting
         *      results will destroy and remove them from this object. This method blocks
         *      if the thread is not finished, so no need to call JoinPool() before getting
         *      results.
         *
         * @return std::vector<T> - A vector containing the returns from each thread that
         *                      ran the PooledLinearCode.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-26
         ******************************************************************************/
        std::vector<T> GetPoolResults()
        {
            // Create instance variable.
            std::vector<T> vResults;

            // Loop the pool futures and get result.
            for (std::future<T> fResult : m_vPoolReturns)
            {
                // Store returned value.
                vResults.emplace_back(fResult.get());
            }

            // Clear pool returns member variable.
            m_vPoolReturns.clear();

            return vResults;
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
         * @date 2023-07-23
         ******************************************************************************/
        virtual ~AutonomyThread()
        {
            // Tell all threads to stop executing user code.
            m_bStopThreads = true;

            // Pause and clear pool queues.
            m_thPool.pause();
            m_thPool.purge();
            m_thMainThread.pause();
            m_thMainThread.purge();

            // Wait for all pools to finish.
            m_thPool.wait_for_tasks();
            m_thMainThread.wait_for_tasks();
        }

        /******************************************************************************
         * @brief When this method is called, it starts a new thread that runs the
         *      code within the ThreadedContinuousCode method. This is the users
         *      main code that will run the important and continuous code for the class.
         *
         *      If this method is called directly after itself, RunPool(), or RunDetachedPool(), it will signal
         *      for those threads to stop and wait until they exit on their next iteration. Any
         *      number of tasks that are still queued will be cleared. Old results will be destroyed.
         *      If you want to wait until they fully execute their code, then call the Join()
         *      method before starting a new thread.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-22
         ******************************************************************************/
        void Start()
        {
            // Tell any open thread to stop.
            m_bStopThreads = true;

            // Pause queuing of new tasks to the threads, then purge them.
            m_thPool.pause();
            m_thPool.purge();
            m_thMainThread.pause();
            m_thMainThread.purge();

            // Wait for loop, pool and main thread to join.
            this->Join();

            // Clear results vector.
            m_vPoolReturns.clear();
            // Reset thread stop toggle.
            m_bStopThreads = false;

            // Submit single task to pool queue and store resulting future. Still using pool, as it's scheduling is more efficient.
            m_fuMainReturn = m_thMainThread.submit(&AutonomyThread::RunThread, this, std::ref(m_bStopThreads));

            // Unpause pool queues.
            m_thPool.unpause();
            m_thMainThread.unpause();
        }

        /******************************************************************************
         * @brief Signals threads to stop executing user code, terminate. DOES NOT JOIN.
         *      This method will not force the thread to exit, if the user code is not
         *      written properly and contains WHILE statement or any other long-executing
         *      or blocking code, then the thread will not exit until the next iteration.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-22
         ******************************************************************************/
        void RequestStop() { m_bStopThreads = true; }

        /******************************************************************************
         * @brief Waits for thread to finish executing and then closes thread.
         *      This method will block the calling code until thread is finished.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-22
         ******************************************************************************/
        void Join()
        {
            // Wait for pool to finish all tasks.
            m_thPool.wait_for_tasks();
            // Wait for main thread to finish.
            m_thMainThread.wait_for_tasks();
        }

        /******************************************************************************
         * @brief Check if the code within the thread and all pools created by it are
         *       finished executing and the thread is ready to be closed.
         *
         * @return true - The thread is finished and joinable.
         * @return false - The thread is still running code.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-22
         ******************************************************************************/
        bool Joinable() const
        {
            // Check current number of running and queued tasks.
            if (m_thMainThread.get_tasks_total() <= 0 && m_thPool.get_tasks_total() <= 0)
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
