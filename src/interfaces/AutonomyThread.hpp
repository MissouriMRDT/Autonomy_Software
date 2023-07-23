/******************************************************************************
 * @brief This interface defines the base functions needed to multi-thread a
 *      class in Autonomy_Software. Some methods contain default implementations
 *      and others are pure virtual methods that need to be implemented by the
 *      inheritor.
 *
 * @file AutonomyThread.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0716
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/
#include <atomic>
#include <thread>

#ifndef AUTONOMYTHREAD_H
#define AUTONOMYTHREAD_H

class AutonomyThread
{
    private:
        // Declare interface class private member variables.
        std::jthread m_tThread;
        bool m_bThreadStarted = false;

        // Declare interface class pure virtual functions. (These must be overriden be inheritor.)
        virtual void RunCode() = 0;

        // Declare and define private interface methods.
        /******************************************************************************
         * @brief This method is ran in a seperate thread. It is a middleware between the
         *      class member thread and the user code that handles graceful stopping of
         *      user code.
         *
         * @param sToken - The stop token used to determine if the thread should shutdown.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        void RunThread(std::stop_token& sToken)
        {
            while (!sToken.stop_requested())
            {
                // Call method containing user code.
                this->RunCode();
            }
        }

    public:
        /******************************************************************************
         * @brief Destroy the Autonomy Thread object. If the parent object or main thread
         *      is destroyed or exited, a race condition will occur. Stopping and joining
         *      the thread here insures that the main program can't exit if the user
         *      forgot to stop and join the thread.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0723
         ******************************************************************************/
        ~AutonomyThread()
        {
            // Check if thread is still started.
            if (m_bThreadStarted)
            {
                // Print warning log.

                // Rejoin thread before destroying this object.
                RequestStop();
                Join();
            }
        }

        /******************************************************************************
         * @brief When this method is called, is starts a new thread and runs
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        void Start()
        {
            // Check if the thread has already been started.
            if (!m_bThreadStarted)
            {
                // Start new thread that runs the RunThread method.
                m_tThread = std::jthread([this](std::stop_token sToken) { this->RunThread(sToken); });

                // Set toggle.
                m_bThreadStarted = true;

                // Print debug log.
            }
        }

        /******************************************************************************
         * @brief Signals thread to stop executing user code, terminate. DOES NOT JOIN.
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
            // Make sure thread has been started.
            if (m_bThreadStarted)
            {
                // Signal thread to stop.
                m_tThread.request_stop();

                // Check if a stop was requested while thread is still running.
                if (!m_tThread.joinable())
                {
                    // Print info log.
                }
            }
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
            // Make sure thread has been started.
            if (m_bThreadStarted)
            {
                // Wait for thread to finish and then join.
                m_tThread.join();

                // Once thread has joined, reset start toggle.
                m_bThreadStarted = false;
            }
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
            // Make sure thread has been started.
            if (m_bThreadStarted)
            {
                // Check if the thread code is finished executing.
                return m_tThread.joinable();
            }
            else
            {
                return false;
            }
        }

        /******************************************************************************
         * @brief Detach this thread from its caller, allowing it to run completely
         *      independant from the main program. Once detached, this object will no
         *      longer be able to reference its internal thread member variable and the
         *      thread will not be stoppable or joinable from this object.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        void Detach()
        {
            // Detach the jthread from its caller.
            m_tThread.detach();

            // Since thread is detached, we are no longer in control. Reset start toggle so we can start another thread.
            m_bThreadStarted = false;
        }
};

#endif
