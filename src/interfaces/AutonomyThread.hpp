/******************************************************************************
 * @brief This interface defines the base functions needed to multi-thread a
 *      class in Autonomy_Software. Some methods contain default implementations
 *      and others are pure virtual methods that need to be implemented by the
 *      inheriter.
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
        // Declare interface class pure virtual functions. (These must be overriden be inheriter.)
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
            std::cout << sToken.stop_requested() << std::endl;
            while (!sToken.stop_requested())
            {
                // Call method containing user code.
                // this->RunCode();
            }
        }

        // Declare interface class private member variables.
        std::jthread m_tThread;

    public:
        // Declare interface class virtual functions. (These can be overidden, but not required.)
        virtual ~AutonomyThread() = default;

        // Declare and define other public interface methods.
        /******************************************************************************
         * @brief When this method is called, is starts a new thread and runs
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        void Start()
        {
            // Start new thread that runs the RunThread method.
            m_tThread = std::jthread([this](std::stop_token sToken) { this->RunThread(sToken); });
        }

        /******************************************************************************
         * @brief Signals thread to stop executing user code, terminate, and join.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        void Stop()
        {
            // Signal thread to stop.
            m_tThread.request_stop();
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
            // Wait for thread to finish and then join.
            m_tThread.join();
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
            // Check if the thread code is finished executing.
            return m_tThread.joinable();
        }

        /******************************************************************************
         * @brief Detach this thread from its caller, allowing it to run completely
         *      independant from the main program. Once detached, this object will no
         *      longer be able to reference its internal thread member variable.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        void Detach()
        {
            // Detach the jthread from its caller.
            m_tThread.detach();
        }
};

#endif
