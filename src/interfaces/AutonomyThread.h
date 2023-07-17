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

class AutonomyThread : public std::jthread
{
    private:
        // Define interface member object and variables.

    public:
        // Define interface methods.
};

#endif
