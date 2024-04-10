/******************************************************************************
 * @brief Unit test for IPS utility class.
 *
 * @file IPS.cc
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/util/IPS.hpp"

/// \cond
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

/// \endcond

/******************************************************************************
 * @brief Check that IPS doesn't leak any memory.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
TEST(IPSTests, DoesNotLeak)
{
    // Create a new IPS object.
    IPS* pIPS = new IPS();
    // Delete object.
    delete pIPS;
    // Point to null.
    pIPS = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
TEST(IPSTests, Leaks)
{
    // Create a new IPS object.
    IPS* pIPS = new IPS();
    EXPECT_TRUE(pIPS != nullptr);
}

/******************************************************************************
 * @brief Test IPS counting functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
TEST(IPSTest, IterationCounter)
{
    // Create a new IPS object.
    IPS* pIPS = new IPS();

    // Call tick method, wait 16.666 ms, call tick method.
    pIPS->Tick();
    std::this_thread::sleep_for(std::chrono::microseconds(16666));
    pIPS->Tick();

    // Get the calculated FPS.
    double dFPS = pIPS->GetExactIPS();

    // Check if FPS is within 5 frames of 60.
    EXPECT_TRUE(dFPS <= 65 && dFPS >= 55);

    // Delete object.
    delete pIPS;
    // Point to null.
    pIPS = nullptr;
}

/******************************************************************************
 * @brief Test IPS metrics accuracy.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
TEST(IPSTest, MetricsFunctionality)
{
    // Create a new IPS object.
    IPS* pIPS = new IPS();

    // Declare a start time, current time, elapsed time.
    std::chrono::time_point tStartTime         = std::chrono::high_resolution_clock::now();
    std::chrono::time_point tCurrentTime       = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> tElapsedTime = tCurrentTime - tStartTime;

    // Loop for a second.
    while (tElapsedTime.count() < 1)
    {
        // For the first 500ms, sleep for 2ms per iteration.
        if (tElapsedTime.count() < 0.5)
        {
            // Call tick method.
            pIPS->Tick();

            // Sleep for 2ms.
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        // For the next 400ms, sleep for 10ms.
        else if (tElapsedTime.count() < 0.9)
        {
            // Call tick method.
            pIPS->Tick();

            // Sleep for 2ms.
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Update elapsed time.
        tCurrentTime = std::chrono::high_resolution_clock::now();
        tElapsedTime = tCurrentTime - tStartTime;
    }

    // Retrieve IPS stats.
    double dAverage  = pIPS->GetAverageIPS();
    double dHigh     = pIPS->GetHighestIPS();
    double dLow      = pIPS->GetLowestIPS();
    double d1Percent = pIPS->Get1PercentLow();

    // Test that the returned average makes sense.
    EXPECT_TRUE(dAverage <= dHigh && dAverage >= dLow);
    // Test that the returned minimum makes sense.
    EXPECT_TRUE(dLow >= -1 && dLow <= dHigh);
    // Test that the returned maximum makes sense.
    EXPECT_TRUE(dHigh >= dLow);
    // Test that the returned 1% low makes sense.
    EXPECT_TRUE(d1Percent <= dHigh && d1Percent >= dLow);

    // Delete object.
    delete pIPS;
    // Point to null.
    pIPS = nullptr;
}

/******************************************************************************
 * @brief Test the functionality of the reset method.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
TEST(IPSTest, ResetFunction)
{
    // Create a new IPS object.
    IPS* pIPS = new IPS();

    // Call tick method, wait 16.666 ms, call tick method.
    pIPS->Tick();
    std::this_thread::sleep_for(std::chrono::microseconds(16666));
    pIPS->Tick();

    // Get the calculated values and check that they are not equal to zero.
    EXPECT_NE(pIPS->GetAverageIPS(), 0);
    EXPECT_NE(pIPS->GetHighestIPS(), 0);
    EXPECT_NE(pIPS->GetLowestIPS(), 9999999);
    EXPECT_NE(pIPS->Get1PercentLow(), 0);

    // Reset the counter.
    pIPS->Reset();

    // Get the calculated values and check that they are equal to zero.
    EXPECT_EQ(pIPS->GetAverageIPS(), 0);
    EXPECT_EQ(pIPS->GetHighestIPS(), 0);
    EXPECT_EQ(pIPS->GetLowestIPS(), 9999999);
    EXPECT_EQ(pIPS->Get1PercentLow(), 0);

    // Delete object.
    delete pIPS;
    // Point to null.
    pIPS = nullptr;
}
