/******************************************************************************
 * @brief Unit test for NumberOperations utility class.
 *
 * @file NumberOperations.cc
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/util/NumberOperations.hpp"

/// \cond
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

/// \endcond

/******************************************************************************
 * @brief Test the functionality of the Clamp function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-16
 ******************************************************************************/
TEST(NumOpsTest, Clamp)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength               = 6;
    const double aValues[nTestValuesLength]   = {1.0, 0.0, 567.0, 0.05, -1.0, -89.3};
    const double aMinimums[nTestValuesLength] = {0.0, 0.0, -5.0, 0.05, -3.0, 50.0};
    const double aMaximums[nTestValuesLength] = {2.0, 1.0, 0.0, 0.04, 0.0, -100.0};
    const double aOutput[nTestValuesLength]   = {1.0, 0.0, 0.0, 0.05, -1.0, 50.0};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate clamp values.
        double dResult = numops::Clamp(aValues[nIter], aMinimums[nIter], aMaximums[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_EQ(dResult, aOutput[nIter]);    // output check.
    }
}

/******************************************************************************
 * @brief Test the functionality of the Bounded function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-16
 ******************************************************************************/
TEST(NumOpsTest, Bounded)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength               = 6;
    const double aValues[nTestValuesLength]   = {1.0, 0.0, 567.0, 0.05, -1.0, -89.3};
    const double aMinimums[nTestValuesLength] = {0.0, 0.0, -5.0, 0.05, -3.0, 50.0};
    const double aMaximums[nTestValuesLength] = {2.0, 1.0, 0.0, 0.04, 0.0, -100.0};
    const bool aOutput[nTestValuesLength]     = {true, true, false, false, true, false};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate valid bounds.
        bool bResult = numops::Bounded(aValues[nIter], aMinimums[nIter], aMaximums[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_EQ(bResult, aOutput[nIter]);    // output check.
    }
}

/******************************************************************************
 * @brief Test the functionality of the MapRange function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
TEST(NumOpsTest, MapRange)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength                  = 4;
    const double aValues[nTestValuesLength]      = {0.0, -1.0, 2.0, 0};
    const double aOldMinimums[nTestValuesLength] = {-1.0, -1.0, -4.0, -180};
    const double aOldMaximums[nTestValuesLength] = {1.0, 1.0, 4.0, 180};
    const double aNewMinimums[nTestValuesLength] = {0.0, 0.0, -2.0, 0};
    const double aNewMaximums[nTestValuesLength] = {2.0, 2.0, 2.0, 360};
    const double aOutput[nTestValuesLength]      = {1.0, 0.0, 1.0, 180};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate valid bounds.
        double dResult = numops::MapRange(aValues[nIter], aOldMinimums[nIter], aOldMaximums[nIter], aNewMinimums[nIter], aNewMaximums[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_EQ(dResult, aOutput[nIter]);
    }
}

/******************************************************************************
 * @brief Test the functionality of the InputAngleModulus function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-19
 ******************************************************************************/
TEST(NumOpsTest, InputAngleModulus)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength               = 9;
    const double aValues[nTestValuesLength]   = {1.0, -1.0, 4.0, 360.0, 350.0, 170.0, -90, 360.0, -90};
    const double aMinimums[nTestValuesLength] = {0.0, 0.0, -2.0, -180.0, -180.0, -180.0, 0.0, 0.0, 0.0};
    const double aMaximums[nTestValuesLength] = {2.0, 2.0, 2.0, 180.0, 180.0, 180.0, 360.0, 360.0, 360.0};
    const double aOutput[nTestValuesLength]   = {1.0, 1.0, 0.0, 0.0, -10.0, 170.0, 270.0, 0.0, 270.0};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate valid bounds.
        double dResult = numops::InputAngleModulus(aValues[nIter], aMinimums[nIter], aMaximums[nIter]);
        // Check that the expected output values were calculated.
        EXPECT_EQ(dResult, aOutput[nIter]);
    }
}

/******************************************************************************
 * @brief Test the functionality of the AngularDifference function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2024-04-03
 ******************************************************************************/
TEST(NumOpsTest, AngularDifference)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength                   = 8;
    const double aFirstValues[nTestValuesLength]  = {0.0, 330.0, 30.0, 270.0, 60.0, 0.0, 170.0, 60.0};
    const double aSecondValues[nTestValuesLength] = {0.0, 30.0, 330.0, 180.0, 120.0, 360.0, 190.0, 90.0};
    const double aOutput[nTestValuesLength]       = {0.0, 60.0, -60.0, 90.0, 60.0, 0.0, 20.0, 30.0};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate valid bounds.
        double dResult = numops::AngularDifference(aFirstValues[nIter], aSecondValues[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_EQ(dResult, aOutput[nIter]);
    }
}

/******************************************************************************
 * @brief Test the functionality of the CoordinateFrameRotate3D function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2024-04-03
 ******************************************************************************/
TEST(NumOpsTest, CoordinateFrameRotate3D)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength                = 7;
    const double aX[nTestValuesLength]         = {15.0, 1.0, 1.0, 1.0, 5.0, 10.0, 420000.0};
    const double aY[nTestValuesLength]         = {10.0, 0.0, 0.0, 0.0, 5.0, 10.0, 315.0};
    const double aZ[nTestValuesLength]         = {40.0, 0.0, 0.0, 0.0, 5.0, 10.0, -65780.0};
    const double aXRot[nTestValuesLength]      = {0.0, 90.0, 0.0, 0.0, 90.0, 90.0, 0.0};
    const double aYRot[nTestValuesLength]      = {0.0, 0.0, 90.0, 0.0, 90.0, 45.0, 65.4};
    const double aZRot[nTestValuesLength]      = {0.0, 0.0, 0.0, 90.0, 90.0, 30.0, 0.0};
    const double aExpectedX[nTestValuesLength] = {15.0, 1.0, 0.0, 0.0, 5.0, 17.25, 115028.38};
    const double aExpectedY[nTestValuesLength] = {10.0, 0.0, 0.0, 1.0, 5.0, -1.59, 315.0};
    const double aExpectedZ[nTestValuesLength] = {40.0, 0.0, -1.0, 0.0, -5.0, 0.0, -409262.11};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Pack point cloud vector.
        std::vector<numops::CoordinatePoint<double>> vPointCloud;
        vPointCloud.emplace_back(numops::CoordinatePoint(aX[nIter], aY[nIter], aZ[nIter]));

        // Calculate new rotated point.
        numops::CoordinateFrameRotate3D(vPointCloud, aXRot[nIter], aYRot[nIter], aZRot[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_NEAR(vPointCloud[0].tX, aExpectedX[nIter], 0.01);
        EXPECT_NEAR(vPointCloud[0].tY, aExpectedY[nIter], 0.01);
        EXPECT_NEAR(vPointCloud[0].tZ, aExpectedZ[nIter], 0.01);
    }
}
