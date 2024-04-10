/******************************************************************************
 * @brief Unit test for PIDController algorithm class.
 *
 * @file PIDController.cc
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/algorithms/controllers/PIDController.h"

/// \cond
#include <array>
#include <chrono>
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Check that PIDController doesn't leak any memory.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-23
 ******************************************************************************/
TEST(PIDControllerTest, DoesNotLeak)
{
    // Create a new PIDController object.
    controllers::PIDController* pPIDController = new controllers::PIDController(1.0, 0.1, 0.1, 0.01);
    // Delete object.
    delete pPIDController;
    // Point to null.
    pPIDController = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-23
 ******************************************************************************/
TEST(PIDControllerTest, Leaks)
{
    // Create a new PIDController object.
    controllers::PIDController* pPIDController = new controllers::PIDController(1.0, 0.1, 0.1, 0.01);
    EXPECT_TRUE(pPIDController != nullptr);
}

/******************************************************************************
 * @brief Test PIDController proportional functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-23
 ******************************************************************************/
TEST(PIDControllerTest, ProportionalControl)
{
    // Create a new PIDController object.
    controllers::PIDController* pPIDController = new controllers::PIDController(1.0, 0.0, 0.0);

    // Create array for storing input and expect output values.
    const int nTestValuesLength                     = 5;
    const double aActualInput[nTestValuesLength]    = {0.0, 1.0, 2.0, 3.0, -50.0};
    const double aSetpointInput[nTestValuesLength]  = {0.0, 1.0, 1.0, 1.0, -49.5};
    const double aExpectedOutput[nTestValuesLength] = {0.0, 0.0, -1.0, -2.0, 0.5};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate drive powers.
        double dOutput = pPIDController->Calculate(aActualInput[nIter], aSetpointInput[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_NEAR(dOutput, aExpectedOutput[nIter], 0.01);    // Left output check.
    }

    // Delete object.
    delete pPIDController;
    // Point to null.
    pPIDController = nullptr;
}

/******************************************************************************
 * @brief Test PIDController integral functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-23
 ******************************************************************************/
TEST(PIDControllerTest, IntegralControl)
{
    // Create a new PIDController object.
    controllers::PIDController* pPIDController = new controllers::PIDController(0.0, 1.0, 0.0);

    // Create array for storing input and expect output values.
    const int nTestValuesLength                     = 5;
    const double aActualInput[nTestValuesLength]    = {0.0, 1.0, 2.0, 3.0, -50.0};
    const double aSetpointInput[nTestValuesLength]  = {0.0, 1.0, 1.0, 1.0, -49.5};
    const double aExpectedOutput[nTestValuesLength] = {0.0, 0.0, -1.0, -4.0, -5.5};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate drive powers a few times.
        double dOutput = pPIDController->Calculate(aActualInput[nIter], aSetpointInput[nIter]);
        dOutput        = pPIDController->Calculate(aActualInput[nIter], aSetpointInput[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_NEAR(dOutput, aExpectedOutput[nIter], 0.01);    // Left output check.
    }

    // Delete object.
    delete pPIDController;
    // Point to null.
    pPIDController = nullptr;
}

/******************************************************************************
 * @brief Test PIDController derivative functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-23
 ******************************************************************************/
TEST(PIDControllerTest, DerivativeControl)
{
    // Create a new PIDController object.
    controllers::PIDController* pPIDController = new controllers::PIDController(0.0, 0.0, 1.0);

    // Create array for storing input and expect output values.
    const int nTestValuesLength                     = 5;
    const double aActualInput[nTestValuesLength]    = {0.0, 1.0, 2.0, 3.0, -50.0};
    const double aSetpointInput[nTestValuesLength]  = {0.0, 1.0, 1.0, 1.0, -49.5};
    const double aExpectedOutput[nTestValuesLength] = {0.0, -1.0, -1.0, -1.0, 53.0};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Get last iterator index.
        int nLastIter;
        ((nIter - 1) <= 0) ? nLastIter = 0 : nLastIter = (nIter - 1);

        // Calculate drive powers a few times.
        double dOutput = pPIDController->Calculate(aActualInput[nLastIter], aSetpointInput[nLastIter]);
        dOutput        = pPIDController->Calculate(aActualInput[nIter], aSetpointInput[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_NEAR(dOutput, aExpectedOutput[nIter], 0.01);    // Left output check.
    }

    // Delete object.
    delete pPIDController;
    // Point to null.
    pPIDController = nullptr;
}
