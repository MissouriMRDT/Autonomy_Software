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
#include "../../../../../src/AutonomyGlobals.h"
#include "../../../../TestingBase.hh"

/// \cond
#include <array>
#include <chrono>
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the PIDController
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class PIDControllerTests : public TestingBase<PIDControllerTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new PIDControllerTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        PIDControllerTests() {}

        /******************************************************************************
         * @brief Destroy the PIDControllerTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~PIDControllerTests() {}

        /******************************************************************************
         * @brief Setup the PIDControllerTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestSetup() override {}

        /******************************************************************************
         * @brief Teardown the PIDControllerTests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Check that PIDController doesn't leak any memory.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-23
 ******************************************************************************/
TEST_F(PIDControllerTests, DoesNotLeak)
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
TEST_F(PIDControllerTests, Leaks)
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
TEST_F(PIDControllerTests, ProportionalControl)
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
        dOutput        = pPIDController->Calculate(aActualInput[nIter]);
        dOutput        = pPIDController->Calculate();

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
TEST_F(PIDControllerTests, IntegralControl)
{
    // Create a new PIDController object.
    controllers::PIDController* pPIDController = new controllers::PIDController(0.0, 1.0, 0.0);

    // Create array for storing input and expect output values.
    const int nTestValuesLength                     = 5;
    const double aActualInput[nTestValuesLength]    = {0.0, 1.0, 2.0, 3.0, -50.0};
    const double aSetpointInput[nTestValuesLength]  = {0.0, 1.0, 1.0, 1.0, -49.5};
    const double aExpectedOutput[nTestValuesLength] = {0.0, 0.0, -2.0, -7.0, -8.0};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate drive powers a few times.
        double dOutput = pPIDController->Calculate(aActualInput[nIter], aSetpointInput[nIter]);
        dOutput        = pPIDController->Calculate(aActualInput[nIter]);
        dOutput        = pPIDController->Calculate();

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
TEST_F(PIDControllerTests, DerivativeControl)
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

/******************************************************************************
 * @brief Test PIDController limits adherence.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-11-24
 ******************************************************************************/
TEST_F(PIDControllerTests, ControllerLimits)
{
    // Create a new PIDController object.
    controllers::PIDController* pPIDController = new controllers::PIDController(1.0, 0.0, 0.0);
    // Set controller limits.
    pPIDController->SetMaxSetpointDifference(0.1);
    pPIDController->SetMaxIntegralEffort(1);
    pPIDController->SetOutputLimits(0.1);
    pPIDController->SetOutputRampRate(0.01);

    // Create array for storing input and expect output values.
    const int nTestValuesLength                     = 5;
    const double aActualInput[nTestValuesLength]    = {0.0, 1.0, 2.0, 3.0, -50.0};
    const double aSetpointInput[nTestValuesLength]  = {0.0, 1.0, 1.0, 1.0, -49.5};
    const double aExpectedOutput[nTestValuesLength] = {0.0, -0.01, -0.01, -0.02, -0.01};

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
 * @brief Test PIDController wrap around for rotational positioning.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-11-24
 ******************************************************************************/
TEST_F(PIDControllerTests, ContinuousInput)
{
    // Create a new PIDController object.
    controllers::PIDController* pPIDController = new controllers::PIDController(1.0, 0.0, 0.0);
    // Enable continuous input.
    pPIDController->EnableContinuousInput(-180.0, 180.0);

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

    // Disable continuous input.
    pPIDController->DisableContinuousInput();
    // Reset controller.
    pPIDController->Reset();

    // Create array for storing input and expect output values.
    const double aExpectedOutput2[nTestValuesLength] = {0.0, 0.0, -1.0, -2.0, 0.5};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate drive powers.
        double dOutput = pPIDController->Calculate(aActualInput[nIter], aSetpointInput[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_NEAR(dOutput, aExpectedOutput2[nIter], 0.01);    // Left output check.
    }

    // Delete object.
    delete pPIDController;
    // Point to null.
    pPIDController = nullptr;
}

/******************************************************************************
 * @brief Test PIDController mutators and accessors.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-11-25
 ******************************************************************************/
TEST_F(PIDControllerTests, MutatorsAndAccessors)
{
    // Create a new PIDController object.
    controllers::PIDController* pPIDController = new controllers::PIDController(1.0, 0.0, 0.0);

    ////////////////////////////////////////////////////////////////////////////
    // Test mutators and then see if accessors return the same values.
    ////////////////////////////////////////////////////////////////////////////

    // PID gains.
    pPIDController->SetProportional(2.0);
    EXPECT_NEAR(pPIDController->GetProportional(), 2.0, 0.01);
    pPIDController->SetIntegral(3.0);
    EXPECT_NEAR(pPIDController->GetIntegral(), 3.0, 0.01);
    pPIDController->SetDerivative(4.0);
    EXPECT_NEAR(pPIDController->GetDerivative(), 4.0, 0.01);
    pPIDController->SetFeedforward(5.0);
    EXPECT_NEAR(pPIDController->GetFeedforward(), 5.0, 0.01);
    pPIDController->SetPID(6.0, 7.0, 8.0);
    EXPECT_NEAR(pPIDController->GetProportional(), 6.0, 0.01);
    EXPECT_NEAR(pPIDController->GetIntegral(), 7.0, 0.01);
    EXPECT_NEAR(pPIDController->GetDerivative(), 8.0, 0.01);
    pPIDController->SetPID(9.0, 10.0, 11.0, 12.0);
    EXPECT_NEAR(pPIDController->GetProportional(), 9.0, 0.01);
    EXPECT_NEAR(pPIDController->GetIntegral(), 10.0, 0.01);
    EXPECT_NEAR(pPIDController->GetDerivative(), 11.0, 0.01);
    EXPECT_NEAR(pPIDController->GetFeedforward(), 12.0, 0.01);

    // Other settings.
    pPIDController->SetSetpoint(6.0);
    EXPECT_NEAR(pPIDController->GetSetpoint(), 6.0, 0.01);
    pPIDController->SetMaxSetpointDifference(7.0);
    EXPECT_NEAR(pPIDController->GetMaxSetpointDifference(), 7.0, 0.01);
    pPIDController->SetMaxIntegralEffort(8.0);
    EXPECT_NEAR(pPIDController->GetMaxIntegralEffort(), 8.0, 0.01);
    pPIDController->SetOutputRampRate(9.0);
    EXPECT_NEAR(pPIDController->GetOutputRampRate(), 9.0, 0.01);
    pPIDController->SetOutputFilter(0.1);
    EXPECT_NEAR(pPIDController->GetOutputFilter(), 0.1, 0.01);
    pPIDController->SetDirection(true);
    EXPECT_TRUE(pPIDController->GetReversed());

    // No accessors for these.
    pPIDController->SetOutputLimits(11.0, 12.0);
    pPIDController->SetOutputLimits(13.0);

    // Delete object.
    delete pPIDController;
    // Point to null.
    pPIDController = nullptr;
}
