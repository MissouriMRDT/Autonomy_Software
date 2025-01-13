/******************************************************************************
 * @brief Unit test for MultimediaBoard driver class.
 *
 * @file MultimediaBoard.cc
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 *
 * @copyright Copyright MRDT 2024 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/drivers/MultimediaBoard.h"
#include "../../../TestingBase.hh"

/// \cond
#include <gmock/gmock.h>
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the MultimediaBoard
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class MultimediaBoardTests : public TestingBase<MultimediaBoardTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new Multimedia Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        MultimediaBoardTests() { SetUp(); }

        /******************************************************************************
         * @brief Destroy the Multimedia Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~MultimediaBoardTests() { TearDown(); }

        /******************************************************************************
         * @brief Setup the Multimedia Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void SetUp() override
        {
            // Call the base setup method. This initializes the loggers and RoveComm instances.
            RequiredSetup();
        }

        /******************************************************************************
         * @brief Teardown the Multimedia Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TearDown() override
        {
            // Call the base teardown method. This stops the RoveComm instances and loggers.
            RequiredTeardown();
        }
};

/******************************************************************************
 * @brief Test for memory leaks
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(MultimediaBoardTests, DoesNotLeak)
{
    MultimediaBoard* testBoard = new MultimediaBoard();
    ASSERT_NE(testBoard, nullptr);
    delete testBoard;
    testBoard = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(MultimediaBoardTests, Leaks)
{
    MultimediaBoard* testBoard = new MultimediaBoard();
    EXPECT_NE(testBoard, nullptr);
    // Intentionally not deleting to test leak detection
}


// /******************************************************************************
//  * @brief Test that the constructor initializes the lighting state and RGB values correctly
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
TEST_F(MultimediaBoardTest, ConstructorInitializesCorrectly) 
{
    EXPECT_EQ(multimediaBoard->GetCurrentLightingState(), 
              MultimediaBoard::MultimediaBoardLightingState::eOff);
    
    MultimediaBoard::RGB defaultRGB;
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dRed, defaultRGB.dRed);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dGreen, defaultRGB.dGreen);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dBlue, defaultRGB.dBlue);
}

// /******************************************************************************
//  * @brief Test that the lighting state is set correctly
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
TEST_F(MultimediaBoardTest, SendLightingStateSetsStateCorrectly)
{
    multimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eTeleOp);
    EXPECT_EQ(multimediaBoard->GetCurrentLightingState(), 
              MultimediaBoard::MultimediaBoardLightingState::eTeleOp);

    multimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
    EXPECT_EQ(multimediaBoard->GetCurrentLightingState(),
              MultimediaBoard::MultimediaBoardLightingState::eAutonomy);

    multimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
    EXPECT_EQ(multimediaBoard->GetCurrentLightingState(),
              MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
}

// /******************************************************************************
//  * @brief Test that the RGB values are set correctly
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
TEST_F(MultimediaBoardTest, SendRGBSetsRGBValuesCorrectly)
{
    MultimediaBoard::RGB rgbValues(255, 128, 64);
    multimediaBoard->SendRGB(rgbValues);
    
    auto values = multimediaBoard->GetCustomLightingValues();
    EXPECT_EQ(values.dRed, rgbValues.dRed);
    EXPECT_EQ(values.dGreen, rgbValues.dGreen);
    EXPECT_EQ(values.dBlue, rgbValues.dBlue);
}
