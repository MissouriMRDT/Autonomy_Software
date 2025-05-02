/******************************************************************************
 * @brief Unit test for MultimediaBoard driver class.
 *
 * @file MultimediaBoard.cc
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
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
        std::unique_ptr<MultimediaBoard> m_pMultimediaBoard;

    public:
        /******************************************************************************
         * @brief Construct a new Multimedia Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        MultimediaBoardTests() {}

        /******************************************************************************
         * @brief Destroy the Multimedia Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~MultimediaBoardTests() {}

        /******************************************************************************
         * @brief Setup the Multimedia Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestSetup() override { m_pMultimediaBoard = std::make_unique<MultimediaBoard>(); }

        /******************************************************************************
         * @brief Teardown the Multimedia Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestTeardown() override {}
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
    MultimediaBoard* pTestBoard = new MultimediaBoard();
    ASSERT_NE(pTestBoard, nullptr);
    delete pTestBoard;
    pTestBoard = nullptr;
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
    MultimediaBoard* pTestBoard = new MultimediaBoard();
    EXPECT_NE(pTestBoard, nullptr);
    // Intentionally not deleting to test leak detection
}

/******************************************************************************
 * @brief Test that the constructor initializes the lighting state and RGB values correctly
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(MultimediaBoardTests, ConstructorInitializesCorrectly)
{
    EXPECT_EQ(m_pMultimediaBoard->GetCurrentLightingState(), MultimediaBoard::MultimediaBoardLightingState::eOff);

    MultimediaBoard::RGB stDefaultRGB;
    EXPECT_EQ(m_pMultimediaBoard->GetCustomLightingValues().dRed, stDefaultRGB.dRed);
    EXPECT_EQ(m_pMultimediaBoard->GetCustomLightingValues().dGreen, stDefaultRGB.dGreen);
    EXPECT_EQ(m_pMultimediaBoard->GetCustomLightingValues().dBlue, stDefaultRGB.dBlue);
}

/******************************************************************************
 * @brief Test that the lighting state is set correctly
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(MultimediaBoardTests, SendLightingStateSetsStateCorrectly)
{
    m_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eTeleOp);
    EXPECT_EQ(m_pMultimediaBoard->GetCurrentLightingState(), MultimediaBoard::MultimediaBoardLightingState::eTeleOp);

    m_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
    EXPECT_EQ(m_pMultimediaBoard->GetCurrentLightingState(), MultimediaBoard::MultimediaBoardLightingState::eAutonomy);

    m_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
    EXPECT_EQ(m_pMultimediaBoard->GetCurrentLightingState(), MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
}

/******************************************************************************
 * @brief Test that the RGB values are set correctly
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(MultimediaBoardTests, SendRGBSetsRGBValuesCorrectly)
{
    MultimediaBoard::RGB stCurrentRGBValues(255, 128, 64);
    m_pMultimediaBoard->SendRGB(stCurrentRGBValues);

    MultimediaBoard::RGB stRGBValues = m_pMultimediaBoard->GetCustomLightingValues();
    EXPECT_EQ(stRGBValues.dRed, stCurrentRGBValues.dRed);
    EXPECT_EQ(stRGBValues.dGreen, stCurrentRGBValues.dGreen);
    EXPECT_EQ(stRGBValues.dBlue, stCurrentRGBValues.dBlue);
}
