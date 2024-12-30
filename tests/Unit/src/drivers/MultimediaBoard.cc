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

/// \cond
#include <gmock/gmock.h>
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Test for memory leaks
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST(MultimediaBoardTest, DoesNotLeak)
{
    MultimediaBoard* multimediaBoard = new MultimediaBoard();
    ASSERT_NE(multimediaBoard, nullptr);
    delete multimediaBoard;
    multimediaBoard = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST(MultimediaBoardTest, Leaks)
{
    MultimediaBoard* multimediaBoard = new MultimediaBoard();
    EXPECT_TRUE(multimediaBoard != nullptr);
}

// /******************************************************************************
//  * @brief Mock class for MultimediaBoard
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
// Can only use with TEST_F. Since we are not using TEST_F, we can't use this.
// class MultimediaBoardTest : public ::testing::Test
// {
//     protected:
//         MultimediaBoard* multimediaBoard;

//         void SetUp() override { multimediaBoard = new MultimediaBoard(); }

//         void TearDown() override { delete multimediaBoard; }
// };

// /******************************************************************************
//  * @brief Test that the constructor initializes the lighting state and RGB values correctly
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
TEST(MultimediaBoardTest, ConstructorInitializesCorrectly)
{
    MultimediaBoard* multimediaBoard = new MultimediaBoard();

    EXPECT_EQ(multimediaBoard->GetCurrentLightingState(), MultimediaBoard::MultimediaBoardLightingState::eOff);
    MultimediaBoard::RGB defaultRGB;
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dRed, defaultRGB.dRed);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dGreen, defaultRGB.dGreen);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dBlue, defaultRGB.dBlue);

    delete multimediaBoard;
}

// /******************************************************************************
//  * @brief Test that the lighting state is set correctly
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
TEST(MultimediaBoardTest, SendLightingStateSetsStateCorrectly)
{
    MultimediaBoard* multimediaBoard = new MultimediaBoard();

    multimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eTeleOp);
    EXPECT_EQ(multimediaBoard->GetCurrentLightingState(), MultimediaBoard::MultimediaBoardLightingState::eTeleOp);

    multimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
    EXPECT_EQ(multimediaBoard->GetCurrentLightingState(), MultimediaBoard::MultimediaBoardLightingState::eAutonomy);

    multimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
    EXPECT_EQ(multimediaBoard->GetCurrentLightingState(), MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);

    delete multimediaBoard;
}

// /******************************************************************************
//  * @brief Test that the RGB values are set correctly
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
TEST(MultimediaBoardTest, SendRGBSetsRGBValuesCorrectly)
{
    MultimediaBoard* multimediaBoard = new MultimediaBoard();

    MultimediaBoard::RGB rgbValues(255, 128, 64);
    multimediaBoard->SendRGB(rgbValues);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dRed, rgbValues.dRed);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dGreen, rgbValues.dGreen);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dBlue, rgbValues.dBlue);

    delete multimediaBoard;
}
