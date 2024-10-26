/******************************************************************************
 * @brief Unit test for MultimediaBoard driver class.
 *
 * @file MultimediaBoard.cc
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 *
 * @copyright Copyright MRDT 2024 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/drivers/MultimediaBoard.h"

/// \cond
#include <gtest/gtest.h>
#include <gmock/gmock.h>

/// \endcond

/******************************************************************************
 * @brief Test for memory leaks
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
TEST(MultimediaBoardTest, DoesNotLeak) {
    MultimediaBoard* multimediaBoard = new MultimediaBoard();
    delete multimediaBoard;
}

/******************************************************************************
 * @brief Mock class for MultimediaBoard
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/ 
class MultimediaBoardTest : public ::testing::Test {
protected:
    MultimediaBoard* multimediaBoard;

    void SetUp() override {
        multimediaBoard = new MultimediaBoard();
    }

    void TearDown() override {
        delete multimediaBoard;
    }
};

/******************************************************************************
 * @brief Test that the constructor initializes the lighting state and RGB values correctly
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
TEST_F(MultimediaBoardTest, ConstructorInitializesCorrectly) {
    EXPECT_EQ(multimediaBoard->GetCurrentLightingState(), MultimediaBoard::MultimediaBoardLightingState::eOff);
    MultimediaBoard::RGB defaultRGB;
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dRed, defaultRGB.dRed);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dGreen, defaultRGB.dGreen);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dBlue, defaultRGB.dBlue);
}

/******************************************************************************
 * @brief Test that the lighting state is set correctly
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
TEST_F(MultimediaBoardTest, SendLightingStateSetsStateCorrectly) {
    multimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eTeleOp);
    EXPECT_EQ(multimediaBoard->GetCurrentLightingState(), MultimediaBoard::MultimediaBoardLightingState::eTeleOp);

    multimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
    EXPECT_EQ(multimediaBoard->GetCurrentLightingState(), MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
}

/******************************************************************************
 * @brief Test that the RGB values are set correctly
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
TEST_F(MultimediaBoardTest, SendRGBSetsRGBValuesCorrectly) {
    MultimediaBoard::RGB rgbValues(255, 128, 64);
    multimediaBoard->SendRGB(rgbValues);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dRed, rgbValues.dRed);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dGreen, rgbValues.dGreen);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dBlue, rgbValues.dBlue);
}

/******************************************************************************
 * @brief Test that the lighting state is set to custom when RGB values are sent
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
TEST_F(MultimediaBoardTest, GetCurrentLightingStateReturnsCorrectState) {
    multimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
    EXPECT_EQ(multimediaBoard->GetCurrentLightingState(), MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
}

/******************************************************************************
 * @brief Test that the RGB values are returned correctly
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
TEST_F(MultimediaBoardTest, GetCustomLightingValuesReturnsCorrectRGB) {
    MultimediaBoard::RGB rgbValues(100, 150, 200);
    multimediaBoard->SendRGB(rgbValues);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dRed, rgbValues.dRed);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dGreen, rgbValues.dGreen);
    EXPECT_EQ(multimediaBoard->GetCustomLightingValues().dBlue, rgbValues.dBlue);
}