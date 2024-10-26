/******************************************************************************
 * @brief Unit test for ObjectDetectionHandler handler class.
 *
 * @file ObjectDetectionHandler.cc
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 *
 * @copyright Copyright MRDT 2024 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/handlers/ObjectDetectionHandler.h"
#include "../../../../../src/vision/objects/ObjectDetector.h"

/// \cond
#include <gtest/gtest.h>
#include <gmock/gmock.h>

/// \endcond

using ::testing::_;
using ::testing::Return;
using ::testing::NiceMock;

/******************************************************************************
 * @brief Test for memory leaks
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
TEST(ObjectDetectionHandlerTest, DoesNotLeak) {
    ObjectDetectionHandler* handler = new ObjectDetectionHandler();
    delete handler;
}

/******************************************************************************
 * @brief Mock class for ObjectDetector
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
class MockObjectDetector : public ObjectDetector {
public:
    MOCK_METHOD(void, Start, (), (override));
    MOCK_METHOD(void, Stop, (), (override));
};

/******************************************************************************
 * @brief Test fixture for ObjectDetectionHandler
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
class ObjectDetectionHandlerTest : public ::testing::Test {
protected:
    ObjectDetectionHandler* handler;
    NiceMock<MockObjectDetector>* mockMainCam;
    NiceMock<MockObjectDetector>* mockLeftCam;
    NiceMock<MockObjectDetector>* mockRightCam;

    void SetUp() override {
        mockMainCam = new NiceMock<MockObjectDetector>();
        mockLeftCam = new NiceMock<MockObjectDetector>();
        mockRightCam = new NiceMock<MockObjectDetector>();

        handler = new ObjectDetectionHandler();
        handler->m_pObjectDetectorMainCam = mockMainCam;
        handler->m_pObjectDetectorLeftCam = mockLeftCam;
        handler->m_pObjectDetectorRightCam = mockRightCam;
    }

    void TearDown() override {
        delete handler;
        delete mockMainCam;
        delete mockLeftCam;
        delete mockRightCam;
    }
};

/******************************************************************************
 * @brief Test that the constructor initializes the ObjectDetectionHandler members correctly
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
TEST_F(ObjectDetectionHandlerTest, ConstructorInitializesMembers) {
    EXPECT_NE(handler->m_pObjectDetectorMainCam, nullptr);
    EXPECT_NE(handler->m_pObjectDetectorLeftCam, nullptr);
    EXPECT_NE(handler->m_pObjectDetectorRightCam, nullptr);
}

/******************************************************************************
 * @brief Test that the destructor deletes the ObjectDetectionHandler members correctly
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
TEST_F(ObjectDetectionHandlerTest, StartAllDetectors) {
    EXPECT_CALL(*mockMainCam, Start()).Times(1);
    EXPECT_CALL(*mockLeftCam, Start()).Times(1);
    EXPECT_CALL(*mockRightCam, Start()).Times(1);

    handler->StartAllDetectors();
}

/******************************************************************************
 * @brief Test that the destructor deletes the ObjectDetectionHandler members correctly
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
TEST_F(ObjectDetectionHandlerTest, StopAllDetectors) {
    EXPECT_CALL(*mockMainCam, Stop()).Times(1);
    EXPECT_CALL(*mockLeftCam, Stop()).Times(1);
    EXPECT_CALL(*mockRightCam, Stop()).Times(1);

    handler->StopAllDetectors();
}

/******************************************************************************
 * @brief Test that the GetObjectDetector method returns the correct ObjectDetector
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
TEST_F(ObjectDetectionHandlerTest, GetObjectDetector) {
    EXPECT_EQ(handler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eHeadMainCam), mockMainCam);
    EXPECT_EQ(handler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eFrameLeftCam), mockLeftCam);
    EXPECT_EQ(handler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eFrameRightCam), mockRightCam);
}