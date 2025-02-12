/******************************************************************************
 * @brief Unit test for ObjectDetectionHandler handler class.
 *
 * @file ObjectDetectionHandler.cc
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 *
 * @copyright Copyright MRDT 2024 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/handlers/ObjectDetectionHandler.h"
#include "../../../../src/vision/objects/ObjectDetector.h"
#include "../../../TestingBase.hh"

/// \cond
#include <gmock/gmock.h>
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the ObjectDetectionHandler
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class ObjectDetectionHandlerTests : public TestingBase<ObjectDetectionHandlerTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new Object Detection Handler Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ObjectDetectionHandlerTests() {}

        /******************************************************************************
         * @brief Destroy the Object Detection Handler Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~ObjectDetectionHandlerTests() {}

        /******************************************************************************
         * @brief Setup the Object Detection Handler Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestSetup() override {}

        /******************************************************************************
         * @brief Teardown the Object Detection Handler Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestTeardown() override {}
};

// FIXME: Do not use "using" for namespaces, it's bad practice.
using ::testing::_;
using ::testing::NiceMock;
using ::testing::Return;

// /******************************************************************************
//  * @brief Test for memory leaks
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
// TEST_F(ObjectDetectionHandlerTests, DoesNotLeak) {
//     ObjectDetectionHandler* handler = new ObjectDetectionHandler();
//     delete handler;
// }
//
// /******************************************************************************
//  * @brief Mock class for ObjectDetector
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
// class MockObjectDetector : public ObjectDetector {
// public:
//     MOCK_METHOD(void, Start, (), (override));
//     MOCK_METHOD(void, Stop, (), (override));
// };

// /******************************************************************************
//  * @brief Test fixture for ObjectDetectionHandler
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
// class ObjectDetectionHandlerTests : public ::testing::Test {
// protected:
//     ObjectDetectionHandler* handler;
//     NiceMock<MockObjectDetector>* mockMainCam;
//     NiceMock<MockObjectDetector>* mockLeftCam;
//     NiceMock<MockObjectDetector>* mockRightCam;

//     void TestSetup() override {
//         mockMainCam = new NiceMock<MockObjectDetector>();
//         mockLeftCam = new NiceMock<MockObjectDetector>();
//         mockRightCam = new NiceMock<MockObjectDetector>();

//         handler = new ObjectDetectionHandler();
//         handler->m_pObjectDetectorMainCam = mockMainCam;
//         handler->m_pObjectDetectorLeftCam = mockLeftCam;
//         handler->m_pObjectDetectorRightCam = mockRightCam;
//     }

//     void TestTeardown() override {
//         delete handler;
//         delete mockMainCam;
//         delete mockLeftCam;
//         delete mockRightCam;
//     }
// };

// /******************************************************************************
//  * @brief Test that the constructor initializes the ObjectDetectionHandler members correctly
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
// TEST_F(ObjectDetectionHandlerTests, ConstructorInitializesMembers) {
//     EXPECT_NE(handler->m_pObjectDetectorMainCam, nullptr);
//     EXPECT_NE(handler->m_pObjectDetectorLeftCam, nullptr);
//     EXPECT_NE(handler->m_pObjectDetectorRightCam, nullptr);
// }

// /******************************************************************************
//  * @brief Test that the destructor deletes the ObjectDetectionHandler members correctly
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
// TEST_F(ObjectDetectionHandlerTests, StartAllDetectors) {
//     EXPECT_CALL(*mockMainCam, Start()).Times(1);
//     EXPECT_CALL(*mockLeftCam, Start()).Times(1);
//     EXPECT_CALL(*mockRightCam, Start()).Times(1);

//     handler->StartAllDetectors();
// }

// /******************************************************************************
//  * @brief Test that the destructor deletes the ObjectDetectionHandler members correctly
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
// TEST_F(ObjectDetectionHandlerTests, StopAllDetectors) {
//     EXPECT_CALL(*mockMainCam, Stop()).Times(1);
//     EXPECT_CALL(*mockLeftCam, Stop()).Times(1);
//     EXPECT_CALL(*mockRightCam, Stop()).Times(1);

//     handler->StopAllDetectors();
// }

// /******************************************************************************
//  * @brief Test that the GetObjectDetector method returns the correct ObjectDetector
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
// TEST_F(ObjectDetectionHandlerTests, GetObjectDetector) {
//     EXPECT_EQ(handler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eHeadMainCam), mockMainCam);
//     EXPECT_EQ(handler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eFrameLeftCam), mockLeftCam);
//     EXPECT_EQ(handler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eFrameRightCam), mockRightCam);
// }
