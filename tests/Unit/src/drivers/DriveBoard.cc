/******************************************************************************
 * @brief Unit test for DriveBoard driver class.
 *
 * @file DriveBoard.cc
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 *
 * @copyright Copyright MRDT 2024 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/drivers/DriveBoard.h"
#include "../../../TestingBase.hh"

/// \cond
#include "../../../../external/rovecomm/src/RoveComm/RoveComm.h"
#include "../../../../external/rovecomm/src/RoveComm/RoveCommManifest.h"
#include "../../../../external/rovecomm/src/RoveComm/RoveCommUDP.h"
#include "../../../../src/AutonomyConstants.h"
#include "../../../../src/AutonomyGlobals.h"
#include "../../../../src/AutonomyLogging.h"
#include "../../../../src/AutonomyNetworking.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the DriveBoard
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class DriveBoardTests : public TestingBase<DriveBoardTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new Drive Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        DriveBoardTests() {}

        /******************************************************************************
         * @brief Destroy the Drive Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~DriveBoardTests() {}

        /******************************************************************************
         * @brief Setup the Drive Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestSetup() override {}

        /******************************************************************************
         * @brief Teardown the Drive Board Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Test for memory leaks
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(DriveBoardTests, DoesNotLeak)
{
    DriveBoard* driveBoard = new DriveBoard();
    ASSERT_NE(driveBoard, nullptr);
    delete driveBoard;
    driveBoard = nullptr;
}

/******************************************************************************
 * @brief This should fail when the --check_for_leaks command line flag is specified.
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(DriveBoardTests, Leaks)
{
    DriveBoard* driveBoard = new DriveBoard();
    EXPECT_TRUE(driveBoard != nullptr);
}

// /******************************************************************************
//  * @brief Mock class for RoveCommUDPNode
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// class MockRoveCommUDPNode : public network::RoveCommUDPNode {
// public:
//     MOCK_METHOD(void, SendUDPPacket, (const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port), (override));
// };

// /******************************************************************************
//  * @brief Test fixture for DriveBoard
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// class DriveBoardTests : public ::testing::Test {
// protected:
//     // Create DriveBoard and MockRoveCommUDPNode objects.
//     DriveBoard* driveBoard;
//     MockRoveCommUDPNode* mockRoveCommUDPNode;

//     // Set up the test fixture.
//     void TestSetup() override {
//         // Create objects.
//         mockRoveCommUDPNode = new MockRoveCommUDPNode();
//         network::g_pRoveCommUDPNode = mockRoveCommUDPNode;
//         driveBoard = new DriveBoard();
//     }

//     // Tear down the test fixture.
//     void TestTeardown() override {
//         delete driveBoard;
//         delete mockRoveCommUDPNode;
//     }
// };

// /******************************************************************************
//  * @brief Test SendDrive with normal input values
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(DriveBoardTests, SendDrive_NormalInput) {
//     diffdrive::DrivePowers drivePowers = {0.5, -0.5};

//     EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
//         .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
//             EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
//             EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
//             EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
//             EXPECT_EQ(packet.vData[0], 0.5);
//             EXPECT_EQ(packet.vData[1], -0.5);
//             EXPECT_STREQ(ipAddress, constants::MODE_SIM ? constants::SIM_IP_ADDRESS.c_str() : manifest::Core::IP_ADDRESS.IP_STR.c_str());
//             EXPECT_EQ(port, constants::ROVECOMM_OUTGOING_UDP_PORT);
//         });

//     driveBoard->SendDrive(drivePowers);
// }

// /******************************************************************************
//  * @brief Test SendDrive with input values out of range
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(DriveBoardTests, SendDrive_OutOfRangeInput) {
//     diffdrive::DrivePowers drivePowers = {2.0, -2.0};

//     EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
//         .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
//             EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
//             EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
//             EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
//             EXPECT_EQ(packet.vData[0], 1.0);
//             EXPECT_EQ(packet.vData[1], -1.0);
//             EXPECT_STREQ(ipAddress, constants::MODE_SIM ? constants::SIM_IP_ADDRESS.c_str() : manifest::Core::IP_ADDRESS.IP_STR.c_str());
//             EXPECT_EQ(port, constants::ROVECOMM_OUTGOING_UDP_PORT);
//         });

//     driveBoard->SendDrive(drivePowers);
// }

// /******************************************************************************
//  * @brief Test SendDrive with minimum input values
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(DriveBoardTests, SendDrive_MinInput) {
//     diffdrive::DrivePowers drivePowers = {-1.0, -1.0};

//     EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
//         .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
//             EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
//             EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
//             EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
//             EXPECT_EQ(packet.vData[0], -1.0);
//             EXPECT_EQ(packet.vData[1], -1.0);
//             EXPECT_STREQ(ipAddress, constants::MODE_SIM ? constants::SIM_IP_ADDRESS.c_str() : manifest::Core::IP_ADDRESS.IP_STR.c_str());
//             EXPECT_EQ(port, constants::ROVECOMM_OUTGOING_UDP_PORT);
//         });

//     driveBoard->SendDrive(drivePowers);
// }

// /******************************************************************************
//  * @brief Test SendDrive with maximum input values
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(DriveBoardTests, SendDrive_MaxInput) {
//     diffdrive::DrivePowers drivePowers = {1.0, 1.0};

//     EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
//         .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
//             EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
//             EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
//             EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
//             EXPECT_EQ(packet.vData[0], 1.0);
//             EXPECT_EQ(packet.vData[1], 1.0);
//             EXPECT_STREQ(ipAddress, constants::MODE_SIM ? constants::SIM_IP_ADDRESS.c_str() : manifest::Core::IP_ADDRESS.IP_STR.c_str());
//             EXPECT_EQ(port, constants::ROVECOMM_OUTGOING_UDP_PORT);
//         });

//     driveBoard->SendDrive(drivePowers);
// }

// /******************************************************************************
//  * @brief Test SendStop
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
// TEST_F(DriveBoardTests, SendStop) {
//     EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
//         .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
//             EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
//             EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
//             EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
//             EXPECT_EQ(packet.vData[0], 0.0);
//             EXPECT_EQ(packet.vData[1], 0.0);
//             EXPECT_STREQ(ipAddress, constants::MODE_SIM ? constants::SIM_IP_ADDRESS.c_str() : manifest::Core::IP_ADDRESS.IP_STR.c_str());
//             EXPECT_EQ(port, constants::ROVECOMM_OUTGOING_UDP_PORT);
//         });

//     driveBoard->SendStop();
// }
