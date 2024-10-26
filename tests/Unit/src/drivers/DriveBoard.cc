/******************************************************************************
 * @brief Unit test for DriveBoard driver class.
 *
 * @file DriveBoard.cc
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 *
 * @copyright Copyright MRDT 2024 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/drivers/DriveBoard.h"

/// \cond
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "../../../../../src/AutonomyConstants.h"
#include "../../../../../src/AutonomyGlobals.h"
#include "../../../../../src/AutonomyLogging.h"
#include "../../../../../src/AutonomyNetworking.h"
#include "../../../../../external/rovecomm/src/RoveComm/RoveComm.h
#include "../../../../../external/rovecomm/src/RoveComm/RoveCommManifest.h

/// \endcond

/******************************************************************************
 * @brief Test for memory leaks
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
TEST(DriveBoardTest, DoesNotLeak) {
    DriveBoard* driveBoard = new DriveBoard();
    delete driveBoard;
}

/******************************************************************************
 * @brief Mock class for RoveCommUDPNode
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
class MockRoveCommUDPNode : public network::RoveCommUDPNode {
public:
    MOCK_METHOD(void, SendUDPPacket, (const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port), (override));
};

/******************************************************************************
 * @brief Test fixture for DriveBoard
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
class DriveBoardTest : public ::testing::Test {
protected:
    // Create DriveBoard and MockRoveCommUDPNode objects.
    DriveBoard* driveBoard;
    MockRoveCommUDPNode* mockRoveCommUDPNode;

    // Set up the test fixture.
    void SetUp() override {
        // Create objects.
        mockRoveCommUDPNode = new MockRoveCommUDPNode();
        network::g_pRoveCommUDPNode = mockRoveCommUDPNode;
        driveBoard = new DriveBoard();
    }

    // Tear down the test fixture.
    void TearDown() override {
        delete driveBoard;
        delete mockRoveCommUDPNode;
    }
};

/******************************************************************************
 * @brief Test SendDrive with normal input values
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(DriveBoardTest, SendDrive_NormalInput) {
    diffdrive::DrivePowers drivePowers = {0.5, -0.5};

    EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
        .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
            EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
            EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
            EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
            EXPECT_EQ(packet.vData[0], 0.5);
            EXPECT_EQ(packet.vData[1], -0.5);
            EXPECT_STREQ(ipAddress, constants::MODE_SIM ? "127.0.0.1" : manifest::Core::IP_ADDRESS.IP_STR.c_str());
            EXPECT_EQ(port, constants::ROVECOMM_OUTGOING_UDP_PORT);
        });

    driveBoard->SendDrive(drivePowers);
}

/******************************************************************************
 * @brief Test SendDrive with input values out of range
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(DriveBoardTest, SendDrive_OutOfRangeInput) {
    diffdrive::DrivePowers drivePowers = {2.0, -2.0};

    EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
        .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
            EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
            EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
            EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
            EXPECT_EQ(packet.vData[0], 1.0);
            EXPECT_EQ(packet.vData[1], -1.0);
            EXPECT_STREQ(ipAddress, constants::MODE_SIM ? "127.0.0.1" : manifest::Core::IP_ADDRESS.IP_STR.c_str());
            EXPECT_EQ(port, constants::ROVECOMM_OUTGOING_UDP_PORT);
        });

    driveBoard->SendDrive(drivePowers);
}

/******************************************************************************
 * @brief Test SendDrive with minimum input values
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(DriveBoardTest, SendDrive_MinInput) {
    diffdrive::DrivePowers drivePowers = {-1.0, -1.0};

    EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
        .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
            EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
            EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
            EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
            EXPECT_EQ(packet.vData[0], -1.0);
            EXPECT_EQ(packet.vData[1], -1.0);
            EXPECT_STREQ(ipAddress, constants::MODE_SIM ? "127.0.0.1" : manifest::Core::IP_ADDRESS.IP_STR.c_str());
            EXPECT_EQ(port, constants::ROVECOMM_OUTGOING_UDP_PORT);
        });

    driveBoard->SendDrive(drivePowers);
}

/******************************************************************************
 * @brief Test SendDrive with maximum input values
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 ******************************************************************************/
TEST_F(DriveBoardTest, SendDrive_MaxInput) {
    diffdrive::DrivePowers drivePowers = {1.0, 1.0};

    EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
        .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
            EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
            EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
            EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
            EXPECT_EQ(packet.vData[0], 1.0);
            EXPECT_EQ(packet.vData[1], 1.0);
            EXPECT_STREQ(ipAddress, constants::MODE_SIM ? "127.0.0.1" : manifest::Core::IP_ADDRESS.IP_STR.c_str());
            EXPECT_EQ(port, constants::ROVECOMM_OUTGOING_UDP_PORT);
        });

    driveBoard->SendDrive(drivePowers);
}

/******************************************************************************
 * @brief Test SendStop
 *
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
******************************************************************************/
TEST_F(DriveBoardTest, SendStop) {
    EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
        .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
            EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
            EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
            EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
            EXPECT_EQ(packet.vData[0], 0.0);
            EXPECT_EQ(packet.vData[1], 0.0);
            EXPECT_STREQ(ipAddress, constants::MODE_SIM ? "127.0.0.1" : manifest::Core::IP_ADDRESS.IP_STR.c_str());
            EXPECT_EQ(port, constants::ROVECOMM_OUTGOING_UDP_PORT);
        });

    driveBoard->SendStop();
}



