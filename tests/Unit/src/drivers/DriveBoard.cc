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
#include "../../../../src/AutonomyConstants.h"
#include "../../../../src/AutonomyGlobals.h"
#include "../../../../src/AutonomyLogging.h"
#include "../../../../src/AutonomyNetworking.h"

/// \cond
#include "../../../../external/rovecomm/src/RoveComm/RoveComm.h"
#include "../../../../external/rovecomm/src/RoveComm/RoveCommManifest.h"
#include "../../../../external/rovecomm/src/RoveComm/RoveCommUDP.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Mock class for RoveCommUDPNode
 *
 * 
 * @date 2024-10-26
 ******************************************************************************/
// Does not work so far 
// class MockRoveCommUDP : public rovecomm::RoveCommUDP {
// public:
//     MOCK_METHOD(int, SendUDPPacketWrapper, (const rovecomm::RoveCommPacket<float>&, const char*, uint16_t));

//     template<typename T>
//     ssize_t SendUDPPacket(const rovecomm::RoveCommPacket<T>& stPacket, const char* cIPAddress, int nPort) {
//         if constexpr (std::is_same_v<T, float>) {
//             return SendUDPPacketWrapper(stPacket, cIPAddress, nPort);
//         } else {
//             // Handle other types if needed
//             return -1;
//         }
//     }
// };

/******************************************************************************
 * @brief Test fixture for DriveBoard
 *
 * 
 * @date 2024-10-26
 ******************************************************************************/
// Does not work so far 
// class DriveBoardTest : public ::testing::Test {
// protected:
//     DriveBoard* driveBoard;
//     MockRoveCommUDP* mockRoveComm;

//     void SetUp() override {
//         mockRoveComm = new MockRoveCommUDP();
//         network::g_pRoveCommUDPNode = mockRoveComm;  // Use correct global variable
//         network::g_bRoveCommUDPStatus = network::g_pRoveCommUDPNode->InitUDPSocket(manifest::General::ETHERNET_UDP_PORT);
//         driveBoard = new DriveBoard();
//     }

//     void TearDown() override {
//         delete driveBoard;
//         delete mockRoveComm;
//         network::g_pRoveCommUDPNode = nullptr;
//     }
// };

// TEST_F(DriveBoardTest, SendDriveNormalInput) {
//     diffdrive::DrivePowers drivePowers = {0.5, -0.5};

//     EXPECT_CALL(*mockRoveComm, SendUDPPacket(testing::_, testing::_, testing::_))
//         .Times(1)
//         .WillOnce(testing::Return(1));

//     driveBoard->SendDrive(drivePowers);
// }

// /******************************************************************************
//  * @brief Test for memory leaks
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(DriveBoardTest, DoesNotLeak)
// {
//     DriveBoard* testBoard = new DriveBoard();
//     ASSERT_NE(testBoard, nullptr);
//     delete testBoard;
//     testBoard = nullptr;
// }

// /******************************************************************************
//  * @brief This should fail when the --check_for_leaks command line flag is specified.
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(DriveBoardTest, Leaks)
// {
//     DriveBoard* testBoard = new DriveBoard();
//     EXPECT_NE(testBoard, nullptr);
//     // Intentionally not deleting to test leak detection
// }

// /******************************************************************************
//  * @brief Test SendDrive with normal input values
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(DriveBoardTest, SendDrive_NormalInput) {
//     diffdrive::DrivePowers drivePowers = {0.5, -0.5};

//     EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
//         .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
//             EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
//             EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
//             EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
//             EXPECT_EQ(packet.vData[0], 0.5);
//             EXPECT_EQ(packet.vData[1], -0.5);
//             EXPECT_STREQ(ipAddress, constants::MODE_SIM ? "127.0.0.1" : manifest::Core::IP_ADDRESS.IP_STR.c_str());
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
// TEST_F(DriveBoardTest, SendDrive_OutOfRangeInput) {
//     diffdrive::DrivePowers drivePowers = {2.0, -2.0};

//     EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
//         .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
//             EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
//             EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
//             EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
//             EXPECT_EQ(packet.vData[0], 1.0);
//             EXPECT_EQ(packet.vData[1], -1.0);
//             EXPECT_STREQ(ipAddress, constants::MODE_SIM ? "127.0.0.1" : manifest::Core::IP_ADDRESS.IP_STR.c_str());
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
// TEST_F(DriveBoardTest, SendDrive_MinInput) {
//     diffdrive::DrivePowers drivePowers = {-1.0, -1.0};

//     EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
//         .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
//             EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
//             EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
//             EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
//             EXPECT_EQ(packet.vData[0], -1.0);
//             EXPECT_EQ(packet.vData[1], -1.0);
//             EXPECT_STREQ(ipAddress, constants::MODE_SIM ? "127.0.0.1" : manifest::Core::IP_ADDRESS.IP_STR.c_str());
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
// TEST_F(DriveBoardTest, SendDrive_MaxInput) {
//     diffdrive::DrivePowers drivePowers = {1.0, 1.0};

//     EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
//         .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
//             EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
//             EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
//             EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
//             EXPECT_EQ(packet.vData[0], 1.0);
//             EXPECT_EQ(packet.vData[1], 1.0);
//             EXPECT_STREQ(ipAddress, constants::MODE_SIM ? "127.0.0.1" : manifest::Core::IP_ADDRESS.IP_STR.c_str());
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
// TEST_F(DriveBoardTest, SendStop) {
//     EXPECT_CALL(*mockRoveCommUDPNode, SendUDPPacket(_, _, _))
//         .WillOnce([](const rovecomm::RoveCommPacket<float>& packet, const char* ipAddress, uint16_t port) {
//             EXPECT_EQ(packet.unDataId, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
//             EXPECT_EQ(packet.unDataCount, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT);
//             EXPECT_EQ(packet.eDataType, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE);
//             EXPECT_EQ(packet.vData[0], 0.0);
//             EXPECT_EQ(packet.vData[1], 0.0);
//             EXPECT_STREQ(ipAddress, constants::MODE_SIM ? "127.0.0.1" : manifest::Core::IP_ADDRESS.IP_STR.c_str());
//             EXPECT_EQ(port, constants::ROVECOMM_OUTGOING_UDP_PORT);
//         });

//     driveBoard->SendStop();
// }
