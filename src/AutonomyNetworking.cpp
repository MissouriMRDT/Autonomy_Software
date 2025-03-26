/******************************************************************************
 * @brief Defines functions and objects used for Autonomy Networking
 *
 * @file AutonomyNetworking.cpp
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2024-03-17
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "AutonomyNetworking.h"

/******************************************************************************
 * @brief Defines functions and objects used for Autonomy Networking
 *
 * @file AutonomyNetworking.cpp
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2024-03-17
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "AutonomyNetworking.h"

/******************************************************************************
 * @brief Namespace containing all networking types/structs that will be used
 *        project wide.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-17
 ******************************************************************************/
namespace network
{
    // RoveComm Instances:
    std::shared_ptr<rovecomm::RoveCommUDP> g_pRoveCommUDPNode;
    std::shared_ptr<rovecomm::RoveCommTCP> g_pRoveCommTCPNode;

    // RoveComm Status:
    std::atomic_bool g_bRoveCommUDPStatus = false;
    std::atomic_bool g_bRoveCommTCPStatus = false;
}    // namespace network
