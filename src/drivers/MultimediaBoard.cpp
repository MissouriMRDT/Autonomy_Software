/******************************************************************************
 * @brief Implements the MultimediaBoard class.
 *
 * @file MultimediaBoard.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "MultimediaBoard.h"
#include "../AutonomyGlobals.h"
#include "../AutonomyNetworking.h"

/// \cond
#include <RoveComm/RoveCommManifest.h>

/// \endcond

/******************************************************************************
 * @brief Construct a new Multimedia Board:: Multimedia Board object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
MultimediaBoard::MultimediaBoard()
{
    // Initialize member variables.
    m_eCurrentLightingState = MultimediaBoardLightingState::eOff;
}

/******************************************************************************
 * @brief Destroy the Multimedia Board:: Multimedia Board object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
MultimediaBoard::~MultimediaBoard()
{
    // Send RGB 0, 0, 0 to multimedia board to turn LED panel off.
    this->SendLightingState(MultimediaBoardLightingState::eOff);
}

/******************************************************************************
 * @brief Sends a predetermined color pattern to board.
 *
 * @param eState - The lighting state. Enum defined in header file for
 * 					MultimediaBoard.h
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
void MultimediaBoard::SendLightingState(MultimediaBoardLightingState eState)
{
    // Update member variables.
    m_eCurrentLightingState = eState;

    // Create new RoveCommPacket. Will be constructed in enum.
    rovecomm::RoveCommPacket<uint8_t> stCorePacket;
    rovecomm::RoveCommPacket<uint8_t> stTelemPacket;

    // Decide what lighting operation to execute.
    switch (eState)
    {
        case MultimediaBoardLightingState::eOff:
        {
            // Construct a RoveComm packet with the lighting data.
            stCorePacket = rovecomm::CreatePacket<manifest::Core::Commands::LEDRGB>({0, 0, 0});
            break;
        }
        case MultimediaBoardLightingState::eCustom:
        {
            // Use RoveComm to send old custom values previously set.
            this->SendRGB(m_stCustomRGBValues);
            break;
        }
        case MultimediaBoardLightingState::eTeleOp:
        {
            // Send Teleop state over RoveComm.
            stTelemPacket = rovecomm::CreatePacket<manifest::Autonomy::Telemetry::STATEDISPLAY>(static_cast<uint8_t>(manifest::Core::DISPLAYSTATE::TELEOP));
            // Construct a RoveComm packet with the lighting data.
            stCorePacket = rovecomm::CreatePacket<manifest::Core::Commands::STATEDISPLAY>(static_cast<uint8_t>(manifest::Core::DISPLAYSTATE::TELEOP));
            break;
        }
        case MultimediaBoardLightingState::eAutonomy:
        {
            // Send Autonomy state over RoveComm.
            stTelemPacket = rovecomm::CreatePacket<manifest::Autonomy::Telemetry::STATEDISPLAY>(static_cast<uint8_t>(manifest::Core::DISPLAYSTATE::AUTONOMY));
            // Construct a RoveComm packet with the lighting data.
            stCorePacket = rovecomm::CreatePacket<manifest::Core::Commands::STATEDISPLAY>(static_cast<uint8_t>(manifest::Core::DISPLAYSTATE::AUTONOMY));
            break;
        }
        case MultimediaBoardLightingState::eReachedGoal:
        {
            // Send Reached Goal state over RoveComm.
            stTelemPacket = rovecomm::CreatePacket<manifest::Autonomy::Telemetry::STATEDISPLAY>(static_cast<uint8_t>(manifest::Core::DISPLAYSTATE::REACHED_GOAL));
            // Construct a RoveComm packet with the lighting data.
            stCorePacket = rovecomm::CreatePacket<manifest::Core::Commands::STATEDISPLAY>(static_cast<uint8_t>(manifest::Core::DISPLAYSTATE::REACHED_GOAL));
            break;
        }
        default:
        {
            throw std::invalid_argument("Unknown lighting state");
        }
    }

    // Check if we should send packets to the SIM or board.
    const manifest::AddressEntry& stIPAddress = constants::MODE_SIM ? constants::SIM_IP_ADDRESS : manifest::Core::IP_ADDRESS;
    // Send multimedia board lighting state to board over RoveComm.
    if (network::g_pRoveCommUDPNode)
    {
        network::g_pRoveCommUDPNode->Send(stCorePacket, stIPAddress, constants::ROVECOMM_OUTGOING_UDP_PORT);
        network::g_pRoveCommUDPNode->Send(stTelemPacket, stIPAddress, constants::ROVECOMM_OUTGOING_UDP_PORT);
    }
}

/******************************************************************************
 * @brief Send a custom RGB value to the board.
 *
 * @param stRGBVal - RGB struct containing color information.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
void MultimediaBoard::SendRGB(RGB stRGBVal)
{
    // Update custom RGB values.
    m_stCustomRGBValues = stRGBVal;
    // Update internal lighting state.
    m_eCurrentLightingState = MultimediaBoardLightingState::eCustom;

    // Check if we should send packets to the SIM or board.
    const manifest::AddressEntry& stIPAddress = constants::MODE_SIM ? constants::SIM_IP_ADDRESS : manifest::Core::IP_ADDRESS;
    // Send RGB values to multimedia board over RoveComm.
    if (network::g_pRoveCommUDPNode)
    {
        network::g_pRoveCommUDPNode->Send<manifest::Core::Commands::LEDRGB>(
            {static_cast<uint8_t>(stRGBVal.dRed), static_cast<uint8_t>(stRGBVal.dGreen), static_cast<uint8_t>(stRGBVal.dBlue)},
            stIPAddress,
            constants::ROVECOMM_OUTGOING_UDP_PORT);
    }
}

/******************************************************************************
 * @brief Accessor for the current lighting state of the multimedia board.
 *
 * @return MultimediaBoard::MultimediaBoardLightingState - An enumerator value representing
 *      the current lighting state of the board.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-03
 ******************************************************************************/
MultimediaBoard::MultimediaBoardLightingState MultimediaBoard::GetCurrentLightingState() const
{
    // Return the current lighting state.
    return m_eCurrentLightingState;
}

/******************************************************************************
 * @brief Accessor for the current custom lighting RGB values.
 *
 * @return MultimediaBoard::RGB - The custom lighting values stored in an RGB struct.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-03
 ******************************************************************************/
MultimediaBoard::RGB MultimediaBoard::GetCustomLightingValues() const
{
    // Return the currently stored custom lighting values.
    return m_stCustomRGBValues;
}
