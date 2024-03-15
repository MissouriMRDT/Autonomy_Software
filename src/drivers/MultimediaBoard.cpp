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
    m_eCurrentLightingState = eOff;
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
    this->SendLightingState(eOff);
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
    rovecomm::RoveCommPacket<uint8_t> stPacket;

    // Decide what lighting operation to execute.
    switch (eState)
    {
        case eOff:
            // Construct a RoveComm packet with the lighting data.
            stPacket.unDataId    = manifest::Core::COMMANDS.find("LEDRGB")->second.DATA_ID;
            stPacket.unDataCount = manifest::Core::COMMANDS.find("LEDRGB")->second.DATA_COUNT;
            stPacket.eDataType   = manifest::Core::COMMANDS.find("LEDRGB")->second.DATA_TYPE;
            // Use RoveComm to send 0, 0, 0 RGB values.
            stPacket.vData.emplace_back(0);
            stPacket.vData.emplace_back(0);
            stPacket.vData.emplace_back(0);
            break;

        case eCustom:
            // Use RoveComm to send old custom values previously set.
            this->SendRGB(m_stCustomRGBValues);
            break;

        case eTeleOp:
            // Construct a RoveComm packet with the lighting data.
            stPacket.unDataId    = manifest::Core::COMMANDS.find("STATEDISPLAY")->second.DATA_ID;
            stPacket.unDataCount = manifest::Core::COMMANDS.find("STATEDISPLAY")->second.DATA_COUNT;
            stPacket.eDataType   = manifest::Core::COMMANDS.find("STATEDISPLAY")->second.DATA_TYPE;
            // Use RoveComm to send BLUE color state value.
            stPacket.vData.emplace_back(manifest::Core::DISPLAYSTATE::TELEOP);
            break;

        case eAutonomy:
            // Construct a RoveComm packet with the lighting data.
            stPacket.unDataId    = manifest::Core::COMMANDS.find("STATEDISPLAY")->second.DATA_ID;
            stPacket.unDataCount = manifest::Core::COMMANDS.find("STATEDISPLAY")->second.DATA_COUNT;
            stPacket.eDataType   = manifest::Core::COMMANDS.find("STATEDISPLAY")->second.DATA_TYPE;
            // Use RoveComm to send RED color state value.
            stPacket.vData.emplace_back(manifest::Core::DISPLAYSTATE::AUTONOMY);
            break;

        case eReachedGoal:
            // Construct a RoveComm packet with the lighting data.
            stPacket.unDataId    = manifest::Core::COMMANDS.find("STATEDISPLAY")->second.DATA_ID;
            stPacket.unDataCount = manifest::Core::COMMANDS.find("STATEDISPLAY")->second.DATA_COUNT;
            stPacket.eDataType   = manifest::Core::COMMANDS.find("STATEDISPLAY")->second.DATA_TYPE;
            // Use RoveComm to send flashing GREEN color state value.
            stPacket.vData.emplace_back(manifest::Core::DISPLAYSTATE::REACHED_GOAL);
            break;

        default:
            // Construct a RoveComm packet with the lighting data.
            stPacket.unDataId    = manifest::Core::COMMANDS.find("LEDRGB")->second.DATA_ID;
            stPacket.unDataCount = manifest::Core::COMMANDS.find("LEDRGB")->second.DATA_COUNT;
            stPacket.eDataType   = manifest::Core::COMMANDS.find("LEDRGB")->second.DATA_TYPE;
            // Send lighting state over RoveComm.
            stPacket.vData.emplace_back(0);
            stPacket.vData.emplace_back(0);
            stPacket.vData.emplace_back(0);
            break;
    }

    // Send multimedia board lighting state to board over RoveComm.
    globals::g_pRoveCommUDPNode->SendUDPPacket(stPacket, manifest::Core::IP_ADDRESS.IP_STR.c_str(), constants::ROVECOMM_OUTGOING_UDP_PORT);
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
    m_eCurrentLightingState = eCustom;

    // Construct a RoveComm packet with the lighting data.
    rovecomm::RoveCommPacket<uint8_t> stPacket;
    stPacket.unDataId    = manifest::Core::COMMANDS.find("LEDRGB")->second.DATA_ID;
    stPacket.unDataCount = manifest::Core::COMMANDS.find("LEDRGB")->second.DATA_COUNT;
    stPacket.eDataType   = manifest::Core::COMMANDS.find("LEDRGB")->second.DATA_TYPE;
    stPacket.vData.emplace_back(stRGBVal.dRed);
    stPacket.vData.emplace_back(stRGBVal.dGreen);
    stPacket.vData.emplace_back(stRGBVal.dBlue);
    // Send RGB values to multimedia board over RoveComm.
    globals::g_pRoveCommUDPNode->SendUDPPacket(stPacket, manifest::Core::IP_ADDRESS.IP_STR.c_str(), constants::ROVECOMM_OUTGOING_UDP_PORT);
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
