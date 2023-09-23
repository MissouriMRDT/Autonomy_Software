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
#include "../AutonomyLogging.h"

/******************************************************************************
 * @brief This struct serves as a container for RGB values, and provides a
 *      few overridden constructors for converting from hex to 8-bit RGB values.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
struct MultimediaBoard::RGB
{
    public:
        // Declare public struct attributes.
        double dRed;
        double dGreen;
        double dBlue;

        /******************************************************************************
         * @brief Construct a new RGB object.
         *
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2023-06-20
         ******************************************************************************/
        RGB()
        {
            // Initialize all member variables.
            this->dRed   = 0;
            this->dGreen = 0;
            this->dBlue  = 0;
        }

        /******************************************************************************
         * @brief Construct a new RGB object.
         *
         * @param iHex - The three hexadecimal digit value containing the RGB values. 0xFFF == 255, 255, 255.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2023-06-20
         ******************************************************************************/
        RGB(int iHex)
        {
            this->dRed   = ((iHex >> 16) & 0xFF);
            this->dGreen = ((iHex >> 8) & 0xFF);
            this->dBlue  = (iHex & 0xFF);
        }

        /******************************************************************************
         * @brief Construct a new RGB object.
         *
         * @param dRed - The red value of the LED panel. (0-255)
         * @param dGreen - The green value of the LED panel. (0-255)
         * @param dBlue - The blue value of the LED panel. (0-255)
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2023-06-20
         ******************************************************************************/
        RGB(double dRed, double dGreen, double dBlue)
        {
            this->dRed   = dRed;
            this->dGreen = dGreen;
            this->dBlue  = dBlue;
        }
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    m_dCustomRed            = 0.0;
    m_dCustomGreen          = 0.0;
    m_dCustomBlue           = 0.0;
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
    this->m_eCurrentLightingState = eState;

    // Decide what lighting operation to execute.
    switch (eState)
    {
        case eOff:
            // Use RoveComm to send 0, 0, 0 RGB values.
            // TODO: Add RoveComm sendpacket.
            break;

        case eCustom:
            // Use RoveComm to send old custom values previously set.
            // TODO: Add RoveComm sendpacket.
            break;

        default:
            // Send lighting state over RoveComm.
            // TODO: Add RoveComm sendpacket.
            break;
    }
    // Send multimedia board lighting state to board over RoveComm.
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
    m_dCustomRed   = stRGBVal.dRed;
    m_dCustomGreen = stRGBVal.dGreen;
    m_dCustomBlue  = stRGBVal.dBlue;

    // Send RGB values to mutlimedia board over RoveComm.
    // TODO: Add RoveComm sendpacket.
}

MultimediaBoard::MultimediaBoardLightingState MultimediaBoard::GetCurrentLightingState() const
{
    // Return the current lighting state.
    return m_eCurrentLightingState;
}
