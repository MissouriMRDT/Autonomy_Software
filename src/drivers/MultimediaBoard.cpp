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
struct RGB
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
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
MultimediaBoard::MultimediaBoard() {}

/******************************************************************************
 * @brief Destroy the Multimedia Board:: Multimedia Board object.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
MultimediaBoard::~MultimediaBoard() {}

/******************************************************************************
 * @brief Sends a predetermined color pattern to board.
 *
 * @param eState - The lighting state. Enum defined in header file for
 * 					MultimediaBoard.h
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
void MultimediaBoard::SendLightingState(/*MultimediaBoardLightingState eState*/) {}

/******************************************************************************
 * @brief Send a custom RGB value to the board.
 *
 * @param rgbVal - RGB struct containing color information. Struct defined in
 * 					MultimediaBoard.h
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
void MultimediaBoard::SendRGB(/*RGB rgbVal*/) {}
