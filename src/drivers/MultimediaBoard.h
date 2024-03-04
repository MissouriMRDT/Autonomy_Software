/******************************************************************************
 * @brief Defines the MultimediaBoard class.
 *
 * @file MultimediaBoard.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef MULTIMEDIABOARD_H
#define MULTIMEDIABOARD_H

/******************************************************************************
 * @brief This class handles communication with the multimedia board on the rover
 *      by sending RoveComm packets over the network.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-23
 ******************************************************************************/
class MultimediaBoard
{
    public:
        /////////////////////////////////////////
        // Declare public enums and structs that are specific to and used withing this class.
        /////////////////////////////////////////

        // Structs.
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

        // Enums
        enum MultimediaBoardLightingState
        {
            eOff = -2,      // LED panel off.
            eCustom,        // A custom value has been set or board should go back to a previously custom set value.
            eTeleOp,        // TeleOp color = BLUE
            eAutonomy,      // Autonomy color = RED
            eReachedGoal    // Goal reached = FLASH GREEN
        };

        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////

        MultimediaBoard();
        ~MultimediaBoard();
        void SendLightingState(MultimediaBoardLightingState eState);
        void SendRGB(RGB stRGBVal);

        /////////////////////////////////////////
        // Setters
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Getters
        /////////////////////////////////////////

        MultimediaBoardLightingState GetCurrentLightingState() const;
        RGB GetCustomLightingValues() const;

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        MultimediaBoardLightingState m_eCurrentLightingState;
        RGB m_stCustomRGBValues;
};
#endif
