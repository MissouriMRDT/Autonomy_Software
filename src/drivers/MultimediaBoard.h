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
        struct RGB;

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

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        MultimediaBoardLightingState eCurrentLightingState;
        double dCustomRed;
        double dCustomGreen;
        double dCustomBlue;
};
#endif
