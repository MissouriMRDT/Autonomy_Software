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

class MultimediaBoard
{
    private:
    public:
        /////////////////////////////////////////
        // Declare public enums and structs that are specific to and used withing this class.
        /////////////////////////////////////////

        // Structs.
        struct RGB;

        // Enums
        enum MultimediaBoardLightingState
        {
            TELEOP,
            AUTONOMY,
            REACHED_MARKER
        };

        MultimediaBoard();
        ~MultimediaBoard();

        void SendLightingState(/*MultimediaBoardLightingState eState*/);
        void SendRGB(/*RGB rgbVal*/);
};

#endif    // MULTIMEDIABOARD_H
