/******************************************************************************
 * @brief Defines the MultimediaBoard class and associated datatypes.
 *
 * @file MultimediaBoard.h
 * @author Byrdman32 (eli@byrdneststudios.com), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0620
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef MULTIMEDIABOARD_H
#define MULTIMEDIABOARD_H

enum MultimediaBoardLightingState
{
    TELEOP,
    AUTONOMY,
    REACHED_MARKER
};

struct RGB
{
        double dRed;
        double dGreen;
        double dBlue;

        RGB()
        {
            this->dRed   = 0;
            this->dGreen = 0;
            this->dBlue  = 0;
        }

        RGB(int iHex)
        {
            this->dRed   = ((iHex >> 16) & 0xFF);
            this->dGreen = ((iHex >> 8) & 0xFF);
            this->dBlue  = ((iHex) &0xFF);
        }

        RGB(double dRed, double dGreen, double dBlue)
        {
            this->dRed   = dRed;
            this->dGreen = dGreen;
            this->dBlue  = dBlue;
        }
};

class MultimediaBoard
{
    public:
        MultimediaBoard();
        ~MultimediaBoard();

        void SendLightingState(MultimediaBoardLightingState eState);
        void SendRGB(RGB);
};

#endif    // MULTIMEDIABOARD_H