/*
   MRDT_MultimediaBoard.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Interfaces with the Multimedia Board over the RoveComm Protocol.
*/

#ifndef MRDT_MULTIMEDIABOARD_H
#define MRDT_MULTIMEDIABOARD_H

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
		this->dRed	 = 0;
		this->dGreen = 0;
		this->dBlue	 = 0;
	}

	RGB(int iHex)
	{
		this->dRed	 = ((iHex >> 16) & 0xFF);
		this->dGreen = ((iHex >> 8) & 0xFF);
		this->dBlue	 = ((iHex) &0xFF);
	}

	RGB(double dRed, double dGreen, double dBlue)
	{
		this->dRed	 = dRed;
		this->dGreen = dGreen;
		this->dBlue	 = dBlue;
	}
};

class MRDT_MultimediaBoard
{
  public:
	MRDT_MultimediaBoard();

	void SendLightingState(MultimediaBoardLightingState eState);
	void SendRGB(RGB);
};

#endif	  // MRDT_MULTIMEDIABOARD_H
