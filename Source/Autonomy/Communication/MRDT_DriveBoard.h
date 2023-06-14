/*
   MRDT_DriveBoard.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Interfaces with the Drive Board over the RoveComm Protocol.
*/

#include "../DataAndComputation/CClamp.h"

#ifndef MRDT_DRIVEBOARD_H
#	define MRDT_DRIVEBOARD_H

class MRDT_DriveBoard
{
  private:
	int m_iTargetSpeedLeft;
	int m_iTargetSpeedRight;

  public:
	MRDT_DriveBoard();

	std::vector<int> CalculateMove(float fSpeed, float fAngle);
	void SendDrive(int iLeftTarget, int iRightTarget);
	void SendStop();
};

#endif	  // MRDT_DRIVEBOARD_H
