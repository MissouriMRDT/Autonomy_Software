/*
   MRDT_DriveBoard.cpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Interfaces with the Drive Board over the RoveComm Protocol.
*/

#include "MRDT_DriveBoard.h"

#include "../Main/MRDT_Autonomy_Globals.h"

MRDT_DriveBoard::MRDT_DriveBoard()
{
	m_iTargetSpeedLeft	= 0;
	m_iTargetSpeedRight = 0;
}

std::vector<int> MRDT_DriveBoard::CalculateMove(float fSpeed, float fAngle)
{
	double dSpeedLeft  = fSpeed;
	double dSpeedRight = fSpeed;

	if (fAngle > 0) { dSpeedRight = dSpeedRight * (1 - (fAngle / 180.0)); }
	else if (fAngle < 0) { dSpeedLeft = dSpeedLeft * (1 + (fAngle / 180.0)); }

	m_iTargetSpeedLeft	= int(Clamp<double>(dSpeedLeft, MIN_DRIVE_POWER, MAX_DRIVE_POWER));
	m_iTargetSpeedRight = int(Clamp<double>(dSpeedRight, MIN_DRIVE_POWER, MAX_DRIVE_POWER));

	PLOG_DEBUG_(AL_ConsoleLogger) << "Driving at (" << m_iTargetSpeedLeft << ", " << m_iTargetSpeedRight << ")";

	return {m_iTargetSpeedLeft, m_iTargetSpeedRight};
}

void MRDT_DriveBoard::SendDrive(int iLeftTarget, int iRightTarget) {}

void MRDT_DriveBoard::SendStop() {}
