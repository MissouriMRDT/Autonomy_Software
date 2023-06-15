/*
   NavigationBoard.cpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Interfaces with the Navigation Board over the RoveComm Protocol.
*/

#include "NavigationBoard.h"

#include "../Autonomy_Globals.h"

NavigationBoard::NavigationBoard()
{
	m_dPitch   = 0;
	m_dRoll	   = 0;
	m_dHeading = 0;

	m_sLocation = {0, 0};

	m_iDistanceToGround = 0;
	m_iLidarQuality		= 0;

	m_tLastTime = time(nullptr);
}

void NavigationBoard::ProcessIMUData(NavBoardPacket_IMU packet)
{
	m_dPitch   = packet.dPitch;
	m_dRoll	   = packet.dRoll;
	m_dHeading = packet.dHeading;

	PLOG_DEBUG_(AL_ConsoleLogger) << "Incoming IMU Data: (" << m_dPitch << ", " << m_dRoll << ", " << m_dHeading << ")";
}

void NavigationBoard::ProcessGPSData(NavBoardPacket_GPS packet)
{
	m_sLocation.dLatitude  = packet.dLatitude;
	m_sLocation.dLongitude = packet.dLongitude;

	m_tLastTime = time(nullptr);

	PLOG_DEBUG_(AL_ConsoleLogger) << "Incoming GPS Data: (" << m_sLocation.dLatitude << ", " << m_sLocation.dLongitude
								  << ")";
}

double NavigationBoard::GetDData(NavigationBoardPacketDoubleComponents eKey) const
{

	double dValue = 0.0;

	switch (eKey)
	{
		case NBPC_PITCH: dValue = m_dPitch; break;
		case NBPC_ROLL: dValue = m_dRoll; break;
		case NBPC_HEADING: dValue = m_dHeading; break;
		default: break;
	}

	return dValue;
}

NavBoardPacket_GPS NavigationBoard::GetSData(NavigationBoardPacketCoordinateComponents eKey) const
{

	NavBoardPacket_GPS sValue = NavBoardPacket_GPS();

	switch (eKey)
	{
		case NBPCC_LOCATION: sValue = m_sLocation; break;
		default: break;
	}

	return sValue;
}
