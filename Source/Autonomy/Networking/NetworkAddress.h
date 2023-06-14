/*
   NetworkAddress.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/23/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:
*/

#include "../Main/MRDT_Autonomy_Globals.h"

#ifndef NETWORKADDRESS_H
#	define NETWORKADDRESS_H

enum NetworkAddressIntegers
{
	NAI_OCTET_ONE,
	NAI_OCTET_TWO,
	NAI_OCTET_THREE,
	NAI_OCTET_FOUR,
	NAI_PORT
};

class NetworkAddress
{
  private:
	int m_iOctet1;
	int m_iOctet2;
	int m_iOctet3;
	int m_iOctet4;
	int m_iPort;

  public:
	NetworkAddress();
	NetworkAddress(std::string szIPAddress, int iPort);
	NetworkAddress(int iOctet1, int iOctet2, int iOctet3, int iOctet4, int iPort);

	int GetIData(NetworkAddressIntegers eKey);
	std::string GetSZData();
};

#endif	  // NETWORKADDRESS_H
