/*
   NetworkAddress.cpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/23/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:
*/

#include "NetworkAddress.h"

NetworkAddress::NetworkAddress()
{
	// IP Address
	this->m_iOctet1 = 0;
	this->m_iOctet2 = 0;
	this->m_iOctet3 = 0;
	this->m_iOctet4 = 0;

	// Port
	this->m_iPort = 0;
}

NetworkAddress::NetworkAddress(std::string szIPAddress, int iPort)
{

	// Set the length of the IP Address String
	const unsigned long lIPAddressLength = szIPAddress.length();

	// Create a counter object for setting integer based IP Octets
	unsigned int iOctetCounter = 0;

	// Create Character Pointers for the substring process
	char* pIPAddress;
	char* pOctetOfIPAddress;

	// Create a new character array
	pIPAddress = new char[lIPAddressLength + 1];

	// Copy the passed in IP Address that was passed
	// into the function into the character array.
	strcpy(pIPAddress, szIPAddress.c_str());

	// Retrieve the first octet of the IP Address
	pOctetOfIPAddress = strtok(pIPAddress, ".");

	while (pOctetOfIPAddress != nullptr)
	{

		// Convert the octet character pointer into a string
		std::string szOctet = pOctetOfIPAddress;

		// Convert the string to an integer and assign it to the appropriate octet
		switch (iOctetCounter)
		{
			case 0: this->m_iOctet1 = std::stoi(szOctet); break;
			case 1: this->m_iOctet2 = std::stoi(szOctet); break;
			case 2: this->m_iOctet3 = std::stoi(szOctet); break;
			case 3: this->m_iOctet4 = std::stoi(szOctet); break;
			default: PLOG_FATAL_(AL_ConsoleLogger) << "Reached Out of Bounds Index"; break;
		}

		// Retrieve the next octet of the IP Address
		pOctetOfIPAddress = strtok(nullptr, "-");

		// Increment the Octet Counter
		iOctetCounter++;
	}

	// Set Port
	this->m_iPort = iPort;

	// Deinitialize Character Pointers
	delete pIPAddress;
	delete pOctetOfIPAddress;
}

NetworkAddress::NetworkAddress(int iOctet1, int iOctet2, int iOctet3, int iOctet4, int iPort)
{
	// IP Address
	this->m_iOctet1 = iOctet1;
	this->m_iOctet2 = iOctet2;
	this->m_iOctet3 = iOctet3;
	this->m_iOctet4 = iOctet4;

	// Port
	this->m_iPort = iPort;
}

int NetworkAddress::GetIData(NetworkAddressIntegers eKey)
{
	int iValue = 0;

	switch (eKey)
	{
		case NAI_OCTET_ONE: iValue = m_iOctet1; break;
		case NAI_OCTET_TWO: iValue = m_iOctet2; break;
		case NAI_OCTET_THREE: iValue = m_iOctet3; break;
		case NAI_OCTET_FOUR: iValue = m_iOctet4; break;
		case NAI_PORT: iValue = m_iPort; break;
		default: PLOG_FATAL_(AL_ConsoleLogger) << "Reached invalid Index!"; break;
	}

	return iValue;
}

std::string NetworkAddress::GetSZData()
{
	return std::to_string(m_iOctet1) + "." + std::to_string(m_iOctet2) + "." + std::to_string(m_iOctet3) + "."
		   + std::to_string(m_iOctet4);
}
