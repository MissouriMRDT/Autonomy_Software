/*
   RoveComm.cpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/21/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:
*/

#include "RoveComm.h"

RoveComm::RoveComm()
{
	pManifestHandler = RoveCommManifestHandler();
	pManifestHandler.SetupBoard(RoveCommManifestIdentifiers::RCMI_CORE);
	pManifestHandler.SetupBoard(RoveCommManifestIdentifiers::RCMI_NAV);

	std::cout << pManifestHandler.GetIPAddress(RoveCommManifestIdentifiers::RCMI_CORE) << "\n"
			  << pManifestHandler.GetIPAddress(RoveCommManifestIdentifiers::RCMI_NAV) << std::endl;
}
