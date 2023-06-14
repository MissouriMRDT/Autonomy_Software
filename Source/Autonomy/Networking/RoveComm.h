/*
   RoveComm.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/21/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:
*/

#include "../Main/MRDT_Autonomy_Globals.h"
#include "RoveCommEthernetTCP.h"
#include "RoveCommEthernetUDP.h"
#include "RoveCommManifestHandler.h"
#include "RoveCommPacket.h"

#include <string>

#ifndef ROVECOMM_H
#	define ROVECOMM_H

class RoveComm
{
  private:
	RoveCommManifestHandler pManifestHandler;

  public:
	RoveComm();
};

#endif	  // ROVECOMM_H
