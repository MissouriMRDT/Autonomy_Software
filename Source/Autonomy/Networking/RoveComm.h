/*
   RoveComm.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/21/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      
*/

#include <string>

#include "../Main/MRDT_Autonomy_Globals.h"
#include "RoveCommPacket.h"
#include "RoveCommEthernetUDP.h"
#include "RoveCommEthernetTCP.h"
#include "RoveCommManifestHandler.h"

#ifndef ROVECOMM_H
#define ROVECOMM_H

class RoveComm {
private:
    RoveCommManifestHandler pManifestHandler;
public:
    RoveComm();
};

#endif // ROVECOMM_H
