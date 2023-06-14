/*
   MRDT_MultimediaBoard.cpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Interfaces with the Multimedia Board over the RoveComm
   Protocol.
*/

#include "MRDT_MultimediaBoard.h"

#include "../Main/MRDT_Autonomy_Globals.h"

MRDT_MultimediaBoard::MRDT_MultimediaBoard() = default;

void MRDT_MultimediaBoard::SendLightingState(MultimediaBoardLightingState eState) {}

void MRDT_MultimediaBoard::SendRGB(RGB) {}
