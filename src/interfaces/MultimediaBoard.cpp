/*
   MultimediaBoard.cpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:			 5/20/2023
   Author:		   Eli Byrd and Clayton Cowen
   Description:	  Interfaces with the Multimedia Board over the RoveComm
   Protocol.
*/

#include "MultimediaBoard.h"

#include "../Autonomy_Globals.h"

MultimediaBoard::MultimediaBoard() = default;

void MultimediaBoard::SendLightingState(MultimediaBoardLightingState eState) {}

void MultimediaBoard::SendRGB(RGB) {}
