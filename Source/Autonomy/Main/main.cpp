/*
   main.cpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Defines the entry point for the application.
*/

#include "../Networking/RoveComm.h"
#include "MRDT_Autonomy_Globals.h"

int main()
{

	InitializeAutonomyLoggers();	// Initialize loggers for Autonomy Software

	RoveCommPacket<double_t> test;

	RoveCommEthernetUDP<int8_t> test2(3543);
	test2.Write(test);

	return 0;
}