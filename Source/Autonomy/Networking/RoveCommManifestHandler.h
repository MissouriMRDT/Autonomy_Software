/*
   RoveCommManifestHandler.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/29/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:
*/

#include "../Main/MRDT_Autonomy_Globals.h"

#include <array>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <string>
#include <vector>

#define ROVECOMM_MANIFEST_FILENAME "../Source/Autonomy/Networking/Manifest.json"

#ifndef ROVECOMMMANIFESTHANDLER_H
#	define ROVECOMMMANIFESTHANDLER_H

enum RoveCommManifestIdentifiers
{
	RCMI_CORE,
	RCMI_NAV
};

enum RoveCommManifestIntegers
{
	RCMI_DATA_ID,
	RCMI_DATA_COUNT
};

enum RoveCommManifestStrings
{
	RCMSZ_NAME,
	RCMSZ_DATA_TYPE
};

struct RoveCommBoardCommand
{
	std::string szName;
	int iDataId;
	std::string szDataType;
	int iDataCount;
};

struct RoveCommBoard
{
	std::string szIPAddress;
	std::vector<RoveCommBoardCommand> vCommands;
	std::vector<RoveCommBoardCommand> vTelemetry;
};

class RoveCommManifestHandler
{
  private:
	RoveCommBoard pCoreBoard;
	RoveCommBoard pNavBoard;

  public:
	void SetupBoard(RoveCommManifestIdentifiers eValue);

	std::string GetIPAddress(RoveCommManifestIdentifiers eIdentifier) const;
};

#endif	  // ROVECOMMMANIFESTHANDLER_H
