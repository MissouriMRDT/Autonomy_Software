/*
   Autonomy_IdentitySoftware.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      The Autonomy Software Version Handler.
*/

#include <string>

#define MAJOR_VERSION 23
#define MINOR_VERSION 0
#define PATCH_VERSION 0
#define BUILD_VERSION 0

#ifndef AUTONOMY_IDENTITYSOFTWARE_H
#	define AUTONOMY_IDENTITYSOFTWARE_H

class Autonomy_IdentitySoftware
{
  private:
	std::string m_szMajorVersion;
	std::string m_szMinorVersion;
	std::string m_szPatchVersion;
	std::string m_szBuildVersion;

  public:
	Autonomy_IdentitySoftware();

	std::string GetVersionNumber();
	std::string GetBuildNumber();
	std::string GetVersionBuildComboNumber();
};

#endif	  // AUTONOMY_IDENTITYSOFTWARE_H