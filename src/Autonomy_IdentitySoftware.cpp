/*
   Autonomy_IdentitySoftware.cpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:			 5/20/2023
   Author:		   Eli Byrd and Clayton Cowen
   Description:	  The Autonomy Software Version Handler.
*/

#include "Autonomy_IdentitySoftware.h"

Autonomy_IdentitySoftware::Autonomy_IdentitySoftware()
{

	// Force extra characters to the version numbers for '0' padding
	m_szMajorVersion = "000";
	m_szMinorVersion = "000";
	m_szPatchVersion = "000";
	m_szBuildVersion = "000";

	// Append the versions after the forced extra characters
	m_szMajorVersion += std::to_string(MAJOR_VERSION);
	m_szMinorVersion += std::to_string(MINOR_VERSION);
	m_szPatchVersion += std::to_string(PATCH_VERSION);
	m_szBuildVersion += std::to_string(BUILD_VERSION);

	// Shorten the strings down to the appropriate lengths
	if (m_szMajorVersion.length() > 2) { m_szMajorVersion.erase(0, m_szMajorVersion.length() - 2); }

	if (m_szMinorVersion.length() > 2) { m_szMinorVersion.erase(0, m_szMinorVersion.length() - 2); }

	if (m_szPatchVersion.length() > 2) { m_szPatchVersion.erase(0, m_szPatchVersion.length() - 2); }

	if (m_szBuildVersion.length() > 3) { m_szBuildVersion.erase(0, m_szBuildVersion.length() - 3); }
}

std::string Autonomy_IdentitySoftware::GetVersionNumber() { return "v" + m_szMajorVersion + "." + m_szMinorVersion + "." + m_szPatchVersion; }

std::string Autonomy_IdentitySoftware::GetBuildNumber() { return "Build " + m_szBuildVersion; }

std::string Autonomy_IdentitySoftware::GetVersionBuildComboNumber()
{
	return "v" + m_szMajorVersion + "." + m_szMinorVersion + "." + m_szPatchVersion + " Build " + m_szBuildVersion;
}