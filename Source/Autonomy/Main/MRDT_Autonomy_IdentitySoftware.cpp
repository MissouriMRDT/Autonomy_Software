/*
   MRDT_Autonomy_IdentitySoftware.cpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Defines global defines, variables, and functions for MRDT Software.
*/

#include "MRDT_Autonomy_IdentitySoftware.h"

MRDT_Autonomy_IdentitySoftware::MRDT_Autonomy_IdentitySoftware() {

    // Force extra characters to the version numbers for '0' padding
    szMajorVersion = "000";
    szMinorVersion = "000";
    szPatchVersion = "000";
    szBuildVersion = "000";

    // Append the versions after the forced extra characters
    szMajorVersion += std::to_string(MAJOR_VERSION);
    szMinorVersion += std::to_string(MINOR_VERSION);
    szPatchVersion += std::to_string(PATCH_VERSION);
    szBuildVersion += std::to_string(BUILD_VERSION);

    // Shorten the strings down to the appropriate lengths
    if (szMajorVersion.length() > 2) {
        szMajorVersion.erase(0, szMajorVersion.length() - 2);
    }

    if (szMinorVersion.length() > 2) {
        szMinorVersion.erase(0, szMinorVersion.length() - 2);
    }

    if (szPatchVersion.length() > 2) {
        szPatchVersion.erase(0, szPatchVersion.length() - 2);
    }

    if (szBuildVersion.length() > 3) {
        szBuildVersion.erase(0, szBuildVersion.length() - 3);
    }
}

std::string MRDT_Autonomy_IdentitySoftware::GetVersionNumber() {
    return "v" + szMajorVersion + "." + szMinorVersion + "." + szPatchVersion;
}

std::string MRDT_Autonomy_IdentitySoftware::GetBuildNumber() {
    return  "Build " + szBuildVersion;
}

std::string MRDT_Autonomy_IdentitySoftware::GetVersionBuildComboNumber() {
    return  "v" + szMajorVersion + "." + szMinorVersion + "." + szPatchVersion + " Build " + szBuildVersion;
}