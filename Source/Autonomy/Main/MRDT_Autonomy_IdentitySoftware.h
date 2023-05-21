/*
   MRDT_Autonomy_IdentitySoftware.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Defines global defines, variables, and functions for MRDT Software.
*/

#include <string>

#define MAJOR_VERSION   23
#define MINOR_VERSION   0
#define PATCH_VERSION   0
#define BUILD_VERSION   0

#ifndef MRDTAUTONOMYIDENTITYSOFTWARE_H
#define MRDTAUTONOMYIDENTITYSOFTWARE_H

class MRDT_Autonomy_IdentitySoftware {
private:
    std::string szMajorVersion;
    std::string szMinorVersion;
    std::string szPatchVersion;
    std::string szBuildVersion;
public:
    MRDT_Autonomy_IdentitySoftware();

    std::string GetVersionNumber();
    std::string GetBuildNumber();
    std::string GetVersionBuildComboNumber();
};

#endif // MRDTAUTONOMYIDENTITYSOFTWARE_H
