/******************************************************************************
 * @brief Defines the IdentitySoftware class.
 *
 * @file IdentitySoftware.h
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef IDENTITY_SOFTWARE_H
#define IDENTITY_SOFTWARE_H

#include <string>

#define MAJOR_VERSION 24
#define MINOR_VERSION 0
#define PATCH_VERSION 0
#define BUILD_VERSION 0

class IdentitySoftware
{
    private:
        std::string m_szMajorVersion;
        std::string m_szMinorVersion;
        std::string m_szPatchVersion;
        std::string m_szBuildVersion;

    public:
        IdentitySoftware();

        std::string GetVersionNumber();
        std::string GetBuildNumber();
        std::string GetVersionBuildComboNumber();
};

#endif    // IDENTITY_SOFTWARE_H
