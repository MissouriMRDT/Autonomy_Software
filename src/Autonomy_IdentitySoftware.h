/******************************************************************************
 * @brief Defines the Autonomy_IdentitySoftware class.
 *
 * @file Autonomy_IdentitySoftware.h
 * @author Byrdman32 (eli@byrdneststudios.com), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0620
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include <string>

#define MAJOR_VERSION 24
#define MINOR_VERSION 0
#define PATCH_VERSION 0
#define BUILD_VERSION 0

#ifndef AUTONOMY_IDENTITYSOFTWARE_H
#define AUTONOMY_IDENTITYSOFTWARE_H

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

#endif    // AUTONOMY_IDENTITYSOFTWARE_H
