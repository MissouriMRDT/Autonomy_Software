/******************************************************************************
 * @brief Implements the Autonomy_IdentitySoftware class.
 * 		Handler for incrementing and tracking software version/build numbers.
 *
 * @file Autonomy_IdentitySoftware.cpp
 * @author Byrdman32 (eli@byrdneststudios.com)
 * @date 2023-0620
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "Autonomy_IdentitySoftware.h"

/******************************************************************************
 * @brief Construct a new Autonomy_IdentitySoftware::Autonomy_IdentitySoftware object.
 *
 *
 * @author Byrdman32 (eli@byrdneststudios.com)
 * @date 2023-0620
 ******************************************************************************/
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
    if (m_szMajorVersion.length() > 2)
    {
        m_szMajorVersion.erase(0, m_szMajorVersion.length() - 2);
    }

    if (m_szMinorVersion.length() > 2)
    {
        m_szMinorVersion.erase(0, m_szMinorVersion.length() - 2);
    }

    if (m_szPatchVersion.length() > 2)
    {
        m_szPatchVersion.erase(0, m_szPatchVersion.length() - 2);
    }

    if (m_szBuildVersion.length() > 3)
    {
        m_szBuildVersion.erase(0, m_szBuildVersion.length() - 3);
    }
}

/******************************************************************************
 * @brief Gets the version number of project.
 *
 * @return std::string - The version number.
 *
 * @author Byrdman32 (eli@byrdneststudios.com)
 * @date 2023-0620
 ******************************************************************************/
std::string Autonomy_IdentitySoftware::GetVersionNumber()
{
    return "v" + m_szMajorVersion + "." + m_szMinorVersion + "." + m_szPatchVersion;
}

/******************************************************************************
 * @brief Gets the build number of project.
 *
 * @return std::string - The build number.
 *
 * @author Byrdman32 (eli@byrdneststudios.com)
 * @date 2023-0620
 ******************************************************************************/
std::string Autonomy_IdentitySoftware::GetBuildNumber()
{
    return "Build " + m_szBuildVersion;
}

/******************************************************************************
 * @brief Gets the combo number container version and build info.
 *
 * @return std::string - The combo number.
 *
 * @author Byrdman32 (eli@byrdneststudios.com)
 * @date 2023-0620
 ******************************************************************************/
std::string Autonomy_IdentitySoftware::GetVersionBuildComboNumber()
{
    return "v" + m_szMajorVersion + "." + m_szMinorVersion + "." + m_szPatchVersion + " Build " + m_szBuildVersion;
}
