/******************************************************************************
 * @brief Implements the IdentitySoftware class.
 * 		Handler for incrementing and tracking software version/build numbers.
 *
 * @file IdentitySoftware.cpp
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "IdentitySoftware.h"

/******************************************************************************
 * @brief Construct a new IdentitySoftware::IdentitySoftware object.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 ******************************************************************************/
IdentitySoftware::IdentitySoftware()
{
    // Force extra characters to the version numbers for '0' padding
    m_szMajorVersion = "000";
    m_szMinorVersion = "000";
    m_szPatchVersion = "000";
    m_szBuildVersion = "000";

    // Append the versions after the forced extra characters
    m_szMajorVersion += std::to_string(AUTONOMY_MAJOR_VERSION);
    m_szMinorVersion += std::to_string(AUTONOMY_MINOR_VERSION);
    m_szPatchVersion += std::to_string(AUTONOMY_PATCH_VERSION);
    m_szBuildVersion += std::to_string(AUTONOMY_BUILD_VERSION);

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
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 ******************************************************************************/
std::string IdentitySoftware::GetVersionNumber()
{
    return "v" + m_szMajorVersion + "." + m_szMinorVersion + "." + m_szPatchVersion;
}

/******************************************************************************
 * @brief Gets the build number of project.
 *
 * @return std::string - The build number.
 *
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 ******************************************************************************/
std::string IdentitySoftware::GetBuildNumber()
{
    return "Build " + m_szBuildVersion;
}

/******************************************************************************
 * @brief Gets the combo number container version and build info.
 *
 * @return std::string - The combo number.
 *
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 ******************************************************************************/
std::string IdentitySoftware::GetVersionBuildComboNumber()
{
    return "v" + m_szMajorVersion + "." + m_szMinorVersion + "." + m_szPatchVersion + " Build " + m_szBuildVersion;
}
