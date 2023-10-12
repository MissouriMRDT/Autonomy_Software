/******************************************************************************
 * @brief Unit test for GeospatialOperations utility class.
 *
 * @file GeospatialOperations.cc
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include <gtest/gtest.h>

#include "../../../../src/util/GeospatialOperations.hpp"

/******************************************************************************
 * @brief Test the functionality of the ConvertGPStoUTM function. Also tests functionality
 *      of GPS and UTM Coordinate structs.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 ******************************************************************************/
TEST(GeoOpsTest, ConvertGPStoUTM)
{
    // Initialize coordinates.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);
    geoops::GPSCoordinate stGPSMDRSCoordinate(34.406267, -110.791997);
    geoops::GPSCoordinate stGPSAustraliaCoordinate(-23.249790, 141.016173);
    geoops::GPSCoordinate stGPSYucatanCoordinate(20.402472, -89.286368);

    // Convert GPS coordinates to UTM coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate     = geoops::ConvertGPSToUTM(stGPSRollaCoordinate);
    geoops::UTMCoordinate stUTMMDRSCoordinate      = geoops::ConvertGPSToUTM(stGPSMDRSCoordinate);
    geoops::UTMCoordinate stUTMAustraliaCoordinate = geoops::ConvertGPSToUTM(stGPSAustraliaCoordinate);
    geoops::UTMCoordinate stUTMYucatanCoordinate   = geoops::ConvertGPSToUTM(stGPSYucatanCoordinate);

    // Check Rolla conversion.
    EXPECT_NEAR(stUTMRollaCoordinate.dEasting, 607344.14, 0.01);
    EXPECT_NEAR(stUTMRollaCoordinate.dNorthing, 4201167.33, 0.01);
    EXPECT_EQ(stUTMRollaCoordinate.nZone, 15);
    EXPECT_NEAR(stUTMRollaCoordinate.dMeridianConvergence, 0.7, 0.1);
    EXPECT_TRUE(stUTMRollaCoordinate.bWithinNorthernHemisphere);

    // Check Mars Desert Research Station conversion.
    EXPECT_NEAR(stUTMMDRSCoordinate.dEasting, 519116.71, 0.01);
    EXPECT_NEAR(stUTMMDRSCoordinate.dNorthing, 3807223.16, 0.01);
    EXPECT_EQ(stUTMMDRSCoordinate.nZone, 12);
    EXPECT_NEAR(stUTMMDRSCoordinate.dMeridianConvergence, 0.1, 0.1);
    EXPECT_TRUE(stUTMMDRSCoordinate.bWithinNorthernHemisphere);

    // Check Australia conversion.
    EXPECT_NEAR(stUTMAustraliaCoordinate.dEasting, 501654.37, 0.01);
    EXPECT_NEAR(stUTMAustraliaCoordinate.dNorthing, 7428828.03, 0.01);
    EXPECT_EQ(stUTMAustraliaCoordinate.nZone, 54);
    EXPECT_NEAR(stUTMAustraliaCoordinate.dMeridianConvergence, 0.0, 0.1);
    EXPECT_FALSE(stUTMAustraliaCoordinate.bWithinNorthernHemisphere);

    // Check Yucatan conversion.
    EXPECT_NEAR(stUTMYucatanCoordinate.dEasting, 261399.43, 0.01);
    EXPECT_NEAR(stUTMYucatanCoordinate.dNorthing, 2257680.11, 0.01);
    EXPECT_EQ(stUTMYucatanCoordinate.nZone, 16);
    EXPECT_NEAR(stUTMYucatanCoordinate.dMeridianConvergence, -0.7, 0.1);
    EXPECT_TRUE(stUTMYucatanCoordinate.bWithinNorthernHemisphere);
}

/******************************************************************************
 * @brief Test the functionality of the ConvertUTMtoGPS function. Also tests functionality
 *      of GPS and UTM Coordinate structs.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 ******************************************************************************/
TEST(GeoOpsTest, ConvertUTMtoGPS)
{
    // Initialize coordinates.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);
    geoops::UTMCoordinate stUTMMDRSCoordinate(519116.71, 3807223.16, 12, true);
    geoops::UTMCoordinate stUTMAustraliaCoordinate(501654.37, 7428828.03, 54, false);
    geoops::UTMCoordinate stUTMYucatanCoordinate(261399.43, 2257680.11, 16, true);

    // Convert GPS coordinates to UTM coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate     = geoops::ConvertUTMToGPS(stUTMRollaCoordinate);
    geoops::GPSCoordinate stGPSMDRSCoordinate      = geoops::ConvertUTMToGPS(stUTMMDRSCoordinate);
    geoops::GPSCoordinate stGPSAustraliaCoordinate = geoops::ConvertUTMToGPS(stUTMAustraliaCoordinate);
    geoops::GPSCoordinate stGPSYucatanCoordinate   = geoops::ConvertUTMToGPS(stUTMYucatanCoordinate);

    // Check Rolla conversion.
    EXPECT_NEAR(stGPSRollaCoordinate.dLatitude, 37.951766, 0.02);
    EXPECT_NEAR(stGPSRollaCoordinate.dLongitude, -91.778187, 0.02);
    EXPECT_NEAR(stGPSRollaCoordinate.dMeridianConvergence, 0.7, 0.1);

    // Check Mars Desert Research Station conversion.
    EXPECT_NEAR(stGPSMDRSCoordinate.dLatitude, 34.406267, 0.02);
    EXPECT_NEAR(stGPSMDRSCoordinate.dLongitude, -110.791997, 0.02);
    EXPECT_NEAR(stGPSMDRSCoordinate.dMeridianConvergence, 0.1, 0.1);

    // Check Australia conversion.
    EXPECT_NEAR(stGPSAustraliaCoordinate.dLatitude, -23.249790, 0.02);
    EXPECT_NEAR(stGPSAustraliaCoordinate.dLongitude, 141.016173, 0.02);
    EXPECT_NEAR(stGPSAustraliaCoordinate.dMeridianConvergence, 0.0, 0.1);

    // Check Yucatan conversion.
    EXPECT_NEAR(stGPSYucatanCoordinate.dLatitude, 20.402472, 0.02);
    EXPECT_NEAR(stGPSYucatanCoordinate.dLongitude, -89.286368, 0.02);
    EXPECT_NEAR(stGPSYucatanCoordinate.dMeridianConvergence, -0.7, 0.1);
}
