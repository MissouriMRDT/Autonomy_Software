/******************************************************************************
 * @brief Defines and implements functions related to operations on location or orientation
 *      coordinate systems within the geoops namespace.
 *
 * @file GeospatialOperations.hpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef GEOSPATIAL_OPERATIONS_HPP
#define GEOSPATIAL_OPERATIONS_HPP

#include <GeographicLib/UTMUPS.hpp>

/******************************************************************************
 * @brief Namespace containing functions related to operations on global position number
 *      systems and other datatypes.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
namespace geoops
{
    /******************************************************************************
     * @brief This struct stores/contains information about orientation.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-23
     ******************************************************************************/
    struct IMUData
    {
        public:
            // Declare struct public member variables.
            double dPitch;
            double dRoll;
            double dHeading;

            /******************************************************************************
             * @brief Construct a new IMUData object.
             *
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            IMUData()
            {
                // Initialize member variables to default values.
                dPitch   = 0.0;
                dRoll    = 0.0;
                dHeading = 0.0;
            }

            /******************************************************************************
             * @brief Construct a new IMUData object.
             *
             * @param dPitch - The pitch of the navboard in degrees.
             * @param dRoll - The roll of the navboard in degrees.
             * @param dHeading - The heading/yaw of the navboard in degrees.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            IMUData(double dPitch, double dRoll, double dHeading)
            {
                // Initialize member variables with given values.
                this->dPitch   = dPitch;
                this->dRoll    = dRoll;
                this->dHeading = dHeading;
            }
    };

    /******************************************************************************
     * @brief This struct stores/contains information about a GPS data.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-23
     ******************************************************************************/
    struct GPSCoordinate
    {
        public:
            // Declare struct public attributes
            double dLatitude;               // The geographic latitude in degrees, typically within the range [-90, 90].
            double dLongitude;              // The geographic longitude in degrees, typically within the range [-180, 180].
            double dAltitude;               // The elevation above sea level in meters.
            double d2DAccuracy;             // The horizontal accuracy of the GPS coordinates in meters.
            double d3DAccuracy;             // The three-dimensional accuracy, including altitude, in meters.
            double dMeridianConvergence;    // The angle between true north and grid north at the given location in degrees. Positive in eastern direction.
            double dScale;                  // The scale factor applied to the UTM coordinates for map projection.

            /////////////////////////////////////////
            // Declare public methods.
            /////////////////////////////////////////
            /******************************************************************************
             * @brief Construct a new GPSCoordinate object.
             *
             * @param dLatitude - The geographic latitude in degrees, typically within the range [-90, 90].
             * @param dLongitude - The geographic longitude in degrees, typically within the range [-180, 180].
             * @param dAltitude - The elevation above sea level in meters.
             * @param d2DAccuracy - The horizontal accuracy of the GPS coordinates in meters.
             * @param d3DAccuracy - The three-dimensional accuracy, including altitude, in meters.
             * @param dMeridianConvergence - The angle between true north and grid north at the given location in degrees. Positive in eastern direction.
             * @param dScale - The scale factor applied to the UTM coordinates for map projection.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            GPSCoordinate(double dLatitude            = 0.0,
                          double dLongitude           = 0.0,
                          double dAltitude            = 0.0,
                          double d2DAccuracy          = -1.0,
                          double d3DAccuracy          = -1.0,
                          double dMeridianConvergence = -1.0,
                          double dScale               = 0.0)
            {
                // Initialize member variables with given values.
                this->dLatitude            = dLatitude;
                this->dLongitude           = dLongitude;
                this->dAltitude            = dAltitude;
                this->d2DAccuracy          = d2DAccuracy;
                this->d3DAccuracy          = d3DAccuracy;
                this->dMeridianConvergence = dMeridianConvergence;
                this->dScale               = dScale;
            }
    };

    /******************************************************************************
     * @brief This struct stores/contains information about a UTM coordinate.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-23
     ******************************************************************************/
    struct UTMCoordinate
    {
        public:
            // Declare struct public attributes.
            double dEasting;                   // The eastward displacement from the UTM zone's central meridian in meters.
            double dNorthing;                  // The northward displacement from the equator in meters.
            double dAltitude;                  // The elevation above sea level in meters.
            int nZone;                         // The UTM zone number identifying the region on the Earth's surface.
            double d2DAccuracy;                // The horizontal accuracy of the UTM coordinates in meters.
            double d3DAccuracy;                // The three-dimensional accuracy, including altitude, in meters.
            double dMeridianConvergence;       // The angle between true north and grid north at the given location in degrees. Positive in eastern direction.
            double dScale;                     // The scale factor applied to the UTM coordinates for map projection.
            bool bWithinNorthernHemisphere;    // Indicates whether the coordinate is located in the northern hemisphere.

            /////////////////////////////////////////
            // Declare public methods.
            /////////////////////////////////////////
            /******************************************************************************
             * @brief Construct a new UTMCoordinate object.
             *
             * @param dEasting - The eastward displacement from the UTM zone's central meridian in meters.
             * @param dNorthing - The northward displacement from the equator in meters.
             * @param dAltitude - The elevation above sea level in meters.
             * @param nZone - The UTM zone number identifying the region on the Earth's surface.
             * @param d2DAccuracy - The horizontal accuracy of the UTM coordinates in meters.
             * @param d3DAccuracy - The three-dimensional accuracy, including altitude, in meters.
             * @param dMeridianConvergence - The angle between true north and grid north at the given location in degrees. Positive in eastern direction.
             * @param dScale - The scale factor applied to the UTM coordinates for map projection.
             * @param bWithinNorthernHemisphere - Indicates whether the coordinate is located in the northern hemisphere.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            UTMCoordinate(double dEasting                = 0.0,
                          double dNorthing               = 0.0,
                          double dAltitude               = 0.0,
                          int nZone                      = 0,
                          double d2DAccuracy             = -1.0,
                          double d3DAccuracy             = -1.0,
                          double dMeridianConvergence    = -1.0,
                          double dScale                  = 0.0,
                          bool bWithinNorthernHemisphere = true)
            {
                // Initialize member variables with given values.
                this->dEasting                  = dEasting;
                this->dNorthing                 = dNorthing;
                this->dAltitude                 = dAltitude;
                this->nZone                     = nZone;
                this->d2DAccuracy               = d2DAccuracy;
                this->d3DAccuracy               = d3DAccuracy;
                this->dMeridianConvergence      = dMeridianConvergence;
                this->dScale                    = dScale;
                this->bWithinNorthernHemisphere = bWithinNorthernHemisphere;
            }
    };

    /******************************************************************************
     * @brief Given a GPS coordinate, convert to UTM and create a new UTMCoordinate object.
     *
     * @param stGPSCoord - The struct containing the GPS coordinate data.
     * @return UTMCoordinate - The UTM coordinate corresponding to the given GPS coordinate.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-12
     ******************************************************************************/
    inline UTMCoordinate ConvertGPSToUTM(GPSCoordinate stGPSCoord)
    {
        // Create instance variables.
        UTMCoordinate stConvertCoord;

        // Get data out of the GPS coord and repackage it into the UTM struct.
        stConvertCoord.d2DAccuracy = stGPSCoord.d2DAccuracy;
        stConvertCoord.d3DAccuracy = stGPSCoord.d3DAccuracy;
        stConvertCoord.dAltitude   = stGPSCoord.dAltitude;
        GeographicLib::UTMUPS::Forward(stGPSCoord.dLatitude,
                                       stGPSCoord.dLongitude,
                                       stConvertCoord.nZone,
                                       stConvertCoord.bWithinNorthernHemisphere,
                                       stConvertCoord.dEasting,
                                       stConvertCoord.dNorthing,
                                       stConvertCoord.dMeridianConvergence,
                                       stConvertCoord.dScale);

        // Return the converted UTM coordinate.
        return stConvertCoord;
    }

    /******************************************************************************
     * @brief Given a UTM coordinate, convert to GPS and create a new GPSCoordinate object.
     *
     * @param stUTMCoord - The struct containing the UTM coordinate data.
     * @return GPSCoordinate - The GPS coordinate corresponding to the given UTM coordinate.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-12
     ******************************************************************************/
    inline GPSCoordinate ConvertUTMToGPS(UTMCoordinate stUTMCoord)
    {
        // Create instance variables.
        GPSCoordinate stConvertCoord;

        // Get data out of the UTM coord and repackage it into the GPS struct.
        stConvertCoord.d2DAccuracy = stUTMCoord.d2DAccuracy;
        stConvertCoord.d3DAccuracy = stUTMCoord.d3DAccuracy;
        stConvertCoord.dAltitude   = stUTMCoord.dAltitude;
        GeographicLib::UTMUPS::Reverse(stUTMCoord.nZone,
                                       stUTMCoord.bWithinNorthernHemisphere,
                                       stUTMCoord.dEasting,
                                       stUTMCoord.dNorthing,
                                       stConvertCoord.dLatitude,
                                       stConvertCoord.dLongitude,
                                       stConvertCoord.dMeridianConvergence,
                                       stConvertCoord.dScale);

        // Return the converted UTM coordinate.
        return stConvertCoord;
    }
}    // namespace geoops
#endif
