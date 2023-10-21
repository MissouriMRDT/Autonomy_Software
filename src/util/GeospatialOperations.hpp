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

#include <GeographicLib/Geodesic.hpp>
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
    /////////////////////////////////////////
    // Declare public variables.
    /////////////////////////////////////////
    // Constants retrieved from: https://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html
    const double dEarthAverageRadius       = 6371000.0;    // Earth's radius in meters.
    const double dEarthEllipsoidFlattening = 0.003353;     // The flattening factor of the earth due to its spin.

    /******************************************************************************
     * @brief This struct is used to store the distance and arc length for a calculated
     *      geographic distance. Storing these values in a struct allows for easy
     *      handling and access to said variables.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-13
     ******************************************************************************/
    struct GeoDistance
    {
        public:
            // Define public struct attributes.
            double dDistanceMeters;
            double dArcLengthDegrees;
    };

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
            int nZone;                         // The UTM zone number identifying the region on the Earth's surface.
            bool bWithinNorthernHemisphere;    // Indicates whether the coordinate is located in the northern hemisphere.
            double dAltitude;                  // The elevation above sea level in meters.
            double d2DAccuracy;                // The horizontal accuracy of the UTM coordinates in meters.
            double d3DAccuracy;                // The three-dimensional accuracy, including altitude, in meters.
            double dMeridianConvergence;       // The angle between true north and grid north at the given location in degrees. Positive in eastern direction.
            double dScale;                     // The scale factor applied to the UTM coordinates for map projection.

            /////////////////////////////////////////
            // Declare public methods.
            /////////////////////////////////////////

            /******************************************************************************
             * @brief Default Construct a new UTMCoordinate object.
             *
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-10-12
             ******************************************************************************/
            UTMCoordinate()
            {
                // Initialize member variables.
                this->dEasting                  = 0.0;
                this->dNorthing                 = 0.0;
                this->nZone                     = 0;
                this->bWithinNorthernHemisphere = true;
                this->dAltitude                 = 0.0;
                this->d2DAccuracy               = -1.0;
                this->d3DAccuracy               = -1.0;
                this->dMeridianConvergence      = 0.0;
                this->dScale                    = 0.0;
            }

            /******************************************************************************
             * @brief Construct a new UTMCoordinate object.
             *
             * @param dEasting - The eastward displacement from the UTM zone's central meridian in meters.
             * @param dNorthing - The northward displacement from the equator in meters.
             * @param nZone - The UTM zone number identifying the region on the Earth's surface.
             * @param bWithinNorthernHemisphere - Indicates whether the coordinate is located in the northern hemisphere.
             * @param dAltitude - The elevation above sea level in meters.
             * @param d2DAccuracy - The horizontal accuracy of the UTM coordinates in meters.
             * @param d3DAccuracy - The three-dimensional accuracy, including altitude, in meters.
             * @param dMeridianConvergence - The angle between true north and grid north at the given location in degrees. Positive in eastern direction.
             * @param dScale - The scale factor applied to the UTM coordinates for map projection.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            UTMCoordinate(double dEasting,
                          double dNorthing,
                          int nZone,
                          bool bWithinNorthernHemisphere,
                          double dAltitude            = 0.0,
                          double d2DAccuracy          = -1.0,
                          double d3DAccuracy          = -1.0,
                          double dMeridianConvergence = -1.0,
                          double dScale               = 0.0)
            {
                // Initialize member variables with given values.
                this->dEasting                  = dEasting;
                this->dNorthing                 = dNorthing;
                this->nZone                     = nZone;
                this->bWithinNorthernHemisphere = bWithinNorthernHemisphere;
                this->dAltitude                 = dAltitude;
                this->d2DAccuracy               = d2DAccuracy;
                this->d3DAccuracy               = d3DAccuracy;
                this->dMeridianConvergence      = dMeridianConvergence;
                this->dScale                    = dScale;
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

    /******************************************************************************
     * @brief The shortest path between two points on an ellipsoid at (lat1, lon1) and (lat2, lon2) is called the geodesic.
     *      Given those two points create an ellipsoid with earth's characteristics and find the distance between them.
     *
     * @param stCoord1 - The first GPS coordinate.
     * @param stCoord2 - The second GPS coordinate.
     * @return GeoDistance - Struct containing the distance in meters and arc length degrees.
     *
     * @see https://geographiclib.sourceforge.io/C++/doc/classGeographicLib_1_1Geodesic.html#ae66c9cecfcbbcb1da52cb408e69f65de
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-13
     ******************************************************************************/
    inline GeoDistance CalculateGeoDistance(const GPSCoordinate stCoord1, const GPSCoordinate stCoord2)
    {
        // Create instance variables.
        GeoDistance stDistances;

        // Construct a geodesic with earth characteristics.
        GeographicLib::Geodesic geGeodesic(dEarthAverageRadius, dEarthEllipsoidFlattening);

        // Solve the inverse geodesic.
        stDistances.dArcLengthDegrees = geGeodesic.Inverse(stCoord1.dLatitude, stCoord1.dLongitude, stCoord2.dLatitude, stCoord2.dLongitude, stDistances.dDistanceMeters);

        // Return result distance.
        return stDistances;
    }
}    // namespace geoops
#endif
