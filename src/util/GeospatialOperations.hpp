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
            double dLatitude;
            double dLongitude;
            double dAltitude;
            double d2DAccuracy;
            double d3DAccuracy;

            /////////////////////////////////////////
            // Declare public methods.
            /////////////////////////////////////////

            /******************************************************************************
             * @brief Construct a new GPSCoordinate object.
             *
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            GPSCoordinate()
            {
                // Initialize member variables to default values.
                this->dLatitude   = 0.0;
                this->dLongitude  = 0.0;
                this->dAltitude   = 0.0;
                this->d2DAccuracy = -1.0;
                this->d3DAccuracy = -1.0;
            }

            /******************************************************************************
             * @brief Construct a new GPSCoordinate object.
             *
             * @param dLatitude - The latitude of the GPS coordinate.
             * @param dLongitude - The longitude of the GPS coordinate.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            GPSCoordinate(double dLatitude, double dLongitude)
            {
                // Initialize member variables with given values.
                this->dLatitude  = dLatitude;
                this->dLongitude = dLongitude;

                // Use default values for everything else.
                dAltitude   = 0.0;
                d2DAccuracy = -1.0;
                d3DAccuracy = -1.0;
            }

            /******************************************************************************
             * @brief Construct a new GPSCoordinate object.
             *
             * @param dLatitude - The latitude of the GPS coordinate.
             * @param dLongitude - The longitude of the GPS coordinate.
             * @param Altitude - The altitude of the GPS coordinate.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            GPSCoordinate(double dLatitude, double dLongitude, double dAltitude)
            {
                // Initialize member variables with given values.
                this->dLatitude  = dLatitude;
                this->dLongitude = dLongitude;
                this->dAltitude  = dAltitude;

                // Use default values for everything else.
                d2DAccuracy = -1.0;
                d2DAccuracy = -1.0;
            }

            /******************************************************************************
             * @brief Construct a new GPSCoordinate object.
             *
             * @param dLatitude - The latitude of the GPS coordinate.
             * @param dLongitude - The longitude of the GPS coordinate.
             * @param Altitude - The altitude of the GPS coordinate.
             * @param d2DAccuracy - The accuracy of the lat/lon
             * @param d3DAccuracy - The accuracy of the lat/lon/alt
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            GPSCoordinate(double dLatitude, double dLongitude, double dAltitude, double d2DAccuracy, double d3DAccuracy)
            {
                // Initialize member variables with given values.
                this->dLatitude   = dLatitude;
                this->dLongitude  = dLongitude;
                this->dAltitude   = dAltitude;
                this->d2DAccuracy = d2DAccuracy;
                this->d3DAccuracy = d3DAccuracy;
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
            double dEasting;
            double dNorthing;
            double dAltitude;
            int nZone;
            double d2DAccuracy;
            double d3DAccuracy;

            /////////////////////////////////////////
            // Declare public methods.
            /////////////////////////////////////////
            /******************************************************************************
             * @brief Construct a new UTMCoordinate object.
             *
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            UTMCoordinate()
            {
                // Initialize member variables to default values.
                this->dEasting    = 0.0;
                this->dNorthing   = 0.0;
                this->dAltitude   = 0.0;
                this->nZone       = 0;
                this->d2DAccuracy = -1.0;
                this->d3DAccuracy = -1.0;
            }

            /******************************************************************************
             * @brief Construct a new UTMCoordinate object.
             *
             * @param dEasting - The Easting of the UTM coordinate.
             * @param dNorthing - The Northing of the UTM coordinate.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            UTMCoordinate(double dEasting, double dNorthing)
            {
                // Initialize member variables with given values.
                this->dEasting  = dEasting;
                this->dNorthing = dNorthing;

                // Use default values for everything else.
                dAltitude   = 0.0;
                nZone       = 0;
                d2DAccuracy = -1.0;
                d3DAccuracy = -1.0;
            }

            /******************************************************************************
             * @brief Construct a new UTMCoordinate object.
             *
             * @param dEasting - The Easting of the UTM coordinate.
             * @param dNorthing - The Northing of the UTM coordinate.
             * @param dAltitude - The Altitude of the UTM coordinate.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            UTMCoordinate(double dEasting, double dNorthing, double dAltitude)
            {
                // Initialize member variables with given values.
                this->dEasting  = dEasting;
                this->dNorthing = dNorthing;
                this->dAltitude = dAltitude;

                // Use default values for everything else.
                nZone       = 0;
                d2DAccuracy = -1.0;
                d2DAccuracy = -1.0;
            }

            /******************************************************************************
             * @brief Construct a new UTMCoordinate object.
             *
             * @param dEasting - The Easting of the UTM coordinate.
             * @param dNorthing - The Northing of the UTM coordinate.
             * @param dAltitude - The Altitude of the UTM coordinate.
             * @param nZone - The zone of the UTM coordinate.
             * @param d2DAccuracy - The accuracy of the east / north
             * @param d3DAccuracy - The accuracy of the east / north / alt
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            UTMCoordinate(double dEasting, double dNorthing, double dAltitude, int nZone, double d2DAccuracy, double d3DAccuracy)
            {
                // Initialize member variables with given values.
                this->dEasting    = dEasting;
                this->dNorthing   = dNorthing;
                this->dAltitude   = dAltitude;
                this->nZone       = nZone;
                this->d2DAccuracy = d2DAccuracy;
                this->d3DAccuracy = d3DAccuracy;
            }
    };
}    // namespace geoops
#endif