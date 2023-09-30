/******************************************************************************
 * @brief Defines and implements functions related to operations on numbers within
 * 		the GeospatialOperations namespace.
 *
 * @file NumberOperations.hpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef GEOSPATIAL_OPERATIONS_HPP
#define GEOSPATIAL_OPERATIONS_HPP

/******************************************************************************
 * @brief Namespace containing functions related to operations on gobal position number
 *      systems and other datatypes.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
namespace geoops
{
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
             * @param d2DAccurary - The accuracy of the lat/lon
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
            double dZone;
            double d2DAccuracy;
            double d3DAccuracy;
    };
}    // namespace geoops
#endif
