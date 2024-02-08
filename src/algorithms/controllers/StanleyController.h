/******************************************************************************
 * @brief Defines the StanleyController class within the controllers namespace.
 *
 * @file StanleyController.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-01
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef STANLEY_CONTROLLER_H
#define STANLEY_CONTROLLER_H

#include "../../util/GeospatialOperations.hpp"
#include "../../util/NumberOperations.hpp"

/// \cond
// Put implicit includes in here.

#include <cmath>

/// \endcond

/******************************************************************************
 * @brief This namespace stores classes, functions, and structs that are used to
 *      implement different controllers that implement advanced control systems
 *      used for accurate and precise robotic control.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-01
 ******************************************************************************/
namespace controllers
{
    /******************************************************************************
     * @brief Provides an implementation of a lightweight lateral StanleyController.
     *      This algorithm is used to precisely control a different drive robot to
     *      follow a given path.
     *
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-01
     ******************************************************************************/
    class StanleyContoller
    {
        public:
            /////////////////////////////////////////
            // Declare public enums that are specific to and used withing this class.
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Declare public methods and member variables.
            /////////////////////////////////////////

            StanleyContoller(const double dKp, const double dDistToFrontAxle, const double dYawTolerance, const std::vector<geoops::UTMCoordinate> vPathUTM);
            ~StanleyContoller();

            double Calculate(const geoops::UTMCoordinate utmCurrentPos, const double dVelocity, const double dYaw);

            /////////////////////////////////////////
            // Declare public primary methods.
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Setters.
            /////////////////////////////////////////

            void SetProportionalGain(const double dKp);
            void SetDistanceToFrontAxle(const double dDistToFrontAxle);
            void SetYawTolerance(const double dYawTolerance);
            void SetPathUTM(std::vector<geoops::UTMCoordinate> vPathUTM);

            /////////////////////////////////////////
            // Getters.
            ////////////////////////////////////////

            double GetProportionalGain() const;
            double GetDistanceToFrontAxle() const;
            double GetYawTolerance() const;
            std::vector<geoops::UTMCoordinate> GetPathUTM() const;

        private:
            /////////////////////////////////////////
            // Declare private methods.
            /////////////////////////////////////////

            unsigned int IdentifyTargetIdx(const geoops::UTMCoordinate utmCurrentPos, const double dYaw) const;
            double CalculateCrossTrackError() const;
            double CalculateTargetYaw(const unsigned int unTargetIdx) const;
            geoops::UTMCoordinate CalculateFrontAxleCoordinate(const geoops::UTMCoordinate utmCurrentPos, const double dBearing) const;

            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////

            double m_dKp;                                     // Proportional gain.
            double m_dDistToFrontAxle;                        // Distance between the position sensor (GPS) and the front axle
            double m_dYawTolerance;                           // Minimum yaw change threshold for execution
            unsigned int m_unLastTargetIdx;                   // Index of last point on path used in Stanley calculation
            std::vector<geoops::UTMCoordinate> m_vPathUTM;    // Stores the sequence of UTM coordinates defining the navigational path
    };
}    // namespace controllers

#endif
