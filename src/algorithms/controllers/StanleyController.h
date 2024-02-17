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
#include <gtest/gtest.h>
#include <limits>
#include <vector>

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
     * @note In this class heading/bearing refers to the absolute orientation of a rover (N,E,S,W).
     * While yaw refers to an angle relative to the agent's current orientation.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-01
     ******************************************************************************/
    class StanleyController
    {
        public:
            /////////////////////////////////////////
            // Declare public methods and member variables.
            /////////////////////////////////////////

            StanleyController(const double dK, const double dDistToFrontAxle, const double dYawTolerance);
            StanleyController(const std::vector<geoops::UTMCoordinate>& vUTMPath, const double dK, const double dDistToFrontAxle, const double dYawTolerance);
            StanleyController(const std::vector<geoops::GPSCoordinate>& vGPSPath, const double dK, const double dDistToFrontAxle, const double dYawTolerance);
            ~StanleyController();

            double Calculate(const geoops::UTMCoordinate& stUTMCurrPos, const double dVelocity, const double dBearing);
            double Calculate(const geoops::GPSCoordinate& stGPSCurrPos, const double dVelocity, const double dBearing);
            void ResetProgress();

            /////////////////////////////////////////
            // Setters.
            /////////////////////////////////////////

            void SetSteeringControlGain(const double dKp);
            void SetDistanceToFrontAxle(const double dDistToFrontAxle);
            void SetYawTolerance(const double dYawTolerance);

            void SetPath(std::vector<geoops::UTMCoordinate>& vUTMPath);
            void SetPath(std::vector<geoops::GPSCoordinate>& vGPSPath);

            /////////////////////////////////////////
            // Getters.
            ////////////////////////////////////////

            double GetSteeringControlGain() const;
            double GetDistanceToFrontAxle() const;
            double GetYawTolerance() const;
            unsigned int GetLastTargetIdx() const;
            std::vector<geoops::UTMCoordinate> GetPathUTM() const;
            std::vector<geoops::GPSCoordinate> GetPathGPS() const;

        private:
            /////////////////////////////////////////
            // Declare private methods.
            /////////////////////////////////////////

            geoops::UTMCoordinate CalculateFrontAxleCoordinate(const geoops::UTMCoordinate& stUTMCurrPos, const double dBearing) const;
            unsigned int IdentifyTargetIdx(const geoops::UTMCoordinate& stUTMFrontAxlePos) const;
            double CalculateTargetBearing(const unsigned int unTargetIdx) const;
            double CalculateCrossTrackError(const geoops::UTMCoordinate& stUTMFrontAxlePos, const unsigned int unTargetIdx, const double dBearing) const;

            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////

            double m_dK;                                      // Steering control gain.
            double m_dDistToFrontAxle;                        // Distance between the position sensor (GPS) and the front axle.
            double m_dYawTolerance;                           // Minimum yaw change threshold for execution.
            unsigned int m_unLastTargetIdx;                   // Index of last point on path used in Stanley calculation.
            std::vector<geoops::UTMCoordinate> m_vUTMPath;    // Stores the sequence of UTM coordinates defining the navigational path.
            std::vector<geoops::GPSCoordinate> m_vGPSPath;    // Stores the sequence of GPS coordinates defining the navigational path.
    };
}    // namespace controllers

#endif
