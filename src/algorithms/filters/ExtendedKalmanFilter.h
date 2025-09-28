#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include "../../util/GeospatialOperations.hpp"
#include "../../vision/cameras/ZEDCam.h"
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <list>
#include <sl/Camera.hpp>

namespace filters
{
    /******************************************************************************
     * @brief The Extended Kalman filter is used to filter out inaccuracies in our GPS data.
     * Our inputs include IMU data, GPS, and our heading from our compass.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-09-10
     ******************************************************************************/

    class extended_kalman_filter
    {
        public:
            /******************************************************************************
             * @brief A snapshot at any given time of our position, velocity, acceleration bias, and orientation.
             * gyroscope
             *
             * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
             * @date 2025-09-27
             ******************************************************************************/
            struct XStateSnapshot
            {
                    Eigen::Vector3d eiPosition;          // X, Y, and Z position
                    Eigen::Vector3d eiVelocity;          // X, Y, and Z velocities
                    Eigen::Quaterniond eiOrientation;    // X, Y, Z orientations relative to world TODO: might make this RoverPose instead
                    Eigen::Vector3d eiAccelBias;         // X, Y, Z acceleration biases
                    Eigen::Vector3d eiGyroBias;          // Gyroscope biases
            };

            // TODO: Figure out what should be const
            // Methods for setting noise values
            void setIMUNoise(double dSigmaAcc, double dSigmaGyro, double dSigmaAccBias, double dSigmaGyroBias);
            void setGPSNoise(Eigen::Matrix3d eiGPS);
            void setCompassNoise(double dSigmaYaw);

        private:
            bool m_bHasInitialGuess;
    };
};    // namespace filters

#endif
