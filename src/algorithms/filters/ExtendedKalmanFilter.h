/******************************************************************************
 * @brief Defines the Extended Kalman Filter. The purpose is to filter out inaccurate GPS data
 *
 * @file ExtendedKalmanFilter.h
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2025-09-27
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include "../../util/GeospatialOperations.hpp"
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <shared_mutex>

namespace filters
{
    /******************************************************************************
     * @brief The Extended Kalman filter is used to filter out inaccuracies in our GPS data.
     * Our inputs include IMU data, GPS, and our heading from our compass.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-09-10
     ******************************************************************************/

    class ExtendedKalmanFilter
    {
        public:
            /////////////////////////////////////////
            // Declare public class structs.
            /////////////////////////////////////////

            /******************************************************************************
             * @brief A snapshot at any given time of our position, velocity, acceleration bias, and orientation.
             *
             *
             * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com) (some code taken from Adam)
             * @date 2025-09-27
             ******************************************************************************/
            struct XStateSnapshot
            {
                    Eigen::Vector3d eiPosition;                           // GPS for the rover
                    Eigen::Quaterniond eiOrientation;                     // Orientation of the rover
                    Eigen::Vector3d eiVelocity;                           // X, Y, and Z velocities
                    Eigen::Vector3d eiAccelBias;                          // X, Y, Z acceleration biases
                    Eigen::Vector3d eiGyroBias;                           // Gyroscope biases
                    std::chrono::system_clock::time_point tmTimestamp;    // When this state snapshot was recorded
            };

            /////////////////////////////////////////
            // Declare public class methods.
            /////////////////////////////////////////

            ExtendedKalmanFilter(const geoops::RoverPose& stInitPose,
                                 const Eigen::Matrix3d& eiAccelCov = Eigen::Matrix3d::Identity(),
                                 const Eigen::Matrix3d& eiGyroCov  = Eigen::Matrix3d::Identity());
            ~ExtendedKalmanFilter();

            /////////////////////////////////////////
            // Setters.
            /////////////////////////////////////////

            void SetInitialGuess(const geoops::RoverPose& stInitPose);
            void SetGPSNoise(const geoops::GPSCoordinate& stCoord);
            void SetCompassNoise(double dSigmaDeg);

            /////////////////////////////////////////
            // Prediction and updating.
            /////////////////////////////////////////

            // Method for prediction with accelerometer and gyroscope
            void Predict(Eigen::Vector3d& eiAccelMeas, Eigen::Vector3d& eiGyroMeas, std::chrono::system_clock::time_point tmTimestamp);

            //  Methods for updating values
            void UpdateGPS(const geoops::GPSCoordinate& stCoord);
            void UpdateCompass(const double dHeading);

            /////////////////////////////////////////
            // Getters.
            /////////////////////////////////////////

            XStateSnapshot GetCurrentState() const;
            geoops::RoverPose GetEstimatedRoverPose() const;

        private:
            /////////////////////////////////////////
            // Declare private class methods.
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Conversions.
            /////////////////////////////////////////
            Eigen::Vector3d ConvertGPSToENU(const geoops::GPSCoordinate& stCoord);
            geoops::GPSCoordinate ConvertENUToGPS(const Eigen::Vector3d& eiPosition) const;
            Eigen::Matrix3d MakeSkewSymmetricMatrix(const Eigen::Vector3d& eiVec);

            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////

            mutable std::shared_mutex m_muStateMutex;                   // Mutex when updating filter.
            bool m_bHasInitialGuess = false;                            // Whether or not there is an initial guess.
            bool m_bOriginSet       = false;                            // Whether or not there is an initial GPS set.
            geoops::GPSCoordinate m_stOriginGPS;                        // The original GPS coordinate set.
            XStateSnapshot m_stCurrentState;                            // The current state.
            Eigen::Matrix<double, 15, 15> m_eiErrorStateCov;            // The covariance matrix for the error-state.
            std::chrono::system_clock::time_point m_tmLastIMUUpdate;    // Time of last IMU update.
            Eigen::Matrix3d m_eiAccelerometerCovariance;                // Accelerometer covariance matrix. (3x3)
            Eigen::Matrix3d m_eiGyroscopeCovariance;                    // Gyroscope covariance matrix. (3x3)
            Eigen::Matrix3d m_eiGPSCovariance;                          // Diff GPS covariance matrix. (3x3)
            Eigen::Vector3d m_eiGravity;                                // Vector for gravity.
            double m_dSigmaAccBias;
            double m_dSigmaGyroBias;
            double m_dSigmaGPSHor;
            double m_dSigmaGPSVer;
            double m_dSigmaYaw;
    };
}    // namespace filters

#endif
