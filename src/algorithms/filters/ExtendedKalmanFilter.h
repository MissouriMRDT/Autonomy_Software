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
#include <list>
#include <sl/Camera.hpp>

using woid = geoops::RoverPose;

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
                    geoops::RoverPose stPose;                             // GPS and heading for the rover
                    Eigen::Vector3d eiVelocity;                           // X, Y, and Z velocities
                    Eigen::Vector3d eiAccelBias;                          // X, Y, Z acceleration biases
                    Eigen::Vector3d eiGyroBias;                           // Gyroscope biases
                    std::chrono::system_clock::time_point tmTimestamp;    // When this state snapshot was recorded
            };

            /////////////////////////////////////////
            // Declare public class methods.
            /////////////////////////////////////////

            ExtendedKalmanFilter();
            // TODO: change inputs if needed
            ExtendedKalmanFilter(const geoops::RoverPose stInitPose,
                                 const Eigen::Vector3d& eiInitVel,
                                 double dSigmaAcc,
                                 double dSigmaGyro,
                                 double dSigmaAccBias,
                                 double dSigmaGyroBias,
                                 double dSigmaGPSHor,
                                 double dSigmaGPSVer,
                                 double dSigmaYaw);
            ~ExtendedKalmanFilter();

            /////////////////////////////////////////
            // Setters.
            /////////////////////////////////////////

            // TODO: Figure out what should be const
            void SetInitialGuess(XStateSnapshot& eiInitState, Eigen::Matrix<double, 15, 15>& eiInitCovariance);
            void SetIMUNoise(double dSigmaAcc, double dSigmaGyro, double dSigmaAccBias, double dSigmaGyroBias);
            void SetGPSNoise(double dSigmaHor, double dSigmaVer);
            void SetCompassNoise(double dSigmaYaw);

            /////////////////////////////////////////
            // Prediction and updating.
            /////////////////////////////////////////

            // Method for prediction with accelerometer and gyroscope
            void Predict(Eigen::Vector3d& eiAccelMeas, Eigen::Vector3d& eiGyroMeas, std::chrono::system_clock::time_point tmTimestamp);

            //  Methods for updating values
            void UpdateGPS(const geoops::GPSCoordinate& stCoord, std::chrono::system_clock::time_point tmTimestamp);
            void UpdateYaw(double dYaw, std::chrono::system_clock::time_point tmTimestamp);
            void UpdateHeading(Eigen::Vector3d dHeading, std::chrono::system_clock::time_point tmTimestamp);

            /////////////////////////////////////////
            // Getters.
            /////////////////////////////////////////

            const XStateSnapshot& GetCurrentState() const;

            /////////////////////////////////////////
            // Conversions.
            /////////////////////////////////////////
            // Takes position and orientation/heading vectors to turn them into RoverPoses.
            woid ToRoverPose(const Eigen::Vector3d& eiPosition, const Eigen::Quaterniond& eiOrientation) const;
            // Takes RoverPose and converts it into position and orientation vectors.
            void FromRoverPose(const geoops::RoverPose& stPose, Eigen::Vector3d& eiPosition, Eigen::Quaterniond& eiOrientation) const;

            // TODO: go back and see if any member vars are missing
        private:
            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////

            bool m_bHasInitialGuess = false;                                      // Whether or not there is an initial guess
            XStateSnapshot m_stInitialState;                                      // To store the original state snapshot
            std::chrono::duration<std::chrono::milliseconds> m_tiHistoryLimit;    // How far back m_liXStateHistory should be recorded.
            std::list<XStateSnapshot> m_liXStateHistory;        // All estimates made in the last m_tiHistoryLimit period, with new estimates inserted at the back.
            Eigen::Matrix<double, 15, 15> m_eiErrorStateCov;    // The covariance matrix for the error-state
            Eigen::Matrix<double, 12, 12> m_eiIMUNoise;         // The continuous noise from the IMU
            std::chrono::system_clock::time_point m_tmLastAccelerometerUpdate;    // Time of last accelerometer update.
            Eigen::Matrix3d m_eiAccelerometerCovariance;                          // Accelerometer covariance matrix. (3x3)
            std::chrono::system_clock::time_point m_tmLastGyroscopeUpdate;        // Time of last gyro update.
            Eigen::Matrix3d m_eiGyroscopeCovariance;                              // Gyroscope covariance matrix. (3x3)
            std::chrono::system_clock::time_point m_tmLastGPSUpdate;              // Time of last diff GPS update.
            Eigen::Matrix3d m_eiGPSCovariance;                                    // Diff GPS covariance matrix. (3x3)
            std::chrono::system_clock::time_point m_tmLastHeadingUpdate;          // Time of last heading update.
            Eigen::Matrix3d m_eiHeadingCovariance;                                // Heading covariance matrix. (3x3)
            Eigen::Vector3d m_eiGravity;                                          // Vector for gravity
    };
};    // namespace filters

#endif
