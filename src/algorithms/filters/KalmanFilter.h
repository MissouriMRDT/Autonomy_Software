#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <list>
#include <sl/Camera.hpp>

#include "../../util/GeospatialOperations.hpp"

namespace filters
{
    /******************************************************************************
     * @brief Kalman Filter for combining positional, velocital, and rotational data sources.
     * The Kalman Filter fuses the ZED's accelerometer data with position/heading measurements
     * from differential GPS to produce a combined estimated state that is more resilient to error.
     *
     *
     * @author Dr. Gant, translated by Adam, written on Sam's computer
     * @date 2025-04-01
     ******************************************************************************/
    class KalmanFilter
    {
        public:
            /******************************************************************************
             * @brief A snapshot of the position, velocity, and rotation of the rover at the given time.
             *
             *
             * @author Adam
             * @date 2025-05-20
             ******************************************************************************/
            struct XStateSnapshot
            {
                    Eigen::Vector3d eiPosition;                           // X, Y, Z
                    Eigen::Vector3d eiVelocity;                           // Vx, Vy, Vz
                    Eigen::Vector3d eiRotation;                           // Rx, Ry, Rz
                    std::chrono::system_clock::time_point tmTimestamp;    // The time at which this data state was recorded.
                                                                          // Eigen::Matrix3d eiXStateMatrix;    // [...p; ...v; ...r]
            };

            void SetInitialGuess(const geoops::UTMCoordinate& stOrigin, const double dHeading);

            XStateSnapshot GetCurrentState() const;
            XStateSnapshot GetInterpolatedHistory(std::chrono::system_clock::time_point tmTimestamp) const;
            geoops::RoverPose GetCurrentPose() const;

            void PredictAccelerometer(Eigen::Vector3d eiAccelerometerOutput, std::chrono::system_clock::time_point tmTimestamp);
            void PredictGyroscope(Eigen::Vector3d eiGyroscopeOutput, std::chrono::system_clock::time_point tmTimestamp);

            // TODO: Magnetometer for redundancy?

            void IngestGPSData(const geoops::UTMCoordinate& stMeasurement);
            void UpdateGPS(Eigen::Vector3d eiGPSOutputNEDFrame, std::chrono::system_clock::time_point tmTimestamp);
            void UpdateHeading(Eigen::Vector3d dHeading, std::chrono::system_clock::time_point tmTimestamp);

            void SetAccelerometerCovariance(Eigen::Matrix3d eiNewCovariance);
            void SetGyroscopeCovariance(Eigen::Matrix3d eiNewCovariance);
            void SetGPSCovariance(Eigen::Matrix3d eiNewCovariance);
            void SetHeadingCovariance(Eigen::Matrix3d eiNewCovariance);

        private:
            bool m_bHasInitialGuess = false;                                      // Whether an initial state has been provided and the Kalman Filter thus initialized.
            geoops::RoverPose m_stInitialGuess;                                   // The absolute coordinate to serve as the origin.
            std::chrono::duration<std::chrono::milliseconds> m_tiHistoryLimit;    // How far back m_liXStateHistory should be recorded.
            std::list<XStateSnapshot> m_liXStateHistory;    // All estimates made in the last m_tiHistoryLimit period, with new estimates inserted at the back.
            Eigen::Matrix<double, 9, 9> m_eiPCovariance;    // P - Current filter covariance matrix. (9x9)
            std::chrono::system_clock::time_point m_tmLastAccelerometerUpdate;    // Time of last accelerometer update.
            Eigen::Matrix3d m_eiAccelerometerCovariance;                          // Accelerometer covariance matrix. (3x3)
            std::chrono::system_clock::time_point m_tmLastGyroscopeUpdate;        // Time of last gyro update.
            Eigen::Matrix3d m_eiGyroscopeCovariance;                              // Gyroscope covariance matrix. (3x3)
            std::chrono::system_clock::time_point m_tmLastGPSUpdate;              // Time of last diff GPS update.
            Eigen::Matrix3d m_eiGPSCovariance;                                    // Diff GPS covariance matrix. (3x3)
            std::chrono::system_clock::time_point m_tmLastHeadingUpdate;          // Time of last heading update.
            Eigen::Matrix3d m_eiHeadingCovariance;                                // Heading covariance matrix. (3x3)

            // TODO: Add write mutexes for all of these

        private:
            static XStateSnapshot InterpolateXState(double dRatio, const XStateSnapshot& stBefore, const XStateSnapshot& stAfter);
            Eigen::Matrix3d GetQAccelerometer(double dDt);
            Eigen::Matrix3d GetQGyroscope(double dDt);
    };
}    // namespace filters

#endif    // KALMAN_FILTER_H
