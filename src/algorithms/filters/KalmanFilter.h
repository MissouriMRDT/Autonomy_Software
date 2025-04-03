#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <list>
#include <sl/Camera.hpp>

#include "../../util/GeospatialOperations.hpp"

using time_point_t = std::chrono::system_clock::time_point;

namespace filters
{
    /******************************************************************************
     * @brief Kalman Filter for combining positional, velocital, and rotational data sources.
     *
     *
     * @author Dr. Gant, translated by Adam
     * @date 2025-04-01
     ******************************************************************************/
    class KalmanFilter
    {
        public:
            struct XStateSnapshot
            {
                    Eigen::Vector3d eiPosition;
                    Eigen::Vector3d eiVelocity;
                    Eigen::Vector3d eiRotation;
                    // Eigen::Matrix3d eiXStateMatrix;    // [...p; ...v; ...r]
                    time_point_t tmTimestamp;
            };

            XStateSnapshot GetCurrentState();
            XStateSnapshot GetInterpolatedHistory(time_point_t tmTimestamp);

            void PredictAccelerometer(Eigen::Vector3d eiAccelerometerOutput, time_point_t tmTimestamp);
            void PredictGyroscope(Eigen::Vector3d eiGyroscopeOutput, time_point_t tmTimestamp);

            // TODO: Magnetometer for redundancy?

            void UpdateDiffGPS(Eigen::Vector3d eiDiffGPSOutputNEDFrame, time_point_t tmTimestamp);
            void UpdateHeading(Eigen::Vector3d dHeading, time_point_t tmTimestamp);

            void SetAccelerometerCovariance(Eigen::Matrix3d eiNewCovariance);
            void SetGyroscopeCovariance(Eigen::Matrix3d eiNewCovariance);
            void SetDiffGPSCovariance(Eigen::Matrix3d eiNewCovariance);
            void SetHeadingCovariance(Eigen::Matrix3d eiNewCovariance);

        private:
            // History for the last 10 seconds or so idk
            std::list<XStateSnapshot> m_liXStateHistory;
            // P - Current filter covariance matrix (9x9)
            Eigen::Matrix<double, 9, 9> m_eiPCovariance;
            // Time of last accelerometer update
            time_point_t m_tmLastAccelerometerUpdate;
            // Accelerometer covariance matrix (3x3)
            Eigen::Matrix3d m_eiAccelerometerCovariance;
            // Time of last gyro update
            time_point_t m_tmLastGyroscopeUpdate;
            // Gyroscope covariance matrix (3x3)
            Eigen::Matrix3d m_eiGyroscopeCovariance;
            // Time of last diff GPS update
            time_point_t m_tmLastDiffGPSUpdate;
            // Diff GPS covariance matrix (3x3)
            Eigen::Matrix3d m_eiDiffGPSCovariance;
            // Time of last heading update
            time_point_t m_tmLastHeadingUpdate;
            // Heading covariance matrix (3x3)
            Eigen::Matrix3d m_eiHeadingCovariance;

            // TODO: Add write mutexes for all of these

        private:
            static XStateSnapshot InterpolateXState(double dRatio, const XStateSnapshot& stBefore, const XStateSnapshot& stAfter);
            Eigen::Matrix3d GetQAccelerometer(double dDt);
            Eigen::Matrix3d GetQGyroscope(double dDt);
    };
}    // namespace filters

#endif    // KALMAN_FILTER_H
