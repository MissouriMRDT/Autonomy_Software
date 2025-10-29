/******************************************************************************
 * @brief Implements the Extended Kalman Filter class.
 *
 * @file ExtendedKalmanFilter.cpp
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2025-09-27
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "ExtendedKalmanFilter.h"
#include "../../AutonomyConstants.h"
#include "../../vision/cameras/ZEDCam.h"

/******************************************************************************
 * @brief This namespace stores classes, functions, and structs used to implement the
 *      Extended Kalman Filter.
 *
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2025-09-27
 ******************************************************************************/
namespace filters
{
    /******************************************************************************
     * @brief Construct a new Extended Kalman Filter:: Extended Kalman Filter object.
     *
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-09-27
     ******************************************************************************/
    ExtendedKalmanFilter::ExtendedKalmanFilter()
    {
        // TODO: Initialize member variables
        m_dSigmaAcc  = constants::KALMAN_SIGMA_ACCELERATION;
        m_dSigmaGyro = constants::KALMAN_SIGMA_GYRO;
        // m_dSigmaAccBias  = constants::KALMAN_SIGMA_ACCELERATION_BIAS;
        // m_dSigmaGyroBias = constants::KALMAN_SIGMA_GYRO_BIAS;
        m_dSigmaGPSHor = constants::KALMAN_SIGMA_GPS_HORIZONTAL_ERROR;
        m_dSigmaGPSVer = constants::KALMAN_SIGMA_GPS_VERTICAL_ERROR;
        m_dSigmaYaw    = constants::KALMAN_SIGMA_YAW;
        // TODO: ugly math sad face
    }

    /******************************************************************************
     * @brief Construct a new Extended Kalman Filter:: Extended Kalman Filter object.
     *
     *@param stInitPose - The initial GPS and heading of the rover.
     *@param dSigmaAcc - The standard deviation of the acceleration.
     *@param dSigmaGyro - The standard deviation of the gyrometer.
     *@param dSigmaAccBias - The standard deviation of the acceleration bias.
     *@param dSigmaGyroBias - The standard deviation of the gyrometer bias.
     *@param dSigmaGPSHor - The standard deviation of the horizontal GPS.
     *@param dSigmaGPSVer - The standard deviation of the vertical GPS.
     *@param dSigmaYaw - The standard deviation of the yaw.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-09-30
     ******************************************************************************/
    ExtendedKalmanFilter::ExtendedKalmanFilter(const geoops::RoverPose& stInitPose,
                                               const Eigen::Matrix3d& eiAccelCov,
                                               const Eigen::Matrix3d& eiGyroCov,
                                               const double dSigmaAccel,
                                               const double dSigmaGyro,
                                               const geoops::GPSCoordinate& stInitGPS,
                                               const double dSigmaYaw)
    {
        // Initialize member variables
        m_eiAccelerometerCovariance = eiAccelCov;
        m_eiGyroscopeCovariance     = eiGyroCov;
        m_dSigmaAcc                 = dSigmaAccel;
        m_dSigmaGyro                = dSigmaGyro;
        m_dSigmaGPSHor              = stInitGPS.dLatitude;
        m_dSigmaGPSVer              = stInitGPS.dLongitude;
        // This will set the values for the position vector and orientation quaternion.
        RoverPoseToGPS(stInitPose, m_eiPosition);
        RoverPoseToOrientation(stInitPose, m_eiOrientation);

        // TODO: do the ugly math for initialization :sob: :cry:
    }

    /******************************************************************************
     * @brief This will set the initial guess for the Extended Kalman Filter.
     *
     * @param eiInitState - The state snapshot of the initial state.
     * @param eiInitCovariance - The initial overall noise covariance matrix for the filter.
     *
     * @author Sam Hajdukiewic (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-21
     ******************************************************************************/
    void ExtendedKalmanFilter::SetInitialGuess(const XStateSnapshot& eiInitState, const Eigen::Matrix<double, 15, 15>& eiInitCovariance)
    {
        // TODO: implement
        return;
    }

    /******************************************************************************
     * @brief This will set the GPS data noise.
     *
     * @param stCoord - The GPS coordinate.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-03
     ******************************************************************************/
    void ExtendedKalmanFilter::SetGPSNoise(const geoops::GPSCoordinate& stCoord)
    {
        // Clear existing covariance
        m_eiGPSCovariance.setZero();

        double dSigmaHor = (stCoord.d2DAccuracy > 0.0) ? stCoord.d2DAccuracy : 1.0;
        double dSigmaVer = (stCoord.d3DAccuracy > 0.0) ? stCoord.d3DAccuracy : 2.0;

        // Horizontal noise (X = East/West, Y = North/South)
        m_eiGPSCovariance(0, 0) = dSigmaHor * dSigmaHor;    // variance in X
        m_eiGPSCovariance(1, 1) = dSigmaHor * dSigmaHor;    // variance in Y

        // Vertical noise (Z = Up/Down)
        m_eiGPSCovariance(2, 2) = dSigmaVer * dSigmaVer;    // variance in Z
    }

    /******************************************************************************
     * @brief This will set the compass/heading noise.
     *
     * @param dSigmaYaw - The standard deviation of the yaw.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-22
     ******************************************************************************/
    void ExtendedKalmanFilter::SetCompassNoise(double dSigmaYaw)
    {
        m_eiHeadingCovariance.setZero();
        double dYawSquared = dSigmaYaw * dSigmaYaw;
        // (0, 0), (1, 1), and (2, 2)
        m_eiHeadingCovariance.diagonal() << dYawSquared, dYawSquared, dYawSquared;
    }

    /******************************************************************************
     * @brief The main predict/estimate step for EKF. Integrates IMU data to predict state.
     *
     * @param eiAccelMeas - The accerometer reading.
     * @param eiGyroMeas - The gyrometer reading.
     * @param tmTimestamp - The timestamp that the prediction has occurred.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-28
     ******************************************************************************/
    void ExtendedKalmanFilter::Predict(Eigen::Vector3d& eiAccelMeas, Eigen::Vector3d& eiGyroMeas, std::chrono::system_clock::time_point tmTimestamp)
    {
        // Must have an initial guess
        if (!m_bHasInitialGuess)
            return;

        double dt                    = std::chrono::duration<double>(tmTimestamp - m_stInitialState.tmTimestamp).count();
        m_stInitialState.tmTimestamp = tmTimestamp;

        // Accelerometer and gyrometer bias removal
        Eigen::Vector3d eiAcc   = eiAccelMeas - m_stInitialState.eiAccelBias;
        Eigen::Vector3d eiGyro  = eiGyroMeas - m_stInitialState.eiGyroBias;

        Eigen::Vector3d eiOmega = eiGyroMeas * dt;
        double dAngle           = eiOmega.norm();

        Eigen::Quaterniond eiDq;

        // If not 0
        if (dAngle > 0)
            eiDq = Eigen::Quaterniond(Eigen::AngleAxisd(dAngle, eiOmega.normalized()));

        // Identity quaternion
        else
            eiDq = Eigen::Quaterniond::Identity();

        m_eiOrientation = (m_eiOrientation * eiDq).normalized();

        // Acceleration in the world frame (accounts for gravity)
        Eigen::Vector3d eiAccWorldFrame = (m_eiOrientation * eiAcc) + m_eiGravity;

        // TODO: Check if this is correct
        m_stInitialState.eiVelocity += eiAccWorldFrame * dt;
        m_eiPosition += (m_stInitialState.eiVelocity * dt) + (eiAccWorldFrame * dt * dt) / 2.0;

        // Covariance
        Eigen::Matrix<double, 15, 15> eiF = Eigen::Matrix<double, 15, 15>::Zero();

        eiF.block<3, 3>(0, 3)             = Eigen::Matrix3d::Identity();
        // TODO: figure out how to get skew-symmetric matrix
        eiF.block<3, 3>(3, 9)              = -1.0 * m_eiOrientation.toRotationMatrix();
        eiF.block<3, 3>(6, 12)             = -1.0 * Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 15, 15> eiFd = Eigen::Matrix<double, 15, 15>::Identity() + eiF * dt;

        // Process noise Q
        Eigen::Matrix<double, 15, 15> eiQ = Eigen::Matrix<double, 15, 15>::Zero();
        double dt2                        = dt * dt;
        eiQ.block<3, 3>(3, 3)             = (m_dSigmaAcc * m_dSigmaAcc) * Eigen::Matrix3d::Identity() * dt2;
        eiQ.block<3, 3>(6, 6)             = (m_dSigmaGyro * m_dSigmaGyro) * Eigen::Matrix3d::Identity() * dt2;
        eiQ.block<3, 3>(9, 9)             = (m_dSigmaAccBias * m_dSigmaAccBias) * Eigen::Matrix3d::Identity() * dt2;
        eiQ.block<3, 3>(12, 12)           = (m_dSigmaGyroBias * m_dSigmaGyroBias) * Eigen::Matrix3d::Identity() * dt2;

        // Error state covariance
        m_eiErrorStateCov = eiFd * m_eiErrorStateCov * eiFd.transpose() + eiQ;
    }

    /******************************************************************************
     * @brief This will update the GPS noise.
     *
     * @param stCoord - The GPS coordinate.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-21
     ******************************************************************************/
    void ExtendedKalmanFilter::UpdateGPS(const geoops::GPSCoordinate& stCoord)
    {
        // TODO: Look into this and see if math needs to be changed
        //  Check if there is an initial guess set.
        if (!m_bHasInitialGuess)
            return;

        // Save time updated.
        m_tmLastGPSUpdate = stCoord.tmTimestamp;

        // Convert GPS to ENU
        Eigen::Vector3d eiZMeasure = ConvertGPSToENU(stCoord);

        // Build measurement noise matrix (R)
        double dSigma_xy        = (stCoord.d2DAccuracy > 0.1) ? stCoord.d2DAccuracy : 1.0;
        double dSigma_z         = (stCoord.d3DAccuracy > 0.1) ? stCoord.d3DAccuracy : 2.0;

        Eigen::Matrix3d eiR_gps = Eigen::Matrix3d::Zero();
        eiR_gps(0, 0)           = dSigma_xy * dSigma_xy;
        eiR_gps(1, 1)           = dSigma_xy * dSigma_xy;
        eiR_gps(2, 2)           = dSigma_z * dSigma_z;

        // Extract predicted state
        RoverPoseToGPS(m_stInitialState.stPose, m_eiPosition);
        Eigen::Vector3d eiXpred = m_eiPosition;                           // From RoverPose
        Eigen::Matrix3d eiPpos  = m_eiErrorStateCov.block<3, 3>(0, 0);    // top-left 3x3 position covariance

        //  Compute innovation (residual)
        Eigen::Vector3d eiY_tilde = eiZMeasure - eiXpred;

        // Compute innovation covariance (S)
        Eigen::Matrix3d eiS = eiPpos + eiR_gps;

        // Compute Kalman gain (K)
        Eigen::Matrix3d eiK = eiPpos * eiS.inverse();

        // Update state estimate
        Eigen::Vector3d eiXUpdate = eiXpred + eiK * eiY_tilde;

        // Store updated position back into pose
        m_eiPosition = eiXUpdate;

        // Update covariance
        Eigen::Matrix3d eiI = Eigen::Matrix3d::Identity();
        eiPpos              = (eiI - eiK) * eiPpos;

        // Write updated block back into full covariance
        m_eiErrorStateCov.block<3, 3>(0, 0) = eiPpos;

        return;
    }

    /******************************************************************************
     * @brief This will convert a GPS coordinate into ENU.
     *
     * @param stCoord - The GPS coordinate.
     * @return Eigen::Vector3d - The ENU vector.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-22
     ******************************************************************************/
    Eigen::Vector3d ExtendedKalmanFilter::ConvertGPSToENU(const geoops::GPSCoordinate& stCoord)
    {
        static geoops::GPSCoordinate stGpsRef = stCoord;      // First coordinate for reference
        double dEarthRadius                   = 6378137.0;    // Meters

        double dLat                           = (stCoord.dLatitude - stGpsRef.dLatitude) * M_PI / 180.0;
        double dLon                           = (stCoord.dLongitude - stGpsRef.dLongitude) * M_PI / 180.0;
        double avgLat                         = (stCoord.dLatitude + stGpsRef.dLatitude) * 0.5 * M_PI / 180.0;

        double dx                             = dEarthRadius * dLon * cos(avgLat);         // East
        double dy                             = dEarthRadius * dLat;                       // North
        double dz                             = stCoord.dAltitude - stGpsRef.dAltitude;    // Up

        return Eigen::Vector3d(dx, dy, dz);
    }

    /******************************************************************************
     * @brief Converts a RoverPose to orientation quaternion.
     *
     * @param stPose - The current RoverPose.
     * @param eiOrientation - The orientation quaternion.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-03
     ******************************************************************************/
    void ExtendedKalmanFilter::RoverPoseToOrientation(const geoops::RoverPose& stPose, Eigen::Quaterniond& eiOrientation) const
    {
        // Convert heading to orientation quaternion
        double dHeading = stPose.GetCompassHeading();
        eiOrientation   = Eigen::AngleAxisd(dHeading, Eigen::Vector3d::UnitZ());
    }

    /******************************************************************************
     * @brief Converts a RoverPose to position vector.
     *
     * @param stPose - The current RoverPose.
     * @param eiPosition - The position vector.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-03
     ******************************************************************************/
    void ExtendedKalmanFilter::RoverPoseToGPS(const geoops::RoverPose& stPose, Eigen::Vector3d& eiPosition) const
    {
        // Convert GPSCoordinate to position vector
        eiPosition(0) = stPose.GetGPSCoordinate().dLatitude;
        eiPosition(1) = stPose.GetGPSCoordinate().dLongitude;
        eiPosition(2) = stPose.GetGPSCoordinate().dAltitude;
    }

    /******************************************************************************
     * @brief This converts a position and orientation vector into a single RoverPose.
     *
     * @param eiPosition - The position vector.
     * @param eiOrientation - The orientation quaternion.
     * @return RoverPose - GPS and orientation of the rover.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-03
     ******************************************************************************/
    geoops::RoverPose ExtendedKalmanFilter::ToRoverPose(const Eigen::Vector3d& eiPosition, const Eigen::Quaterniond& eiOrientation) const
    {
        // Convert position vector to GPSCoordinate
        geoops::GPSCoordinate stCoord;
        stCoord.dLatitude  = eiPosition(0);
        stCoord.dLongitude = eiPosition(1);
        stCoord.dAltitude  = eiPosition(2);
        // Convert orientation quaternion to heading
        double dHeading = atan2(2.0 * (eiOrientation.x() * eiOrientation.y() + eiOrientation.w() * eiOrientation.z()),
                                eiOrientation.w() * eiOrientation.w() - eiOrientation.x() * eiOrientation.x() - eiOrientation.y() * eiOrientation.y() +
                                    eiOrientation.z() * eiOrientation.z());
        return geoops::RoverPose(stCoord, dHeading);
    }

}    // namespace filters
