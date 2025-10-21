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
        m_dSigmaAcc      = constants::KALMAN_SIGMA_ACCELERATION;
        m_dSigmaGyro     = constants::KALMAN_SIGMA_GYRO;
        m_dSigmaAccBias  = constants::KALMAN_SIGMA_ACCELERATION_BIAS;
        m_dSigmaGyroBias = constants::KALMAN_SIGMA_GYRO_BIAS;
        m_dSigmaGPSHor   = constants::KALMAN_SIGMA_GPS_HORIZONTAL_ERROR;
        m_dSigmaGPSVer   = constants::KALMAN_SIGMA_GPS_VERTICAL_ERROR;
        m_dSigmaYaw      = constants::KALMAN_SIGMA_YAW;
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
    ExtendedKalmanFilter::ExtendedKalmanFilter(const geoops::RoverPose stInitPose,
                                               double dSigmaAcc,
                                               double dSigmaGyro,
                                               double dSigmaAccBias,
                                               double dSigmaGyroBias,
                                               double dSigmaGPSHor,
                                               double dSigmaGPSVer,
                                               double dSigmaYaw)
    {
        // Initialize member variables
        m_dSigmaAcc      = dSigmaAcc;
        m_dSigmaGyro     = dSigmaGyro;
        m_dSigmaAccBias  = dSigmaAccBias;
        m_dSigmaGyroBias = dSigmaGyroBias;
        m_dSigmaGPSHor   = dSigmaGPSHor;
        m_dSigmaGPSVer   = dSigmaGPSVer;
        // This will set the values for the position vector and orientation quaternion.
        FromRoverPose(stInitPose, m_eiPosition, m_eiOrientation);

        // TODO: do the ugly math for initialization :sob: :cry:
    }

    /******************************************************************************
     * @brief Converts a RoverPose to position and orientation vectors to make vector math easier.
     *
     * @param stPose - The current RoverPose.
     * @param eiPosition - The position vector.
     * @param eiOrientation - The orientation quaternion.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-03
     ******************************************************************************/
    void ExtendedKalmanFilter::FromRoverPose(const geoops::RoverPose& stPose, Eigen::Vector3d& eiPosition, Eigen::Quaterniond& eiOrientation) const
    {
        // Convert GPSCoordinate to position vector
        eiPosition(0) = stPose.GetGPSCoordinate().dLatitude;
        eiPosition(1) = stPose.GetGPSCoordinate().dLongitude;
        eiPosition(2) = stPose.GetGPSCoordinate().dAltitude;
        // Convert heading to orientation quaternion
        double dHeading = stPose.GetCompassHeading();
        eiOrientation   = Eigen::AngleAxisd(dHeading, Eigen::Vector3d::UnitZ());
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
        // TODO: make sure that math is right (i think it is)

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

    /******************************************************************************
     * @brief This will set the IMU data noise.
     *
     * @param dSigmaAcc - The standard deviation of the acceleration.
     * @param dSigmaGyro - The standard deviation of the gyroscope data.
     * @param dSigmaAccBias - The standard deviation of the acceleration bias.
     * @param dSigmaGyroBias - The standard deviation of the gyroscope bias.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-03
     ******************************************************************************/
    void ExtendedKalmanFilter::SetIMUNoise(double dSigmaAcc, double dSigmaGyro, double dSigmaAccBias, double dSigmaGyroBias)
    {
        // TODO: implement
    }

    /******************************************************************************
     * @brief This will set the GPS data noise.
     *
     * @param dSigmaHor - The standard deviation of the horizontal GPS noise.
     * @param dSigmaVer - The standard deviation of the vertical GPS noise.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-03
     ******************************************************************************/
    void ExtendedKalmanFilter::SetGPSNoise(double dSigmaHor, double dSigmaVer)
    {
        // Clear existing covariance
        m_eiGPSCovariance.setZero();

        // Horizontal noise (X = East/West, Y = North/South)
        m_eiGPSCovariance(0, 0) = dSigmaHor * dSigmaHor;    // variance in X
        m_eiGPSCovariance(1, 1) = dSigmaHor * dSigmaHor;    // variance in Y

        // Vertical noise (Z = Up/Down)
        m_eiGPSCovariance(2, 2) = dSigmaVer * dSigmaVer;    // variance in Z
    }
}    // namespace filters
