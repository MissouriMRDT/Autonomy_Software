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
        // Initialize member variables
        // TODO: ugly math sad face
    }

    /******************************************************************************
     * @brief Construct a new Extended Kalman Filter:: Extended Kalman Filter object.
     *
     *@param stInitPose - The initial GPS and heading of the rover.
     *@param eiInitVel - The initial velocity of the rover. //TODO: maybe change to accelerations?
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
                                               const Eigen::Vector3d& eiInitVel,
                                               double dSigmaAcc,
                                               double dSigmaGyro,
                                               double dSigmaAccBias,
                                               double dSigmaGyroBias,
                                               double dSigmaGPSHor,
                                               double dSigmaGPSVer,
                                               double dSigmaYaw)
    {
        // Initialize member variables
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
    woid ExtendedKalmanFilter::ToRoverPose(const Eigen::Vector3d& eiPosition, const Eigen::Quaterniond& eiOrientation) const
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
}    // namespace filters
