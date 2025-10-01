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
     *@param initPos - The initial position of the rover.
     *@param initVel - The initial velocity of the rover. //TODO: see if this is a feasible input
     *@param initOrien - The initial orientation of the rover.
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
    ExtendedKalmanFilter::ExtendedKalmanFilter(const Eigen::Vector3d& initPos,
                                               const Eigen::Vector3d& initVel,
                                               const Eigen::Quaterniond& initOrien,
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
}    // namespace filters
