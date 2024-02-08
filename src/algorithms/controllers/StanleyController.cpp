/******************************************************************************
 * @brief Implements the StanleyContoller class within the
 *
 * @file StanleyController.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-01
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "StanleyController.h"

/// \cond
// Put implicit includes in here.

#include <cmath>
#include <limits>

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
     * @brief Construct a new Stanley Contoller:: Stanley Contoller object.
     *
     * @param dKp
     * @param dDistToFrontAxle
     * @param path
     *
     * @author clayjay3 (claytonraycowen@gmail.com),  JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-01
     ******************************************************************************/
    StanleyContoller::StanleyContoller(const double dKp, const double dDistToFrontAxle, const double dYawTolerance, const std::vector<geoops::UTMCoordinate> vPathUTM)
    {
        // Initialize member variables
        m_dKp              = dKp;
        m_dDistToFrontAxle = dDistToFrontAxle;
        m_dYawTolerance    = dYawTolerance;
        m_unLastTargetIdx  = -1;
        m_vPathUTM         = vPathUTM;
    }

    /******************************************************************************
     * @brief Destroy the Stanley Contoller:: Stanley Contoller object.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-02-01
     ******************************************************************************/
    StanleyContoller::~StanleyContoller()
    {
        // Nothing to destroy yet.
    }

    /******************************************************************************
     * @brief Calculate the steering control adjustment for an agent using the Stanley method.
     *
     * This function computes the necessary change in yaw (steering angle) to align the agent with the predetermined path.
     * The steering angle is limited by the proportional gain constant to prevent excessively sharp turns.
     *
     * @param utmCurrentPos the agent's current position in the UTM coordinate space.
     * @param dVelocity the agent's current magnitude of velocity.
     * @param dYaw the agent's current yaw angle (heading).
     * @return The calculated change in yaw needed to align the agent with the path.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-03
     ******************************************************************************/
    double StanleyContoller::Calculate(const geoops::UTMCoordinate utmCurrentPos, const double dVelocity, const double dYaw)
    {
        // Find the point on the path closest to the front axle center
        unsigned int unTargetIdx = IdentifyTargetIdx(utmCurrentPos, dYaw);

        // Make sure the agent proceeds forward through the path
        unTargetIdx       = std::max(unTargetIdx, m_unLastTargetIdx);
        m_unLastTargetIdx = unTargetIdx;

        // Calculate the change in yaw needed to go from the target to the next point
        double dTargetYaw = CalculateTargetYaw(unTargetIdx);

        // Calculate the difference between the rover's yaw and the desired path yaw
        // TODO: Verify InputAngleModulus will work in this use case
        double dYawError     = dTargetYaw - dYaw;
        double dYawErrorNorm = numops::InputAngleModulus<double>(dYawError, -180.0, 180.0);

        // Calculate the change in yaw needed to correct for the cross track error
        double dCrossTrackError = CalculateCrossTrackError();
        double dDeltaYaw        = dYawErrorNorm + std::atan2(m_dKp * dCrossTrackError, dVelocity);

        // If a rotation is small enough we will just go ahead and skip it
        if (dYawError < m_dYawTolerance)
            dDeltaYaw = 0;

        return dDeltaYaw;
    }

    /******************************************************************************
     * @brief Identifies the closest point to the center of the agent's front axle on the path.
     *
     * This function scans through a set of path points to find the one that is nearest to the center of the agent's front axle,
     * providing a crucial reference for any path corrections.
     *
     * @param utmCurrentPos the agent's current position in the UTM coordinate space.
     * @param dYaw the agent's current yaw angle (heading).
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-03
     ******************************************************************************/
    unsigned int StanleyContoller::IdentifyTargetIdx(const geoops::UTMCoordinate utmCurrentPos, const double dYaw) const
    {
        // Calculate the position of the center of the front axle in UTM coordinates.
        geoops::UTMCoordinate utmFrontAxlePos = geoops::UTMCoordinate(utmCurrentPos);
        utmFrontAxlePos.dEasting              = utmCurrentPos.dEasting + this->m_dDistToFrontAxle * std::cos(dYaw);
        utmFrontAxlePos.dNorthing             = utmCurrentPos.dNorthing + this->m_dDistToFrontAxle * std::cos(dYaw);

        // Search for the nearest point in the path
        unsigned int unTargetIdx                                  = -1;
        double dMinDistance                                       = std::numeric_limits<double>::max();
        std::vector<geoops::UTMCoordinate>::const_iterator itPath = this->m_vPathUTM.begin();
        while (itPath != this->m_vPathUTM.end())
        {
            // Find the distance in meters between the center of the front axle and a point on the path.
            geoops::GeoMeasurement gmFrontAxleToPathPoint = geoops::CalculateGeoMeasurement(utmFrontAxlePos, *itPath);

            // Update the closest point to the front axle's center if the current distance is the shortest recorded.
            // Save both the point's index and the distance.
            if (gmFrontAxleToPathPoint.dDistanceMeters < dMinDistance)
            {
                unTargetIdx  = std::distance(this->m_vPathUTM.begin(), itPath);
                dMinDistance = gmFrontAxleToPathPoint.dDistanceMeters;
            }

            // Move the iterator to the next point in the path
            ++itPath;
        }

        return unTargetIdx;
    }

    /******************************************************************************
     * @brief
     *
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-03
     ******************************************************************************/
    double StanleyContoller::CalculateTargetYaw(unsigned int unTargetIdx) const
    {
        // TODO: Implement function
    }
}    // namespace controllers
