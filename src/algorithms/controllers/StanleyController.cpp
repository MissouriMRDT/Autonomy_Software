/******************************************************************************
 * @brief Implements the StanleyController class within the
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
     * @param vPathUTM - Vector of UTM coordinates describing path to follow.
     * @param dKp - Proportional gain.
     * @param dDistToFrontAxle - Distance between the position sensor (GPS) and the front axle.
     * @param dYawTolerance - Minimum yaw change threshold for execution.
     *
     * @note The higher the proportional gain the more reactive the rover will be to changes in yaw.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-03
     ******************************************************************************/
    StanleyController::StanleyController(const std::vector<geoops::UTMCoordinate>& vPathUTM, const double dKp, const double dDistToFrontAxle, const double dYawTolerance)
    {
        // Initialize member variables
        m_dKp              = dKp;
        m_dDistToFrontAxle = dDistToFrontAxle;
        m_dYawTolerance    = dYawTolerance;
        m_unLastTargetIdx  = 0;
        m_vPathUTM         = vPathUTM;
    }

    /******************************************************************************
     * @brief Destroy the Stanley Contoller:: Stanley Contoller object.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-02-01
     ******************************************************************************/
    StanleyController::~StanleyController()
    {
        // Nothing to destroy yet.
    }

    /******************************************************************************
     * @brief Calculate the steering control adjustment for an agent using the Stanley method.
     *
     * This function computes the necessary change in yaw (steering angle) to align the agent with the predetermined path.
     * The steering angle is limited by the proportional gain constant to prevent excessively sharp turns.
     *
     * @param stCurrPosUTM - The agent's current position in the UTM coordinate space.
     * @param dVelocity - The agent's current magnitude of velocity.
     * @param dBearing - The agent's current yaw angle (heading).
     * @return double - The calculated change in yaw needed to align the agent with the path.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-03
     ******************************************************************************/
    double StanleyController::Calculate(const geoops::UTMCoordinate& stCurrPosUTM, const double dVelocity, const double dBearing)
    {
        // Verify the given bearing is within 0-360 degrees.
        if (dBearing < 0 || dBearing > 360)
            LOG_ERROR(logging::g_qSharedLogger, "StanleyController::Calculate bearing must be in the interval [0-360].");

        // Calculate the position for the center of the front axle.
        geoops::UTMCoordinate stFrontAxlePos = CalculateFrontAxleCoordinate(stCurrPosUTM, dBearing);

        // Find the point on the path closest to the front axle center
        unsigned int unTargetIdx = IdentifyTargetIdx(stFrontAxlePos, dBearing);

        // Make sure the agent proceeds forward through the path
        unTargetIdx       = std::max(unTargetIdx, m_unLastTargetIdx);
        m_unLastTargetIdx = unTargetIdx;

        // Calculate the change in yaw needed to go from the target to the next point
        double dTargetYaw = CalculateTargetBearing(unTargetIdx);

        // Calculate the difference between the rover's yaw and the desired path yaw
        double dYawError = numops::InputAngleModulus<double>(dTargetYaw - dBearing, -180.0, 180.0);

        // Calculate the change in yaw needed to correct for the cross track error
        double dCrossTrackError = CalculateCrossTrackError(stFrontAxlePos, unTargetIdx, dBearing);
        double dDeltaYaw        = dYawError + std::atan2(m_dKp * dCrossTrackError, dVelocity);

        // If a rotation is small enough we will just go ahead and skip it
        if (std::abs(dDeltaYaw) < m_dYawTolerance)
            dDeltaYaw = 0;

        // Here we translate the relative change in yaw to an absolute heading
        double dNewBearing = numops::InputAngleModulus<double>(dBearing + dDeltaYaw, 0, 360);
        return dNewBearing;
    }

    /******************************************************************************
     * @brief Setter for proportional gain.
     *
     * @param dKp -  Proportional gain.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-16
     ******************************************************************************/
    void StanleyController::SetProportionalGain(const double dKp)
    {
        m_dKp = dKp;
    }

    /******************************************************************************
     * @brief Setter for distance to front axle.
     *
     * @param dDistToFrontAxle - Distance between the position sensor (GPS) and the front axle.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-16
     ******************************************************************************/
    void StanleyController::SetDistanceToFrontAxle(const double dDistToFrontAxle)
    {
        m_dDistToFrontAxle = dDistToFrontAxle;
    }

    /******************************************************************************
     * @brief Setter for yaw tolerance.
     *
     * @param dYawTolerance - Minimum yaw change threshold for execution.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-16
     ******************************************************************************/
    void StanleyController::SetYawTolerance(const double dYawTolerance)
    {
        m_dYawTolerance = dYawTolerance;
    }

    /******************************************************************************
     * @brief Setter for path.
     *
     * @param vPathUTM -  Vector of UTM coordinates describing path to follow.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-16
     ******************************************************************************/
    void StanleyController::SetPathUTM(std::vector<geoops::UTMCoordinate>& vPathUTM)
    {
        m_vPathUTM        = vPathUTM;
        m_unLastTargetIdx = 0;
    }

    /******************************************************************************
     * @brief Getter for proportional gain.
     *
     * @return double - Proportional gain.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-16
     ******************************************************************************/
    double StanleyController::GetProportionalGain() const
    {
        return m_dKp;
    }

    /******************************************************************************
     * @brief Getter for distance to the front axle.
     *
     * @return double - Distance between the position sensor (GPS) and the front axle.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-16
     ******************************************************************************/
    double StanleyController::GetDistanceToFrontAxle() const
    {
        return m_dDistToFrontAxle;
    }

    /******************************************************************************
     * @brief Getter for yaw tolerance.
     *
     * @return double - Minimum yaw change threshold for execution.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-16
     ******************************************************************************/
    double StanleyController::GetYawTolerance() const
    {
        return m_dYawTolerance;
    }

    /******************************************************************************
     * @brief Getter for the index of the last target point on the path.
     *
     * @return unsigned int -  Index of the last target point on the path.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-16
     ******************************************************************************/
    unsigned int StanleyController::GetLastTargetIdx() const
    {
        return m_unLastTargetIdx;
    }

    /******************************************************************************
     * @brief Getter for path.
     *
     * @return std::vector<geoops::UTMCoordinate> - Sequence of UTM coordinates defining the navigational path.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-16
     ******************************************************************************/
    std::vector<geoops::UTMCoordinate> StanleyController::GetPathUTM() const
    {
        return m_vPathUTM;
    }

    /******************************************************************************
     * @brief Calculate the UTM coordinate of the center of the agent's front axle.
     *
     * @param stCurrPosUTM - The agent's current position in the UTM coordinate space.
     * @param dBearing - The current bearing of the agent, measured in degrees from 0 to 360.
     * @return UTMCoordinate - The UTM coordinate of the center of the agent's front axle.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-08
     ******************************************************************************/
    geoops::UTMCoordinate StanleyController::CalculateFrontAxleCoordinate(const geoops::UTMCoordinate& stCurrPosUTM, const double dBearing) const
    {
        // Verify the given bearing is within 0-360 degrees.
        if (dBearing < 0 || dBearing > 360)
            LOG_ERROR(logging::g_qSharedLogger, "StanleyController::CalculateFrontAxleCoordinate bearing must be in the interval [0-360].");

        // Convert the bearing to a change in degrees of yaw relative to the north axis
        // Here a positive degree represents a change in yaw towards the East.
        // Here a negative degree represents a change in yaw towards the West.
        double dChangeInYawRelToNorth = dBearing <= 180 ? dBearing : dBearing - 360;

        // Convert to radians.
        dChangeInYawRelToNorth = (dChangeInYawRelToNorth / 180) * M_PI;

        // Calculate the unit vector for the agent's orientation in terms of East and North.
        double dOrientEast  = std::sin(dChangeInYawRelToNorth);
        double dOrientNorth = std::cos(dChangeInYawRelToNorth);

        // Calculate the UTM coordinate for the center of the agent's front axle.
        geoops::UTMCoordinate stFrontAxlePosUTM = geoops::UTMCoordinate(stCurrPosUTM);
        stFrontAxlePosUTM.dEasting              = stCurrPosUTM.dEasting + m_dDistToFrontAxle * dOrientEast;
        stFrontAxlePosUTM.dNorthing             = stCurrPosUTM.dNorthing + m_dDistToFrontAxle * dOrientNorth;

        // Return the UTM coordinate of the center of the agent's front axle.
        return stFrontAxlePosUTM;
    }

    /******************************************************************************
     * @brief Identifies the closest point to the center of the agent's front axle on the path.
     *
     * This function scans through a set of path points to find the one that is nearest to the center of the agent's front axle,
     * providing a crucial reference for any path corrections.
     *
     * @param utmFrontAxlePos - The UTM coordinate of the center of the agent's front axle.
     * @param dBearing - The current bearing of the agent, measured in degrees from 0 to 360.
     * @return unsigned int -  Index of the target point on the path.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-03
     ******************************************************************************/
    unsigned int StanleyController::IdentifyTargetIdx(const geoops::UTMCoordinate& stmFrontAxlePosUTM, const double dBearing) const
    {
        // Verify the given bearing is within 0-360 degrees.
        if (dBearing < 0 || dBearing > 360)
            LOG_ERROR(logging::g_qSharedLogger, "StanleyController::IdentifyTargetIdx bearing must be in the interval [0-360].");

        // Search for the nearest point in the path
        unsigned int unTargetIdx;
        double dMinDistance                                       = std::numeric_limits<double>::max();
        std::vector<geoops::UTMCoordinate>::const_iterator itPath = this->m_vPathUTM.begin();
        while (itPath != this->m_vPathUTM.end())
        {
            // Find the distance in meters between the center of the front axle and a point on the path.
            double dDistanceFromPoint = std::hypot(stmFrontAxlePosUTM.dNorthing - itPath->dNorthing, stmFrontAxlePosUTM.dEasting - itPath->dEasting);

            // Update the closest point to the front axle's center if the current distance is the shortest recorded.
            // Save both the point's index and the distance.
            if (dDistanceFromPoint < dMinDistance)
            {
                unTargetIdx  = std::distance(this->m_vPathUTM.begin(), itPath);
                dMinDistance = dDistanceFromPoint;
            }

            // Move the iterator to the next point in the path
            ++itPath;
        }

        return unTargetIdx;
    }

    /******************************************************************************
     * @brief Calculate the required bearing to navigate from the current target point to the subsequent point on the path.
     *
     * @param unTargetIdx - Index of the target point on the path.
     * @return double - Bearing required to get from the target point to the subsequent point on the path.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-03
     ******************************************************************************/
    double StanleyController::CalculateTargetBearing(const unsigned int unTargetIdx) const
    {
        // Verify the target index is a valid point on the path.
        if (unTargetIdx >= m_vPathUTM.size())
            LOG_ERROR(logging::g_qSharedLogger, "StanleyController::CalculateTargetBearing target does not exist.");

        // The yaw is calculated by finding the bearing needed to navigate from the
        // target point to the next point in the path.
        geoops::UTMCoordinate stTargetPointUTM = m_vPathUTM[unTargetIdx];
        geoops::UTMCoordinate stNextPointUTM   = m_vPathUTM[unTargetIdx + 1];

        // Calculate the displacement from the target point to the next point in the path.
        double dDisplacementEast  = stNextPointUTM.dEasting - stTargetPointUTM.dEasting;
        double dDisplacementNorth = stNextPointUTM.dNorthing - stTargetPointUTM.dNorthing;

        // Calculate the magnitude of the displacement.
        double dDisplacementMagnitude = std::hypot(dDisplacementEast, dDisplacementNorth);

        // Calculate the bearing in degrees required to navigate to the next point on the path.
        dDisplacementNorth /= dDisplacementMagnitude;
        double dBearing = (std::acos(dDisplacementNorth) / M_PI) * 180;
        if (dDisplacementEast < 0)
            dBearing = 360 - dBearing;

        // Return the relative bearing needed to get from the target point to the next point in the path.
        return dBearing;
    }

    /******************************************************************************
     * @brief Calculate the cross track error. This error expresses how far off the agent is from the path (lateral distance).
     *
     * Here is what causes a high magnitude for cross track error.
     * 1. The agent is far from the target point (a large displacement from the path).
     * 2. The agent is driving parallel to the path.
     * The reason for number 2 is if the agent is far from the path and driving parallel it will
     * continue to stay off course. However, if the agent is on the path and driving parallel the
     * displacement vector will be negligible and the error will likewise be negligible.
     *
     * Note that the significance of the error is its magnitude. Positive and negative are the same amount of error just in opposing
     * directions.
     *
     * @param utmFrontAxlePos - The UTM coordinate of the center of the agent's front axle.
     * @param unTargetIdx - Index of the target point on the path.
     * @param dBearing - The current bearing of the agent, measured in degrees from 0 to 360.
     * @return double - The cross track error (CTE).
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-02-09
     ******************************************************************************/
    double StanleyController::CalculateCrossTrackError(const geoops::UTMCoordinate& stFrontAxlePosUTM, const unsigned int unTargetIdx, const double dBearing) const
    {
        // Verify the given bearing is within 0-360 degrees.
        if (dBearing < 0 || dBearing > 360)
            LOG_ERROR(logging::g_qSharedLogger, "StanleyController::CalculateCrossTrackError bearing must be in the interval [0-360].");

        // Convert the bearing to a change in degrees of yaw relative to the north axis
        // Here a positive degree represents a change in yaw towards the East.
        // Here a negative degree represents a change in yaw towards the West.
        double dChangeInYawRelToNorth = dBearing <= 180 ? dBearing : dBearing - 360;
        dChangeInYawRelToNorth        = (dChangeInYawRelToNorth / 180) * M_PI;
        dChangeInYawRelToNorth -= M_PI / 2;

        // Calculate the front axle vector.
        // This vector will point perpendicular to the orientation of the agent (representing the front drive axle).
        double dFrontDriveAxleX = std::sin(dChangeInYawRelToNorth);
        double dFrontDriveAxleY = std::cos(dChangeInYawRelToNorth);

        // Find the displacement between the target and the center of the front drive axle.
        geoops::UTMCoordinate stTargetPosUTM = m_vPathUTM[unTargetIdx];
        double dDisplacementX                = stFrontAxlePosUTM.dEasting - stTargetPosUTM.dEasting;
        double dDisplacementY                = stFrontAxlePosUTM.dNorthing - stTargetPosUTM.dNorthing;

        // Calculate the cross track error as a dot product of our front drive axle and the displacement vector.
        return (dDisplacementX * dFrontDriveAxleX) + (dDisplacementY * dFrontDriveAxleY);
    }

}    // namespace controllers
