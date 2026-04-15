/******************************************************************************
 * @brief Implements the Pure Pursuit Controller class.
 *
 * @file PurePursuitController.cpp
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2026-03-15
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#include "PurePursuitController.h"
#include "../../AutonomyConstants.h"

/// \cond
#include <algorithm>
#include <cmath>

/// \endcond

/******************************************************************************
 * @brief This namespace stores classes, functions, and structs that are used to
 * implement different controllers that implement advanced control systems
 * used for accurate and precise robotic control.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-01
 ******************************************************************************/
namespace controllers
{
    /******************************************************************************
     * @brief Construct a new Pure Pursuit Controller:: Pure Pursuit Controller object.
     *
     * @param dLookaheadDistance - The number of waypoints the rover will look ahead.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-03-15
     ******************************************************************************/
    PurePursuitController::PurePursuitController(const double dLookaheadDistance)
    {
        // Initialize member variables.
        m_dLookaheadDistance               = dLookaheadDistance;
        m_nCurrentReferencePathTargetIndex = 0;
    }

    /******************************************************************************
     * @brief Calculate an updated drive vector for the rover based on the current pose
     * using the pure pursuit control law.
     *
     * @param stCurrentPose - The current pose of the rover.
     * @param dMaxSpeed - The maximum speed the rover can travel.
     * @return DriveVector - The new output heading and speed for the rover.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-03-15
     ******************************************************************************/
    PurePursuitController::DriveVector PurePursuitController::Calculate(const geoops::RoverPose& stCurrentPose, const double dMaxSpeed)
    {
        // Check if the reference path is empty.
        if (m_vReferencePath.empty())
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "PurePursuitController::Calculate: Reference path is empty. Cannot calculate drive powers.");

            return DriveVector{0.0, 0.0};
        }

        // Evaluate the rover position to properly advance the path index based on physical location.
        m_nCurrentReferencePathTargetIndex = FindClosestWaypointIndex(stCurrentPose.GetUTMCoordinate());

        // Check if we are at the end of the path.
        if (m_nCurrentReferencePathTargetIndex >= static_cast<int>(m_vReferencePath.size()) - 2)
        {
            // Get the start and end points of the last segment.
            geoops::UTMCoordinate stLastPoint         = m_vReferencePath.back().GetUTMCoordinate();
            geoops::UTMCoordinate stSecondToLastPoint = m_vReferencePath[m_vReferencePath.size() - 2].GetUTMCoordinate();
            geoops::UTMCoordinate stRoverPos          = stCurrentPose.GetUTMCoordinate();

            // Calculate vector of the last segment (Start -> End).
            double dSegmentX     = stLastPoint.dEasting - stSecondToLastPoint.dEasting;
            double dSegmentY     = stLastPoint.dNorthing - stSecondToLastPoint.dNorthing;
            double dSegmentLenSq = dSegmentX * dSegmentX + dSegmentY * dSegmentY;

            // Calculate vector from Segment Start -> Rover.
            double dRoverVectorX = stRoverPos.dEasting - stSecondToLastPoint.dEasting;
            double dRoverVectorY = stRoverPos.dNorthing - stSecondToLastPoint.dNorthing;

            // Project rover onto the segment vector.
            double dNormalDistance = 0.0;
            if (dSegmentLenSq > 1e-6)
            {
                dNormalDistance = (dRoverVectorX * dSegmentX + dRoverVectorY * dSegmentY) / dSegmentLenSq;
            }

            // Check if we have passed the end or are very close to it.
            if (dNormalDistance >= 1.0)
            {
                // We have reached or passed the end. Drive straight towards the final waypoint.
                double dHeadingToLastWaypoint = geoops::CalculateGeoMeasurement(stRoverPos, stLastPoint).dStartRelativeBearing;
                return DriveVector{dHeadingToLastWaypoint, dMaxSpeed};
            }
        }

        // Find the lookahead point on the path ahead of the rover.
        geoops::Waypoint stLookaheadWaypoint = FindLookaheadWaypoint(stCurrentPose.GetUTMCoordinate());

        // Calculate the required heading to reach the lookahead point.
        double dDeltaX = stLookaheadWaypoint.GetUTMCoordinate().dEasting - stCurrentPose.GetUTMCoordinate().dEasting;
        double dDeltaY = stLookaheadWaypoint.GetUTMCoordinate().dNorthing - stCurrentPose.GetUTMCoordinate().dNorthing;

        // Math angle of segment (atan2 gives angle from +X (East), CCW positive) in degrees.
        double dTargetMathDeg = std::atan2(dDeltaY, dDeltaX) * (180.0 / M_PI);

        // Convert math angle to COMPASS heading (0 = North, clockwise positive): compass = (90 - math) mod 360.
        double dAbsoluteHeadingGoal = std::fmod(90.0 - dTargetMathDeg + 360.0, 360.0);

        return DriveVector{dAbsoluteHeadingGoal, dMaxSpeed};
    }

    /******************************************************************************
     * @brief Set the Reference Path object
     *
     * @param vReferencePath - The new reference path for the controller to follow.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-03-15
     ******************************************************************************/
    void PurePursuitController::SetReferencePath(const std::vector<geoops::Waypoint>& vReferencePath)
    {
        // Reset the index and store the new path.
        m_nCurrentReferencePathTargetIndex = 0;
        m_vReferencePath                   = vReferencePath;
    }

    /******************************************************************************
     * @brief Set the Lookahead Distance object
     *
     * @param dLookaheadDistance - The distance ahead of the rover to place the target point.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-03-15
     ******************************************************************************/
    void PurePursuitController::SetLookaheadDistance(const double dLookaheadDistance)
    {
        m_dLookaheadDistance = dLookaheadDistance;
    }

    /******************************************************************************
     * @brief This will allow us to set the max amount of indices to lookahead.
     *
     * @param nLookAheadIndex - How many waypoints/indices we can look ahead.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-04-14
     ******************************************************************************/
    void PurePursuitController::SetLookaheadIndex(const int nLookAheadIndex)
    {
        m_nLookaheadIndex = nLookAheadIndex;
    }

    /******************************************************************************
     * @brief Get the Reference Path object
     *
     * @return std::vector<geoops::Waypoint> - The current reference path.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-03-15
     ******************************************************************************/
    std::vector<geoops::Waypoint> PurePursuitController::GetReferencePath() const
    {
        return m_vReferencePath;
    }

    /******************************************************************************
     * @brief Get the Lookahead Distance object
     *
     * @return double - The current lookahead distance.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-03-15
     ******************************************************************************/
    double PurePursuitController::GetLookaheadDistance() const
    {
        return m_dLookaheadDistance;
    }

    /******************************************************************************
     * @brief Get the Reference Path Target Index object
     *
     * @return int - The current index of the reference path.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-03-15
     ******************************************************************************/
    int PurePursuitController::GetReferencePathTargetIndex() const
    {
        return m_nCurrentReferencePathTargetIndex;
    }

    /******************************************************************************
     * @brief Finds the closest waypoint index in the path to the current position,
     * enforcing topological blinders to prevent jumping across overlapping paths.
     *
     * @param stCurrentPosition - The current UTM coordinate of the rover.
     * @return int - The index of the closest waypoint.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-03-15
     ******************************************************************************/
    int PurePursuitController::FindClosestWaypointIndex(const geoops::UTMCoordinate& stCurrentPosition)
    {
        // Create instance variables.
        double dClosestDistanceSq = std::numeric_limits<double>::max();
        const size_t siWaypoints  = m_vReferencePath.size();
        int nBestSegmentIndex     = m_nCurrentReferencePathTargetIndex;

        // Check bounds
        if (siWaypoints == 0)
        {
            return 0;
        }

        if (siWaypoints == 1)
        {
            return 0;
        }

        // Step through path to find next carrot on a stick point.
        for (size_t siIter = static_cast<size_t>(m_nCurrentReferencePathTargetIndex); siIter < siWaypoints - 1; ++siIter)
        {
            const geoops::UTMCoordinate& stA = m_vReferencePath[siIter].GetUTMCoordinate();

            double dx                        = stCurrentPosition.dEasting - stA.dEasting;
            double dy                        = stCurrentPosition.dNorthing - stA.dNorthing;
            double dDistSq                   = dx * dx + dy * dy;

            if (dDistSq < dClosestDistanceSq)
            {
                dClosestDistanceSq = dDistSq;
                nBestSegmentIndex  = static_cast<int>(siIter);
            }
        }

        return nBestSegmentIndex;
    }

    /******************************************************************************
     * @brief Searches forward from the current target index to find a point that is
     * at least the lookahead distance away from the rover.
     *
     * @param stCurrentPosition - The current UTM coordinate of the rover.
     * @return geoops::Waypoint - The target waypoint to drive towards.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-03-15
     ******************************************************************************/
    geoops::Waypoint PurePursuitController::FindLookaheadWaypoint(const geoops::UTMCoordinate& stCurrentPosition)
    {
        const size_t siWaypoints = m_vReferencePath.size();

        // Search forward from the current closest point to find the carrot.
        // Limit the search so it tracks strictly along the path and does not jump rings.
        size_t nSearchLimit = std::min(siWaypoints, static_cast<size_t>(m_nCurrentReferencePathTargetIndex + m_nLookaheadIndex));

        for (size_t siIter = m_nCurrentReferencePathTargetIndex; siIter < nSearchLimit; ++siIter)
        {
            const geoops::UTMCoordinate& stTarget = m_vReferencePath[siIter].GetUTMCoordinate();

            double dDeltaX                        = stTarget.dEasting - stCurrentPosition.dEasting;
            double dDeltaY                        = stTarget.dNorthing - stCurrentPosition.dNorthing;
            double dDistance                      = std::hypot(dDeltaX, dDeltaY);

            // Once we find a point that sits on or outside our lookahead radius, return it.
            if (dDistance >= m_dLookaheadDistance)
            {
                return m_vReferencePath[siIter];
            }
        }

        // If no point is far enough ahead, return the last point checked.
        return m_vReferencePath[nSearchLimit - 1];
    }
}    // namespace controllers
