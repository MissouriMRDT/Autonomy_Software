/******************************************************************************
 * @brief Implementations the Predictive Stanley Controller class.
 *
 * @file PredictiveStanleyController.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-10
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/
#include "PredictiveStanleyController.h"
#include "../../AutonomyConstants.h"
#include "../../AutonomyGlobals.h"
#include "../../AutonomyNetworking.h"
#include "../../util/GeospatialOperations.hpp"
#include "../../util/planners/PathPostProcessing.hpp"

/// \cond

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
     * @brief Construct a new Predictive Stanley Controller:: Predictive Stanley Controller object.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    PredictiveStanleyController::PredictiveStanleyController()
    {
        // Initialize member variables.
        m_dControlGain                     = constants::STANLEY_CROSSTRACK_CONTROL_GAIN;
        m_dAngularVelocityLimit            = constants::STANLEY_ANGULAR_VELOCITY_LIMIT;
        m_nPredictionHorizon               = constants::STANLEY_PREDICTION_HORIZON;
        m_dPredictionTimeStep              = constants::STANLEY_PREDICTION_TIME_STEP;
        m_nCurrentReferencePathTargetIndex = 0;
        m_UnicycleModel                    = UnicycleModel();
    }

    /******************************************************************************
     * @brief Construct a new Predictive Stanley Controller:: Predictive Stanley Controller object.
     *
     * @param dControlGain - The control gain for the controller.
     * @param dSteeringAngleLimit - The maximum steering angle the rover can turn.
     * @param nPredictionHorizon - The number of predictions to make.
     * @param dPredictionTimeStep - The time step to predict the future state. How far into the future to predict.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    PredictiveStanleyController::PredictiveStanleyController(const double dControlGain,
                                                             const double dAngularVelocityLimit,
                                                             const int nPredictionHorizon,
                                                             const double dPredictionTimeStep)
    {
        // Initialize member variables.
        m_dControlGain                     = dControlGain;
        m_dAngularVelocityLimit            = dAngularVelocityLimit;
        m_nPredictionHorizon               = nPredictionHorizon;
        m_dPredictionTimeStep              = dPredictionTimeStep;
        m_nCurrentReferencePathTargetIndex = 0;
        m_UnicycleModel                    = UnicycleModel(0.0, 0.0, 0.0);
    }

    /******************************************************************************
     * @brief Destroy the Predictive Stanley Controller:: Predictive Stanley Controller object.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    PredictiveStanleyController::~PredictiveStanleyController()
    {
        // Nothing to do yet.
    }

    /******************************************************************************
     * @brief Calculate an updated steering angle for the rover based on the current pose
     *      using the predictive stanley controller.
     *
     * @param stCurrentPose - The current pose of the rover.
     * @param dMaxSpeed - The maximum speed the rover can travel.
     * @return double - The new output steering angle for the rover.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    PredictiveStanleyController::DriveVector PredictiveStanleyController::Calculate(const geoops::RoverPose& stCurrentPose, const double dMaxSpeed)
    {
        // Create instance variables.
        double dSteeringAngle = 0.0;
        std::vector<UnicycleModel::Prediction> vPredictions;

        // Check if the reference path is empty.
        if (m_vReferencePath.empty())
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "PredictiveStanleyController::Calculate: Reference path is empty. Cannot calculate drive powers.");

            return DriveVector{0.0, 0.0};
        }

        // Check if we are at the end of the path. Normally stanley would continue driving in the last direction of the calculated path
        // headings, but we want to make sure we get to the end point, so we'll just drive straight to it once at the end of the path.
        if (m_nCurrentReferencePathTargetIndex >= static_cast<int>(m_vReferencePath.size()) - 1)
        {
            // Get the last point in the path.
            geoops::Waypoint stLastWaypoint = m_vReferencePath.back();
            // Calculate the heading to the last point.
            double dHeadingToLastWaypoint = geoops::CalculateGeoMeasurement(stCurrentPose.GetUTMCoordinate(), stLastWaypoint.GetUTMCoordinate()).dStartRelativeBearing;

            return DriveVector{dHeadingToLastWaypoint, 1.0};
        }

        // Update the unicycle model with the current state.
        m_UnicycleModel.UpdateState(stCurrentPose.GetUTMCoordinate().dEasting, stCurrentPose.GetUTMCoordinate().dNorthing, stCurrentPose.GetCompassHeading());
        // Predict the future state of the model.
        m_UnicycleModel.Predict(m_dPredictionTimeStep, m_nPredictionHorizon, vPredictions);

        // Loop through all the predicted future states to compute the steering angle.
        for (size_t nIter = 0; nIter < vPredictions.size(); ++nIter)
        {
            // Create instance variables.
            double dPredictedXPosition = vPredictions[nIter].dXPosition;
            double dPredictedYPosition = vPredictions[nIter].dYPosition;
            double dPredictedTheta     = vPredictions[nIter].dTheta;

            // Create a UTM coordinate for the predicted position.
            geoops::UTMCoordinate stPredictedPosition = stCurrentPose.GetUTMCoordinate();
            stPredictedPosition.dEasting              = dPredictedXPosition;
            stPredictedPosition.dNorthing             = dPredictedYPosition;
            // Find the closest point to the reference path.
            geoops::Waypoint stClosestWaypoint = FindClosestWaypointInPath(stPredictedPosition);

            // Compute the path forward vector from the segment start -> next waypoint (use index set by FindClosestWaypointInPath).
            int nIdx                                    = m_nCurrentReferencePathTargetIndex;
            int nNextIdx                                = std::min(nIdx + 1, static_cast<int>(m_vReferencePath.size() - 1));
            const geoops::UTMCoordinate& stSegmentStart = m_vReferencePath[nIdx].GetUTMCoordinate();
            const geoops::UTMCoordinate& stSegmentEnd   = m_vReferencePath[nNextIdx].GetUTMCoordinate();

            // Forward vector of the path segment.
            double dForwardVectorX = stSegmentEnd.dEasting - stSegmentStart.dEasting;
            double dForwardVectorY = stSegmentEnd.dNorthing - stSegmentStart.dNorthing;
            double dForwardNorm    = std::hypot(dForwardVectorX, dForwardVectorY);
            // Degenerate segment. Skip this prediction step.
            if (dForwardNorm < 1e-9)
            {
                continue;
            }

            // Unit forward vector.
            double dFwdUnitX = dForwardVectorX / dForwardNorm;
            double dFwdUnitY = dForwardVectorY / dForwardNorm;
            // Math angle of segment (atan2 gives angle from +X (East), CCW positive) in degrees.
            double dSegmentMathDeg = std::atan2(dForwardVectorY, dForwardVectorX) * (180.0 / M_PI);
            // Convert math angle to COMPASS heading (0 = North, clockwise positive):
            // compass = (90 - math) mod 360.
            double dPathHeadingDeg = std::fmod(90.0 - dSegmentMathDeg + 360.0, 360.0);
            // Predicted theta is produced by UnicycleModel in DEGREES (compass). Treat as degrees.
            double dPredThetaDeg = dPredictedTheta;
            // Small helper: signed smallest-angle difference in degrees in (-180, 180].
            std::function<double(double, double)> AngleDiffDeg = [](double targetDeg, double sourceDeg) -> double
            {
                double diff = std::fmod(targetDeg - sourceDeg + 540.0, 360.0) - 180.0;
                return diff;
            };
            // Heading error (degrees): positive means we should rotate clockwise to match target (compass conv).
            double dHeadingError = AngleDiffDeg(dPathHeadingDeg, dPredThetaDeg);

            // Vehicle vector relative to the closest point. (projection)
            double dVehicleVectorX = dPredictedXPosition - stClosestWaypoint.GetUTMCoordinate().dEasting;
            double dVehicleVectorY = dPredictedYPosition - stClosestWaypoint.GetUTMCoordinate().dNorthing;
            // Longitudinal projection and lateral vector.
            double dLongitudinal    = dVehicleVectorX * dFwdUnitX + dVehicleVectorY * dFwdUnitY;
            double dLateralX        = dVehicleVectorX - dLongitudinal * dFwdUnitX;
            double dLateralY        = dVehicleVectorY - dLongitudinal * dFwdUnitY;
            double dLateralDistance = std::hypot(dLateralX, dLateralY);
            // Sign using cross product. (left positive)
            double dCross           = dFwdUnitX * dVehicleVectorY - dFwdUnitY * dVehicleVectorX;
            double dCrossTrackError = (dCross > 0.0 ? 1.0 : -1.0) * dLateralDistance;    // meters

            // Apply an exponential weight factor that decreases as we predict further into the future.
            double dTimeWeight = std::exp(-1.5 * static_cast<double>(nIter));

            // Use model velocity if available, otherwise fall back to provided dMaxSpeed
            double dModelSpeed = m_UnicycleModel.GetVelocity();
            double dSpeed      = (dModelSpeed > 0.0) ? dModelSpeed : dMaxSpeed;
            dSpeed             = std::max(dSpeed, 1e-4);

            // Stanley control law to compute the steering angle.
            double dStanleyTermRad = std::atan2(m_dControlGain * dCrossTrackError, dSpeed);
            double dStanleyTermDeg = dStanleyTermRad * (180.0 / M_PI);
            // Combine the heading error and the stanley term.
            dSteeringAngle += (dStanleyTermDeg + dHeadingError) * dTimeWeight;

            // Limit the steering angle to the given limit.
            dSteeringAngle = std::clamp(dSteeringAngle, -m_dAngularVelocityLimit, m_dAngularVelocityLimit);
        }

        // The new steering heading must be from 0-360 degrees.
        double dAbsoluteHeadingGoal = numops::InputAngleModulus(stCurrentPose.GetCompassHeading() + dSteeringAngle, 0.0, 360.0);

        // Calculate ETA
        double remainingDistance =
            geoops::CalculateGeoMeasurement(stCurrentPose.GetUTMCoordinate(), m_vReferencePath[m_nCurrentReferencePathTargetIndex]).dDistanceMeters;
        for (int i = m_nCurrentReferencePathTargetIndex; i < m_vReferencePath.size() - 1; i++)
        {
            remainingDistance += geoops::CalculateGeoMeasurement(m_vReferencePath[i], m_vReferencePath[i + 1]).dDistanceMeters;
        }
        double timeRemaining = remainingDistance / globals::g_pNavigationBoard->GetVelocity();

        // Initialize packet
        rovecomm::RoveCommPacket<double> stPacket;
        stPacket.unDataId  = 11105;
        stPacket.eDataType = manifest::DataTypes::DOUBLE_T;
        stPacket.unDataCount = 1;
        stPacket.vData.emplace_back(timeRemaining);
        
        // Send time remaining over RoveComm to Basestation
        if (network::g_pRoveCommUDPNode)
        {
            // Send packet on local machine (This needs to be changed to actual Basestation IP)
            network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, "192.168.0.117", 9000);

            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Sent waypoint: ()");
        }

        return DriveVector{dAbsoluteHeadingGoal, dMaxSpeed};
    }

    /******************************************************************************
     * @brief Sets the path that the controller will follow.
     *
     * @param vReferencePath - A reference to a vector of waypoints that make up the path.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    void PredictiveStanleyController::SetReferencePath(const std::vector<geoops::Waypoint>& vReferencePath)
    {
        // Reset the current target index.
        m_nCurrentReferencePathTargetIndex = 0;
        // Reset the bicycle model.
        m_UnicycleModel.ResetState();

        // Smooth the path by fitting it to a B-spline.
        std::vector<geoops::Waypoint> vSmoothedPath = pathplanners::postprocessing::FitPathWithBSpline(vReferencePath);

        // Set the reference path.
        m_vReferencePath = vSmoothedPath;

        // Initialize packet
        rovecomm::RoveCommPacket<double> stPacket;
        stPacket.unDataId  = 11104;

        stPacket.eDataType = manifest::DataTypes::DOUBLE_T;
        /* Difference threshold for including a waypoint in the packet
        If the magnitude of the difference between latitude and longitude between the last and current
         point is less than this, the waypoint is skipped.
        */
        double minDiff = 0.0001;
        // Keep track of last added waypoint latitude and longitude
        double lastLat = 0.0;
        double lastLon = 0.0;

        for (const auto& waypoint : m_vReferencePath)
        {
            // Convert waypoint to GPSCoordinate
            const geoops::GPSCoordinate& gps = waypoint.GetGPSCoordinate();

            // Difference between last point and this point
            double diff = std::abs(gps.dLatitude - lastLat) + std::abs(gps.dLongitude - lastLon);

            // Skip this waypoint if it is too close to the last one
            if (diff < minDiff)
            {
                LOG_INFO(logging::g_qSharedLogger, "Skipped waypoint: ({}, {})", gps.dLatitude, gps.dLongitude);
                continue;
            }
            
            LOG_INFO(logging::g_qSharedLogger, "Added waypoint: ({}, {})", gps.dLatitude, gps.dLongitude);

            // Add waypoint to the packet data
            stPacket.vData.emplace_back(gps.dLatitude);
            stPacket.vData.emplace_back(gps.dLongitude);
            
            // Track the last waypoint that was added
            lastLat = gps.dLatitude;
            lastLon = gps.dLongitude;
        }

        // Set the datacount to the number of waypoints added
        stPacket.unDataCount = stPacket.vData.size();

        // Send path data over RoveComm to Basestation
        if (network::g_pRoveCommUDPNode)
        {
            // Send packet on local machine (This needs to be changed to actual Basestation IP)
            network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, "192.168.0.117", 9000);

            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Sent waypoint: ()");
        }
    }

    /******************************************************************************
     * @brief Sets the path that the controller will follow.
     *
     * @param vReferencePath - A reference to a vector of UTM coordinates that make up the path.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    void PredictiveStanleyController::SetReferencePath(const std::vector<geoops::UTMCoordinate>& vReferencePath)
    {
        // Convert UTM coordinates to waypoints.
        std::vector<geoops::Waypoint> vConvertedPath;
        for (const geoops::UTMCoordinate& stUTMCoord : vReferencePath)
        {
            vConvertedPath.emplace_back(geoops::Waypoint(stUTMCoord, geoops::WaypointType::eNavigationWaypoint, 0.0, -1));
        }

        // Set the reference path.
        this->SetReferencePath(vConvertedPath);
    }

    /******************************************************************************
     * @brief Sets the path that the controller will follow.
     *
     * @param vReferencePath - A reference to a vector of GPS coordinates that make up the path.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    void PredictiveStanleyController::SetReferencePath(const std::vector<geoops::GPSCoordinate>& vReferencePath)
    {
        // Convert GPS coordinates to waypoints.
        std::vector<geoops::Waypoint> vConvertedPath;
        for (const geoops::GPSCoordinate& stGPSCoord : vReferencePath)
        {
            vConvertedPath.emplace_back(geoops::Waypoint(stGPSCoord, geoops::WaypointType::eNavigationWaypoint, 0.0, -1));
        }

        // Set the reference path.
        this->SetReferencePath(vConvertedPath);
    }

    /******************************************************************************
     * @brief Setter for the control gain of the stanley controller.
     *
     * @param dControlGain - The control gain for the controller.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    void PredictiveStanleyController::SetControlGain(const double dControlGain)
    {
        m_dControlGain = dControlGain;
    }

    /******************************************************************************
     * @brief Setter for the angular velocity limit of the stanley controller.
     *
     * @param dAngularVelocityLimit - The angular velocity limit for the controller.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-11
     ******************************************************************************/
    void PredictiveStanleyController::SetAngularVelocityLimit(const double dAngularVelocityLimit)
    {
        m_dAngularVelocityLimit = dAngularVelocityLimit;
    }

    /******************************************************************************
     * @brief Accessor for the control gain of the stanley controller.
     *
     * @return double - The control gain for the controller.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    double PredictiveStanleyController::GetControlGain() const
    {
        return m_dControlGain;
    }

    /******************************************************************************
     * @brief Accessor for the angular velocity limit of the stanley controller.
     *
     * @return double - The angular velocity limit for the controller.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-11
     ******************************************************************************/
    double PredictiveStanleyController::GetAngularVelocityLimit() const
    {
        return m_dAngularVelocityLimit;
    }

    /******************************************************************************
     * @brief Accessor for the reference path that the controller is following.
     *
     * @return std::vector<geoops::Waypoint> - A copy of the reference path.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    std::vector<geoops::Waypoint> PredictiveStanleyController::GetReferencePath() const
    {
        return m_vReferencePath;
    }

    /******************************************************************************
     * @brief Accessor for the current target index in the reference path.
     *
     * @return double - The current target index in the reference path.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    double PredictiveStanleyController::GetReferencePathTargetIndex() const
    {
        return m_nCurrentReferencePathTargetIndex;
    }

    /******************************************************************************
     * @brief Given the current position of the rover, find the point on the reference
     *      path that is closest to the rover's front axle position (based on the wheelbase).
     *      This is what makes sure the rover progresses forward in indexes along the path.
     *
     * @param stCurrentPosition - The current position of the rover.
     * @param dCurrentHeading - The current heading of the rover in degrees from 0-360.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    geoops::Waypoint PredictiveStanleyController::FindClosestWaypointInPath(const geoops::UTMCoordinate& stCurrentPosition)
    {
        // Create instance variables.
        geoops::Waypoint stClosestWaypoint;
        double dClosestDistanceSq = std::numeric_limits<double>::max();
        const size_t nWaypoints   = m_vReferencePath.size();

        // Check for empty path.
        if (nWaypoints == 0)
        {
            return stClosestWaypoint;
        }

        // If only a single waypoint, return it directly.
        if (nWaypoints == 1)
        {
            m_nCurrentReferencePathTargetIndex = 0;
            return m_vReferencePath.front();
        }

        // We'll search across path segments (i -> i+1) and compute the closest point on each segment.
        int nBestSegmentIndex = -1;
        geoops::UTMCoordinate stBestProjection;

        for (size_t siIter = m_nCurrentReferencePathTargetIndex; siIter < nWaypoints - 1; ++siIter)
        {
            const geoops::UTMCoordinate& stA = m_vReferencePath[siIter].GetUTMCoordinate();
            const geoops::UTMCoordinate& stB = m_vReferencePath[siIter + 1].GetUTMCoordinate();

            // Segment vector. (B - A)
            double dSX = stB.dEasting - stA.dEasting;
            double dSY = stB.dNorthing - stA.dNorthing;

            // Vector from A -> P.
            double dPX                   = stCurrentPosition.dEasting - stA.dEasting;
            double dPY                   = stCurrentPosition.dNorthing - stA.dNorthing;

            double sSegmentLengthSquared = dSX * dSX + dSY * dSY;
            double dProjT                = 0.0;
            if (sSegmentLengthSquared > 0.0)
            {
                // projection parameter t = dot(P-A, B-A) / |B-A|^2.
                dProjT = (dPX * dSX + dPY * dSY) / sSegmentLengthSquared;
                dProjT = std::clamp(dProjT, 0.0, 1.0);
            }
            else
            {
                // Degenerate segment (A == B). Use A as projection.
                dProjT = 0.0;
            }

            // Projection point coordinates.
            double dProjX = stA.dEasting + dProjT * dSX;
            double dProjY = stA.dNorthing + dProjT * dSY;

            // Squared distance from P to projection.
            double dx     = stCurrentPosition.dEasting - dProjX;
            double dy     = stCurrentPosition.dNorthing - dProjY;
            double distSq = dx * dx + dy * dy;

            if (distSq < dClosestDistanceSq)
            {
                dClosestDistanceSq                         = distSq;
                nBestSegmentIndex                          = static_cast<int>(siIter);
                stBestProjection.dEasting                  = dProjX;
                stBestProjection.dNorthing                 = dProjY;
                stBestProjection.nZone                     = stA.nZone;
                stBestProjection.bWithinNorthernHemisphere = stA.bWithinNorthernHemisphere;
            }
        }

        // If we didn't find a segment (shouldn't happen), fall back to the closest waypoint vertex search.
        if (nBestSegmentIndex < 0)
        {
            double dClosestDist = std::numeric_limits<double>::max();
            for (size_t siIter = 0; siIter < nWaypoints; ++siIter)
            {
                double dDistance = geoops::CalculateGeoMeasurement(stCurrentPosition, m_vReferencePath[siIter].GetUTMCoordinate()).dDistanceMeters;
                if (dDistance < dClosestDist)
                {
                    dClosestDist                       = dDistance;
                    stClosestWaypoint                  = m_vReferencePath[siIter];
                    m_nCurrentReferencePathTargetIndex = static_cast<int>(siIter);
                }
            }
            return stClosestWaypoint;
        }

        // Build a waypoint for the projected point.
        // Use the properties of the segment start waypoint as a base, then set the UTM to the projection.
        geoops::Waypoint stSegmentStart   = m_vReferencePath[nBestSegmentIndex];
        geoops::UTMCoordinate stBestCoord = geoops::UTMCoordinate(stBestProjection.dEasting,
                                                                  stBestProjection.dNorthing,
                                                                  stBestProjection.nZone,
                                                                  stBestProjection.bWithinNorthernHemisphere,
                                                                  stSegmentStart.GetUTMCoordinate().dAltitude,
                                                                  stSegmentStart.GetUTMCoordinate().d2DAccuracy,
                                                                  stSegmentStart.GetUTMCoordinate().d3DAccuracy,
                                                                  stSegmentStart.GetUTMCoordinate().dMeridianConvergence,
                                                                  stSegmentStart.GetUTMCoordinate().dScale,
                                                                  stSegmentStart.GetUTMCoordinate().eCoordinateAccuracyFixType,
                                                                  stSegmentStart.GetUTMCoordinate().bIsDifferential);
        stClosestWaypoint                 = geoops::Waypoint(stBestCoord);

        // Update the controller's current reference index to the segment start index so callers can use (index + 1).
        m_nCurrentReferencePathTargetIndex = nBestSegmentIndex;

        return stClosestWaypoint;
    }
}    // namespace controllers
