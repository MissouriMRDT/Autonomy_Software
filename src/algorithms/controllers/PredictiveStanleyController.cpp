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
        m_dControlGain = constants::STANLEY_CROSSTRACK_CONTROL_GAIN;
        // m_dWheelbase                       = constants::STANLEY_DIST_TO_FRONT_AXLE;
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
        for (size_t nIter = 0.0; nIter < vPredictions.size(); ++nIter)
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
            geoops::Waypoint stClosestWaypoint = FindClosestWaypointInPath(stPredictedPosition, dPredictedTheta);

            // Compute the heading error. This is the difference between the heading of the rover and the heading or curvature of the path.
            double dHeadingError = numops::AngularDifference(m_vReferencePathCurvature[m_nCurrentReferencePathTargetIndex], dPredictedTheta);

            /*
                Compute the cross track error. This is the distance between the predicted position and the closest point on the path. The sign of the cross track error
                indicates which side of the path the rover is on. Left is positive, right is negative. The sign of the crosstrack error is determined by the sign of the
            */

            // Get the reference path vector: from the closest waypoint to the next waypoint.
            double dForwardVectorX = m_vReferencePath[m_nCurrentReferencePathTargetIndex + 1].GetUTMCoordinate().dEasting - stClosestWaypoint.GetUTMCoordinate().dEasting;
            double dForwardVectorY =
                m_vReferencePath[m_nCurrentReferencePathTargetIndex + 1].GetUTMCoordinate().dNorthing - stClosestWaypoint.GetUTMCoordinate().dNorthing;
            // Compute the norm and unit vector for the path segment.
            double dForwardNorm = sqrt(dForwardVectorX * dForwardVectorX + dForwardVectorY * dForwardVectorY);
            double dFwdUnitX    = dForwardVectorX / dForwardNorm;
            double dFwdUnitY    = dForwardVectorY / dForwardNorm;
            // Get the vehicle's position vector relative to the closest waypoint.
            double dVehicleVectorX = dPredictedXPosition - stClosestWaypoint.GetUTMCoordinate().dEasting;
            double dVehicleVectorY = dPredictedYPosition - stClosestWaypoint.GetUTMCoordinate().dNorthing;
            // Project the vehicle vector onto the path unit vector to obtain the longitudinal component.
            double dLongitudinal = dVehicleVectorX * dFwdUnitX + dVehicleVectorY * dFwdUnitY;
            // Compute the lateral error vector by subtracting the longitudinal projection from the vehicle vector.
            double dLateralX = dVehicleVectorX - dLongitudinal * dFwdUnitX;
            double dLateralY = dVehicleVectorY - dLongitudinal * dFwdUnitY;
            // The cross-track error is the magnitude of this lateral vector.
            double dLateralDistance = sqrt(dLateralX * dLateralX + dLateralY * dLateralY);
            // Determine the sign of the cross-track error using the cross product (left positive, right negative).
            int nCrossTrackErrorSign = (dFwdUnitX * dVehicleVectorY - dFwdUnitY * dVehicleVectorX) > 0 ? 1 : -1;
            // Final cross-track error.
            double dCrossTrackError = nCrossTrackErrorSign * dLateralDistance;

            // Apply an exponential weight factor that decreases as we predict further into the future.
            double dTimeWeight = std::exp(-1.5 * static_cast<double>(nIter));
            // Limit the cross track error steering angle.
            dCrossTrackError = std::clamp(m_dControlGain * dCrossTrackError, -m_dAngularVelocityLimit, m_dAngularVelocityLimit);
            // Calculate the steering angle using lateral and heading errors, weighted by the time step.
            dSteeringAngle += dTimeWeight * (dCrossTrackError - dHeadingError);

            // Limit the steering angle to the given limit.
            dSteeringAngle = std::clamp(dSteeringAngle, -m_dAngularVelocityLimit, m_dAngularVelocityLimit);
        }

        // The new steering heading must be from 0-360 degrees.
        double dAbsoluteHeadingGoal = numops::InputAngleModulus(stCurrentPose.GetCompassHeading() + dSteeringAngle, 0.0, 360.0);

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
        // Create instance variables.
        double dCurvature = 0.0;

        // Apply path smoothing first.
        std::vector<geoops::Waypoint> vSmoothedPath = pathplanners::postprocessing::FitPathWithBSpline(vReferencePath);

        // Loop through the reference path and calculate the curvature at each point.
        for (size_t nIter = 0; nIter < vSmoothedPath.size(); ++nIter)
        {
            // Calculate the curvature at this point.
            if (nIter > 0 && nIter < vSmoothedPath.size() - 1)
            {
                // Calculate the curvature.
                dCurvature = geoops::CalculateGeoMeasurement(vSmoothedPath[nIter - 1].GetUTMCoordinate(), vSmoothedPath[nIter].GetUTMCoordinate()).dStartRelativeBearing;

                // If this is the second iteration, then also set the curvature of the previous point.
                if (nIter == 1)
                {
                    m_vReferencePathCurvature[0] = dCurvature;
                }
            }

            // Store the waypoint and curvature.
            m_vReferencePathCurvature.push_back(dCurvature);
        }

        // Reset the current target index.
        m_nCurrentReferencePathTargetIndex = 0;
        // Reset the bicycle model.
        m_UnicycleModel.ResetState();
        // Set the reference path.
        m_vReferencePath = vSmoothedPath;
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
        // Create instance variables.
        double dCurvature = 0.0;

        // Apply path smoothing first.
        std::vector<geoops::Waypoint> vSmoothedPath = pathplanners::postprocessing::FitPathWithBSpline(vReferencePath);

        // Loop through the reference path and calculate the curvature at each point.
        for (size_t nIter = 0; nIter < vSmoothedPath.size(); ++nIter)
        {
            // Calculate the curvature at this point.
            if (nIter > 0 && nIter < vSmoothedPath.size() - 1)
            {
                // Calculate the curvature.
                dCurvature = geoops::CalculateGeoMeasurement(vSmoothedPath[nIter - 1].GetUTMCoordinate(), vSmoothedPath[nIter].GetUTMCoordinate()).dStartRelativeBearing;

                // If this is the second iteration, then also set the curvature of the previous point.
                if (nIter == 1)
                {
                    m_vReferencePathCurvature[0] = dCurvature;
                }
            }

            // Store the waypoint and curvature.
            m_vReferencePathCurvature.push_back(dCurvature);
        }

        // Reset the current target index.
        m_nCurrentReferencePathTargetIndex = 0;
        // Reset the unicycle model.
        m_UnicycleModel.ResetState();
        // Set the reference path.
        m_vReferencePath = vSmoothedPath;
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
        // Create instance variables.
        double dCurvature = 0.0;

        // Apply path smoothing first.
        std::vector<geoops::Waypoint> vSmoothedPath = pathplanners::postprocessing::FitPathWithBSpline(vReferencePath);

        // Loop through the reference path and calculate the curvature at each point.
        for (size_t nIter = 0; nIter < vSmoothedPath.size(); ++nIter)
        {
            // Calculate the curvature at this point.
            if (nIter > 0 && nIter < vSmoothedPath.size() - 1)
            {
                // Calculate the curvature.
                dCurvature = geoops::CalculateGeoMeasurement(vSmoothedPath[nIter - 1].GetUTMCoordinate(), vSmoothedPath[nIter].GetUTMCoordinate()).dStartRelativeBearing;

                // If this is the second iteration, then also set the curvature of the previous point.
                if (nIter == 1)
                {
                    m_vReferencePathCurvature[0] = dCurvature;
                }
            }

            // Store the waypoint and curvature.
            m_vReferencePathCurvature.push_back(dCurvature);
        }

        // Reset the current target index.
        m_nCurrentReferencePathTargetIndex = 0;
        // Reset the unicycle model.
        m_UnicycleModel.ResetState();
        // Set the reference path.
        m_vReferencePath = vSmoothedPath;
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
     * @brief Setter for the wheelbase of the stanley controller.
     *
     * @param dWheelbase - The distance between the front and rear axles of the rover.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-11
     ******************************************************************************/
    //    void PredictiveStanleyController::SetWheelbase(const double dWheelbase)
    //    {
    //        m_dWheelbase = dWheelbase;
    //    }

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
     * @brief Accessor for the wheelbase of the stanley controller.
     *
     * @return double - The wheelbase of the controller.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-11
     ******************************************************************************/
    //    double PredictiveStanleyController::GetWheelbase() const
    //    {
    //        return m_dWheelbase;
    //    }

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
    geoops::Waypoint PredictiveStanleyController::FindClosestWaypointInPath(const geoops::UTMCoordinate& stCurrentPosition, const double dCurrentHeading)
    {
        geoops::Waypoint stClosestWaypoint;
        double dClosestDistance = std::numeric_limits<double>::max();

        // Loop through the reference path.
        for (size_t nIter = 0; nIter < m_vReferencePath.size(); ++nIter)
        {
            // Calculate the distance to the current waypoint.
            double dDistance = geoops::CalculateGeoMeasurement(stCurrentPosition, m_vReferencePath[nIter].GetUTMCoordinate()).dDistanceMeters;

            // Check if this waypoint is closer.
            if (dDistance < dClosestDistance)
            {
                stClosestWaypoint                  = m_vReferencePath[nIter];
                dClosestDistance                   = dDistance;
                m_nCurrentReferencePathTargetIndex = nIter;
            }
        }

        return stClosestWaypoint;
    }
}    // namespace controllers
