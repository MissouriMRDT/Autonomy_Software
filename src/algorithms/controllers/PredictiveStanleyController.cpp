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
        m_dWheelbase                       = constants::STANLEY_DIST_TO_FRONT_AXLE;
        m_dSteeringAngleLimit              = constants::STANLEY_STEERING_ANGLE_LIMIT;
        m_nPredictionHorizon               = constants::STANLEY_PREDICTION_HORIZON;
        m_dPredictionTimeStep              = constants::STANLEY_PREDICTION_TIME_STEP;
        m_nCurrentReferencePathTargetIndex = 0;
        m_BicycleModel                     = BicycleModel(m_dWheelbase, 0.0, 0.0, 0.0);
    }

    /******************************************************************************
     * @brief Construct a new Predictive Stanley Controller:: Predictive Stanley Controller object.
     *
     * @param dControlGain - The control gain for the controller.
     * @param dSteeringAngleLimit - The maximum steering angle the rover can turn.
     * @param dWheelbase - The distance between the front and rear axles of the rover.
     * @param nPredictionHorizon - The number of predictions to make.
     * @param dPredictionTimeStep - The time step to predict the future state. How far into the future to predict.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    PredictiveStanleyController::PredictiveStanleyController(const double dControlGain,
                                                             const double dSteeringAngleLimit,
                                                             const double dWheelbase,
                                                             const int nPredictionHorizon,
                                                             const double dPredictionTimeStep)
    {
        // Initialize member variables.
        m_dControlGain                     = dControlGain;
        m_dSteeringAngleLimit              = dSteeringAngleLimit;
        m_dWheelbase                       = dWheelbase;
        m_nPredictionHorizon               = nPredictionHorizon;
        m_dPredictionTimeStep              = dPredictionTimeStep;
        m_nCurrentReferencePathTargetIndex = 0;
        m_BicycleModel                     = BicycleModel(m_dWheelbase, 0.0, 0.0, 0.0);
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
     * @return double - The new output steering angle for the rover.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    PredictiveStanleyController::DriveVector PredictiveStanleyController::Calculate(const geoops::RoverPose& stCurrentPose)
    {
        // Create instance variables.
        double dSteeringAngle = 0.0;
        std::vector<BicycleModel::Prediction> vPredictions;

        // Update the bicycle model with the current state.
        m_BicycleModel.UpdateState(stCurrentPose.GetUTMCoordinate().dEasting, stCurrentPose.GetUTMCoordinate().dNorthing, stCurrentPose.GetCompassHeading());
        // Predict the future state of the model.
        m_BicycleModel.Predict(m_dPredictionTimeStep, m_nPredictionHorizon, vPredictions);

        // Loop through all the predicted future states to compute the steering angle.
        for (size_t nIter = 0.0; nIter < vPredictions.size(); ++nIter)
        {
            // Create instance variables.
            double dPredictedXPosition = vPredictions[nIter].dXPosition;
            double dPredictedYPosition = vPredictions[nIter].dYPosition;
            double dPredictedTheta     = vPredictions[nIter].dTheta;

            // Find the closest point to the reference path.
            geoops::Waypoint stClosestWaypoint = FindClosestWaypointInPath(stCurrentPose.GetUTMCoordinate(), stCurrentPose.GetCompassHeading());
            // Create a UTM coordinate for the predicted position.
            geoops::UTMCoordinate stPredictedPosition = stClosestWaypoint.GetUTMCoordinate();
            stPredictedPosition.dEasting              = dPredictedXPosition;
            stPredictedPosition.dNorthing             = dPredictedYPosition;

            // Compute the heading error. This is the difference between the heading of the rover and the heading or curvature of the path.
            double dHeadingError = m_vReferencePathCurvature[m_nCurrentReferencePathTargetIndex] - dPredictedTheta;

            /*
                Compute the cross track error. This is the distance between the predicted position and the closest point on the path. The sign of the cross track error
                indicates which side of the path the rover is on. Left is positive, right is negative. The sign of the crosstrack error is determined by the sign of the
            */

            // Get the reference path vector. This is the vector from the closest point on the path to a forward point on the path.
            double dForwardVectorX = m_vReferencePath[m_nCurrentReferencePathTargetIndex + 1].GetUTMCoordinate().dEasting - stClosestWaypoint.GetUTMCoordinate().dEasting;
            double dForwardVectorY =
                m_vReferencePath[m_nCurrentReferencePathTargetIndex + 1].GetUTMCoordinate().dNorthing - stClosestWaypoint.GetUTMCoordinate().dNorthing;
            // Get the vehicle position vector. This is the vector from the closest point on the path to the predicted position.
            double dVehicleVectorX = dPredictedXPosition - stClosestWaypoint.GetUTMCoordinate().dEasting;
            double dVehicleVectorY = dPredictedYPosition - stClosestWaypoint.GetUTMCoordinate().dNorthing;
            // Calculate the sign of the cross track error.
            int nCrossTrackErrorSign = (dForwardVectorX * dVehicleVectorY - dForwardVectorY * dVehicleVectorX) > 0 ? 1 : -1;
            // Calculate the cross track error.
            double dCrossTrackError = nCrossTrackErrorSign * geoops::CalculateGeoMeasurement(stClosestWaypoint.GetUTMCoordinate(), stPredictedPosition).dDistanceMeters;

            // Apply an exponential weight factor that decreases as we predict further into the future.
            double dTimeWeight = std::exp(-2.5 * static_cast<double>(nIter));
            // Calculate the steering angle using lateral and heading errors, weighted by the time step.
            dSteeringAngle += dTimeWeight * (m_dControlGain * dCrossTrackError + dHeadingError);

            // Limit the steering angle to the given limit.
            dSteeringAngle = std::min(dSteeringAngle, m_dSteeringAngleLimit);
            dSteeringAngle = std::max(dSteeringAngle, -m_dSteeringAngleLimit);
        }

        // The new steering heading must be from 0-360 degrees.
        dSteeringAngle = numops::InputAngleModulus(stCurrentPose.GetCompassHeading() + dSteeringAngle, 0.0, 360.0);

        return DriveVector{dSteeringAngle, 1.0};
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

        // Loop through the reference path and calculate the curvature at each point.
        for (size_t nIter = 0; nIter < vReferencePath.size(); ++nIter)
        {
            // Calculate the curvature at this point.
            if (nIter > 0 && nIter < vReferencePath.size() - 1)
            {
                // Calculate the curvature.
                dCurvature =
                    geoops::CalculateGeoMeasurement(vReferencePath[nIter - 1].GetUTMCoordinate(), vReferencePath[nIter].GetUTMCoordinate()).dStartRelativeBearing;

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
        m_BicycleModel.ResetState();
        // Set the reference path.
        m_vReferencePath = vReferencePath;
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

        // Clear the current reference path.
        m_vReferencePath.clear();

        // Loop through the reference path and calculate the curvature at each point.
        for (size_t nIter = 0; nIter < vReferencePath.size(); ++nIter)
        {
            // Calculate the curvature at this point.
            if (nIter > 0 && nIter < vReferencePath.size() - 1)
            {
                // Calculate the curvature.
                dCurvature = geoops::CalculateGeoMeasurement(vReferencePath[nIter - 1], vReferencePath[nIter]).dStartRelativeBearing;

                // If this is the second iteration, then also set the curvature of the previous point.
                if (nIter == 1)
                {
                    m_vReferencePathCurvature[0] = dCurvature;
                }
            }

            // Store the waypoint and curvature.
            m_vReferencePathCurvature.push_back(dCurvature);
            // Convert the UTM coordinate to a waypoint.
            m_vReferencePath.push_back(geoops::Waypoint(vReferencePath[nIter]));
        }

        // Reset the current target index.
        m_nCurrentReferencePathTargetIndex = 0;
        // Reset the bicycle model.
        m_BicycleModel.ResetState();
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

        // Clear the current reference path.
        m_vReferencePath.clear();

        // Loop through the reference path and calculate the curvature at each point.
        for (size_t nIter = 0; nIter < vReferencePath.size(); ++nIter)
        {
            // Calculate the curvature at this point.
            if (nIter > 0 && nIter < vReferencePath.size() - 1)
            {
                // Calculate the curvature.
                dCurvature = geoops::CalculateGeoMeasurement(vReferencePath[nIter - 1], vReferencePath[nIter]).dStartRelativeBearing;

                // If this is the second iteration, then also set the curvature of the previous point.
                if (nIter == 1)
                {
                    m_vReferencePathCurvature[0] = dCurvature;
                }
            }

            // Store the waypoint and curvature.
            m_vReferencePathCurvature.push_back(dCurvature);
            // Convert the GPS coordinate to a waypoint.
            m_vReferencePath.push_back(geoops::Waypoint(vReferencePath[nIter]));
        }

        // Reset the current target index.
        m_nCurrentReferencePathTargetIndex = 0;
        // Reset the bicycle model.
        m_BicycleModel.ResetState();
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
     * @brief Setter for the steering angle limit of the stanley controller.
     *
     * @param dSteeringAngleLimit - The steering angle limit for the controller.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-11
     ******************************************************************************/
    void PredictiveStanleyController::SetSteeringAngleLimit(const double dSteeringAngleLimit)
    {
        m_dSteeringAngleLimit = dSteeringAngleLimit;
    }

    /******************************************************************************
     * @brief Setter for the wheelbase of the stanley controller.
     *
     * @param dWheelbase - The distance between the front and rear axles of the rover.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-11
     ******************************************************************************/
    void PredictiveStanleyController::SetWheelbase(const double dWheelbase)
    {
        m_dWheelbase = dWheelbase;
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
     * @brief Accessor for the steering angle limit of the stanley controller.
     *
     * @return double - The steering angle limit for the controller.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-11
     ******************************************************************************/
    double PredictiveStanleyController::GetSteeringAngleLimit() const
    {
        return m_dSteeringAngleLimit;
    }

    /******************************************************************************
     * @brief Accessor for the wheelbase of the stanley controller.
     *
     * @return double - The wheelbase of the controller.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-11
     ******************************************************************************/
    double PredictiveStanleyController::GetWheelbase() const
    {
        return m_dWheelbase;
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
    geoops::Waypoint PredictiveStanleyController::FindClosestWaypointInPath(const geoops::UTMCoordinate& stCurrentPosition, const double dCurrentHeading)
    {
        // Initialize variables.
        geoops::Waypoint stClosestWaypoint;
        geoops::UTMCoordinate stFrontAxlePosition = stCurrentPosition;
        double dClosestDistance                   = std::numeric_limits<double>::max();

        // Convert the heading to radians.
        double dCurrentHeadingRad = dCurrentHeading * M_PI / 180.0;

        // Calculate the current position of the front axle based on the wheelbase, heading, and current position.
        stFrontAxlePosition.dEasting  = stCurrentPosition.dEasting + m_dWheelbase * std::sin(dCurrentHeadingRad);
        stFrontAxlePosition.dNorthing = stCurrentPosition.dNorthing + m_dWheelbase * std::cos(dCurrentHeadingRad);

        // Loop through the reference path.
        for (size_t nIter = 0; nIter < m_vReferencePath.size(); ++nIter)
        {
            // Calculate the distance to the current waypoint.
            double dDistance = geoops::CalculateGeoMeasurement(stFrontAxlePosition, m_vReferencePath[nIter].GetUTMCoordinate()).dDistanceMeters;

            // Check if this waypoint is closer.
            if (dDistance < dClosestDistance)
            {
                // Update the closest waypoint.
                stClosestWaypoint = m_vReferencePath[nIter];
                dClosestDistance  = dDistance;

                // Update the current target index based on the location of the closest waypoint in the vector path.
                m_nCurrentReferencePathTargetIndex = nIter;
            }
        }

        // Return the closest waypoint.
        return stClosestWaypoint;
    }
}    // namespace controllers
