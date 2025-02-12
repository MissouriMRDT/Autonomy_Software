/******************************************************************************
 * @brief Defines the Predictive Stanley Controller class.
 *
 * @file PredictiveStanleyController.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-10
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef PREDICTIVE_STANLEY_CONTROLLER_H
#define PREDICTIVE_STANLEY_CONTROLLER_H

#include "../../util/GeospatialOperations.hpp"
#include "../../util/logging/PathTracer2D.hpp"
#include "../kinematics/BicycleModel.hpp"

/// \cond
#include <vector>

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
     * @brief This class implements the Predictive Stanley Controller. This controller
     *      is used to follow a path using the Stanley method with predictive control.
     *
     * @note See docs/WhitePapers/2020-A-Path-Tracking-Algorithm-Using-Predictive-Stanley-Lateral-Controller.pdf
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-10
     ******************************************************************************/
    class PredictiveStanleyController
    {
        public:
            /////////////////////////////////////////
            // Declare public class structs.
            /////////////////////////////////////////
            struct DriveVector
            {
                public:
                    double dSteeringAngle;
                    double dVelocity;
            };

            /////////////////////////////////////////
            // Declare public class methods.
            /////////////////////////////////////////
            PredictiveStanleyController();
            PredictiveStanleyController(const double dControlGain,
                                        const double dSteeringAngleLimit,
                                        const double dWheelbase,
                                        const int nPredictionHorizon,
                                        const double dPredictionTimeStep);
            ~PredictiveStanleyController();
            DriveVector Calculate(const geoops::RoverPose& stCurrentPose);

            /////////////////////////////////////////
            // Setters.
            /////////////////////////////////////////

            void SetReferencePath(const std::vector<geoops::Waypoint>& vReferencePath);
            void SetReferencePath(const std::vector<geoops::UTMCoordinate>& vReferencePath);
            void SetReferencePath(const std::vector<geoops::GPSCoordinate>& vReferencePath);
            void SetControlGain(const double dControlGain);
            void SetSteeringAngleLimit(const double dSteeringAngleLimit);
            void SetWheelbase(const double dWheelbase);

            /////////////////////////////////////////
            // Getters.
            /////////////////////////////////////////

            std::vector<geoops::Waypoint> GetReferencePath() const;
            double GetControlGain() const;
            double GetSteeringAngleLimit() const;
            double GetWheelbase() const;
            double GetReferencePathTargetIndex() const;

        private:
            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////

            BicycleModel m_BicycleModel;
            double m_dControlGain;
            double m_dSteeringAngleLimit;
            double m_dWheelbase;
            int m_nPredictionHorizon;
            double m_dPredictionTimeStep;
            int m_nCurrentReferencePathTargetIndex;
            std::vector<double> m_vReferencePathCurvature;
            std::vector<geoops::Waypoint> m_vReferencePath;

            /////////////////////////////////////////
            // Declare private class methods.
            /////////////////////////////////////////

            geoops::Waypoint FindClosestWaypointInPath(const geoops::UTMCoordinate& stCurrentPosition, const double dCurrentHeading);
    };
}    // namespace controllers
#endif
