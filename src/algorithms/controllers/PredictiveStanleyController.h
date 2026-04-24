/******************************************************************************
 * @brief Defines the Predictive Stanley Controller class.
 *
 * @file PredictiveStanleyController.h
 * @author clayjay3 (claytonraycowen@gmail.com) Bailey Schoenike (baileyps03@gmail.com)
 * @date 2025-01-10
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef PREDICTIVE_STANLEY_CONTROLLER_H
#define PREDICTIVE_STANLEY_CONTROLLER_H

#include "../../util/GeospatialOperations.hpp"
#include "../../util/logging/PathTracer.hpp"
#include "../kinematics/UnicycleModel.hpp"

/// \cond
#include <utility>
#include <vector>

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
     * @brief This class implements the Predictive Stanley Controller. This controller
     * is used to follow a path using the Stanley method with predictive control.
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
                    double dThetaHeading;
                    double dVelocity;
            };

            /////////////////////////////////////////
            // Declare public class methods.
            /////////////////////////////////////////
            PredictiveStanleyController(const double dControlGain          = 0.1,
                                        const double dAngularVelocityLimit = 90.0,
                                        const int nPredictionHorizon       = 5,
                                        const double dPredictionTimeStep   = 0.01);
            ~PredictiveStanleyController();
            DriveVector Calculate(const geoops::RoverPose& stCurrentPose, const double dMaxSpeed = constants::NAVIGATING_MOTOR_POWER);

            /////////////////////////////////////////
            // Setters.
            /////////////////////////////////////////

            void SetReferencePath(const std::vector<geoops::Waypoint>& vReferencePath);
            void SetReferencePath(const std::vector<geoops::UTMCoordinate>& vReferencePath);
            void SetReferencePath(const std::vector<geoops::GPSCoordinate>& vReferencePath);
            void SetControlGain(const double dControlGain);
            void SetAngularVelocityLimit(const double dAngularVelocityLimit);

            /////////////////////////////////////////
            // Getters.
            /////////////////////////////////////////

            std::vector<geoops::Waypoint> GetReferencePath() const;
            double GetControlGain() const;
            double GetAngularVelocityLimit() const;
            double GetReferencePathTargetIndex() const;

        private:
            /////////////////////////////////////////
            // Declare private class methods.
            /////////////////////////////////////////

            std::pair<geoops::Waypoint, int> FindClosestWaypointInPath(const geoops::UTMCoordinate& stCurrentPosition, const int nStartIndex) const;

            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////

            UnicycleModel m_UnicycleModel;
            double m_dControlGain;
            double m_dAngularVelocityLimit;
            int m_nPredictionHorizon;
            double m_dPredictionTimeStep;
            int m_nCurrentReferencePathTargetIndex;
            std::vector<geoops::Waypoint> m_vReferencePath;
    };
}    // namespace controllers
#endif
