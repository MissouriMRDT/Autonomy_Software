// PurePursuitController.h
/******************************************************************************
 * @brief Defines the Pure Pursuit Controller class.
 *
 * @file PurePursuitController.h
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2026-03-15
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H

#include "../../util/GeospatialOperations.hpp"
#include "../../util/logging/PathTracer.hpp"

/// \cond
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
     * @brief This class implements the Pure Pursuit Controller. This controller
     * is used to follow a path by calculating the steering angle required
     * to reach a lookahead point a set distance down the reference path.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-03-15
     ******************************************************************************/
    class PurePursuitController
    {
        public:
            /////////////////////////////////////////
            // Declare public structs.
            /////////////////////////////////////////

            /******************************************************************************
             * @brief The struct for the drive vector that includes heading and velocity.
             *
             *
             * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
             * @date 2026-03-15
             ******************************************************************************/
            struct DriveVector
            {
                public:
                    double dThetaHeading;
                    double dVelocity;
            };

            /////////////////////////////////////////
            // Constructors and Destructors.
            /////////////////////////////////////////

            PurePursuitController(const double dLookaheadDistance = 1.0, const int nLookaheadIndex = 5);
            ~PurePursuitController() = default;

            /////////////////////////////////////////
            // Declare public class methods.
            /////////////////////////////////////////

            DriveVector Calculate(const geoops::RoverPose& stCurrentPose, const double dMaxSpeed);

            /////////////////////////////////////////
            // Setters.
            /////////////////////////////////////////

            void SetReferencePath(const std::vector<geoops::Waypoint>& vReferencePath);
            void SetLookaheadDistance(const double dLookaheadDistance);
            void SetLookaheadIndex(const int nLookaheadIndex);

            /////////////////////////////////////////
            // Getters.
            /////////////////////////////////////////

            std::vector<geoops::Waypoint> GetReferencePath() const;
            double GetLookaheadDistance() const;
            int GetReferencePathTargetIndex() const;

        private:
            /////////////////////////////////////////
            // Declare private class methods.
            /////////////////////////////////////////

            int FindClosestWaypointIndex(const geoops::UTMCoordinate& stCurrentPosition);
            geoops::Waypoint FindLookaheadWaypoint(const geoops::UTMCoordinate& stCurrentPosition);

            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////

            double m_dLookaheadDistance;
            int m_nLookaheadIndex;
            int m_nCurrentReferencePathTargetIndex;
            std::vector<geoops::Waypoint> m_vReferencePath;
    };
}    // namespace controllers
#endif
