/******************************************************************************
 * @brief Defines the ASTAR path finder class within the pathplanners
 *      namespace.
 *
 * @file AStar.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-01
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef ASTAR_H
#define ASTAR_H

#include "../../util/planners/Nodes.hpp"

/// \cond
// Put implicit includes in here.
#include <sl/Camera.hpp>

/// \endcond

/******************************************************************************
 * @brief This namespace stores classes, functions, and structs that are used to
 *      implement different path planner algorithms used by the rover to determine
 *      the optimal path to take for any given situation.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-01
 ******************************************************************************/
namespace pathplanners
{
    /******************************************************************************
     * @brief Implements the A* (ASTAR) algorithm with the ability to plan paths around
     *      obstacles and provide path bias points that the algorithm will try to adhere to.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-01
     ******************************************************************************/
    class AStar
    {
        public:
            /////////////////////////////////////////
            // Declare public member variables.
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Declare public primary methods.
            /////////////////////////////////////////
            AStar();
            ~AStar();
            std::vector<geoops::Waypoint> PlanAvoidancePath(const geoops::Waypoint& stStartCoordinate, const geoops::Waypoint& stGoalCoordinate);
            std::vector<geoops::Waypoint> PlanAvoidancePath(const geoops::UTMCoordinate& stStartCoordinate, const geoops::UTMCoordinate& stGoalCoordinate);
            std::vector<geoops::Waypoint> PlanAvoidancePath(const geoops::GPSCoordinate& stStartCoordinate, const geoops::GPSCoordinate& stGoalCoordinate);

            /////////////////////////////////////////
            // Setters.
            /////////////////////////////////////////
            void UpsertObstacleData(const std::vector<geoops::Waypoint>& vObstacles);
            void UpsertObstacleData(const std::vector<geoops::UTMCoordinate>& vObstacles);
            void UpsertObstacleData(const std::vector<geoops::GPSCoordinate>& vObstacles);
            void ClearObstacleData();

            /////////////////////////////////////////
            // Getters.
            /////////////////////////////////////////
            std::vector<geoops::Waypoint> GetPath() const;
            std::vector<geoops::Waypoint> GetObstacleData() const;

        private:
            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////
            // Start and Goal Nodes
            nodes::AStarNode m_stStartNode;
            nodes::AStarNode m_stGoalNode;
            // Nodes used as the final path for routing
            std::vector<geoops::Waypoint> m_vPathCoordinates;
            // Obstacles for AStar to use during routing
            std::vector<geoops::Waypoint> m_vObstacles;
            // Time point for measuring total planning time.
            std::chrono::steady_clock::time_point m_tmStartTime;

            /////////////////////////////////////////
            // Declare private methods.
            /////////////////////////////////////////
            geoops::Waypoint FindNearestGoalPoint(const geoops::UTMCoordinate& stGoalCoordinate);
            geoops::Waypoint FindNearestStartPoint(const geoops::UTMCoordinate& stStartCoordinate);
            void RoundUTMCoordinate(geoops::UTMCoordinate& stCoordinateToRound);
            void ConstructPath(const nodes::AStarNode& stFinalNode);
            std::string UTMCoordinateToString(const geoops::UTMCoordinate& stToTranslate);
            bool ValidCoordinate(const double dEasting, const double dNorthing);
    };
}    // namespace pathplanners

#endif
