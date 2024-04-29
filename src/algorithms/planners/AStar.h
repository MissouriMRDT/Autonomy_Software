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
            struct Obstacle;

            /////////////////////////////////////////
            // Declare public primary methods.
            /////////////////////////////////////////
            AStar();
            ~AStar();

            std::vector<geoops::UTMCoordinate> PlanAvoidancePath(const geoops::UTMCoordinate& stStartCoordinate,
                                                                 const geoops::UTMCoordinate& stGoalCoordinate,
                                                                 const std::vector<sl::ObjectData>& vObstacles = std::vector<sl::ObjectData>());

            // Moved to public for unit testing.
            geoops::UTMCoordinate FindNearestBoundaryPoint(const geoops::UTMCoordinate& stGoalCoordinate);
            geoops::UTMCoordinate RoundUTMCoordinate(const geoops::UTMCoordinate& stCoordinateToRound);
            void ConstructPath(const nodes::AStarNode& stFinalNode);

            /////////////////////////////////////////
            // Setters.
            /////////////////////////////////////////
            void AddObstacle(const sl::ObjectData& slObstacle);
            void AddObstacle(const Obstacle& stObstacle);
            void UpdateObstacleData(const std::vector<sl::ObjectData>& vObstacles, const bool& bClearObstacles = true);
            void UpdateObstacleData(const std::vector<Obstacle>& vObstacles, const bool& bClearObstacles = true);
            void ClearObstacleData();

            void SetStartCoordinate(const geoops::UTMCoordinate& stStart) { m_stStartNode = nodes::AStarNode(nullptr, stStart); }

            /////////////////////////////////////////
            // Getters.
            /////////////////////////////////////////
            const std::vector<geoops::UTMCoordinate> GetPath() { return m_vPathCoordinates; }

        private:
            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////
            // Start and Goal Nodes
            nodes::AStarNode m_stStartNode;
            nodes::AStarNode m_stGoalNode;
            // Nodes used as the final path for routing
            std::vector<geoops::UTMCoordinate> m_vPathCoordinates;
            // Obstacles for AStar to use during routing
            std::vector<Obstacle> m_vObstacles;

            /////////////////////////////////////////
            // Declare private methods.
            /////////////////////////////////////////
            std::string UTMCoordinateToString(const geoops::UTMCoordinate& stToTranslate);
            bool ValidCoordinate(const double& dEasting, const double& dNorthing);
    };
}    // namespace pathplanners

#endif
