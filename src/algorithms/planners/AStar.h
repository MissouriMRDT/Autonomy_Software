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
     * @todo Make Obstacle into struct for readability.
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-01
     ******************************************************************************/
    class AStar
    {
        private:
            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////
            struct NodeGreaterThan;
            struct Obstacle;
            // Start and Goal Nodes
            nodes::AStarNode m_stStartNode;
            nodes::AStarNode m_stGoalNode;
            // Nodes used as the final path for routing
            std::vector<geoops::UTMCoordinate> m_vPathCoordinates;
            // Obstacles for AStar to use during routing
            std::vector<Obstacle> m_vObstacles;
            // TODO: Move to constants
            // Multiplier for marking extra nodes around objects as obstacles
            const float m_fAvoidanceMultiplier = 1.2;
            // Maximum search grid size (UTM)
            const double m_dMaximumSearchGridSize = 10.0;
            // Represents the node size / accuracy in meters
            const double m_dNodeSize = 0.5;
            // Square root of m_dNodeSize
            const double m_dSqrtNodeSize = M_SQRT1_2;

            /////////////////////////////////////////
            // Declare private methods.
            /////////////////////////////////////////
            void ClearObstacleData();
            void UpdateObstacleData(const std::vector<sl::ObjectData>& vObstacles);
            void FindNearestBoundaryPoint(const geoops::UTMCoordinate& stGoalCoordinate);
            void UTMCoordinateToString(const geoops::UTMCoordinate& stToTranslate, std::string& szTranslation);
            bool ValidCoordinate(const double& dEasting, const double& dNorthing);
            void RoundUTMCoordinate(geoops::UTMCoordinate& stCoordinateToRound);
            double CalculateNodeHValue(const nodes::AStarNode& stNodeToCalculate);
            void ConstructPath(const nodes::AStarNode& stFinalNode);

        public:
            /////////////////////////////////////////
            // Declare public member variables.
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Declare public primary methods.
            /////////////////////////////////////////
            AStar();
            ~AStar();
            std::vector<geoops::UTMCoordinate> PlanAvoidancePath(const std::vector<sl::ObjectData>& vObstacles);
            /////////////////////////////////////////
            // Setters.
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Getters.
            /////////////////////////////////////////
    };
}    // namespace pathplanners

#endif
