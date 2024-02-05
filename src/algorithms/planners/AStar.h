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
#include "../../vision/cameras/ZEDCam.h"

/// \cond
// Put implicit includes in here.

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
            // Start and Goal Nodes
            nodes::AStarNode m_StartNode;
            nodes::AStarNode m_GoalNode;
            // Nodes used as the path for routing (make this efficient using parent pointers)
            std::vector<nodes::AStarNode> m_vPathNodes;
            // Obstacles for AStar to use during routing
            // 
            std::vector<std::pair<geoops::UTMCoordinate, float>> m_vObstacles;
            // Multiplier for marking extra nodes around objects as obstacles
            const float m_fAvoidanceMultiplier = 1.2;

            /////////////////////////////////////////
            // Declare private methods.
            /////////////////////////////////////////
            void UpdateObstacles(std::vector<sl::ObjectData> vObstacles);

        public:
            /////////////////////////////////////////
            // Declare public member variables.
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Declare public primary methods.
            /////////////////////////////////////////
            AStar();
            ~AStar();
            void PlanAvoidancePath(std::vector<sl::ObjectData> vObstacles);
            /////////////////////////////////////////
            // Setters.
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Getters.
            /////////////////////////////////////////
    };
}    // namespace pathplanners

#endif
