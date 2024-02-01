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
     *
     * @author
     * @date 2024-02-01
     ******************************************************************************/
    class AStar
    {
        private:
            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////
            std::vector<nodes::AStarNode> m_vPathNodes;

            /////////////////////////////////////////
            // Declare private methods.
            /////////////////////////////////////////

        public:
            /////////////////////////////////////////
            // Declare public member variables.
            /////////////////////////////////////////
            AStar();
            ~AStar();

            /////////////////////////////////////////
            // Declare public primary methods.
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Setters.
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Getters.
            /////////////////////////////////////////
    };
}    // namespace pathplanners

#endif
