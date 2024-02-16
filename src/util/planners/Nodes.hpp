/******************************************************************************
 * @brief Defines and implements different structs and functions used to store data
 *      about path nodes/points for different path planning algorithms. Each struct
 *      and function is defined within the nodes namespace, which is defined in the
 *      pathplanners namespace.
 *
 * @file Nodes.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-01
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef NODES_HPP
#define NODES_HPP

#include "../GeospatialOperations.hpp"

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
     * @brief This namespace defines and implements structs and functions used by
     *      various different algorithms to store data about points within a path.
     *
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-02-01
     ******************************************************************************/
    namespace nodes
    {

        /******************************************************************************
         * @brief This node struct stores point data needed by the A* (ASTAR) algorithm
         *      to properly plan a path. Each node is connected to a parent node and
         *      position, distance from start, heuristic value, and node cost are stored.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-02-01
         ******************************************************************************/
        struct AStarNode
        {
            public:
                // Declare struct public member variables.
                const AStarNode* stParentNode;           // A pointer to the parent node the this nodes comes after in the path.
                geoops::UTMCoordinate stNodeLocation;    // The global position of this point/node stored in UTM format.
                double dKg;                              // The cost of the path from the star node to the current node.
                double dKh;                              // The heuristic estimate of how much it will cost to get to the end node.
                double dKf;                              // The sum of dKg and dKh. An estimate for the total cost of the node.

                /******************************************************************************
                 * @brief Construct a new AStarNode struct.
                 *
                 * @param stParentNode - A pointer to the parent node the this nodes comes after in the path.
                 * @param stNodeLocation - The global position of this point/node stored in UTM format.
                 * @param dKg - The cost of the path from the star node to the current node.
                 * @param dKh - The heuristic estimate of how much it will cost to get to the end node.
                 * @param dKf - The sum of dKg and dKh. An estimate for the total cost of the node.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2024-02-01
                 ******************************************************************************/
                AStarNode(const AStarNode* stParentNode              = nullptr,
                          const geoops::UTMCoordinate stNodeLocation = geoops::UTMCoordinate(),
                          const double dKg                           = 0.0,
                          const double dKh                           = 0.0,
                          const double dKf                           = 0.0)
                {
                    // Initialize struct member variables.
                    this->stParentNode   = stParentNode;
                    this->stNodeLocation = stNodeLocation;
                    this->dKg            = dKg;
                    this->dKh            = dKh;
                    this->dKf            = dKf;
                }

                /******************************************************************************
                 * @brief Overloaded comparison operators for AStarNode struct.
                 *
                 * @param other - The other AStarNode in the comparison.
                 *
                 * @author Kai Shafe (kasq5m@umsystem.edu)
                 * @date 2024-02-15
                 ******************************************************************************/
                bool operator<(const AStarNode& other) const { return this->dKf < other.dKf; }

                bool operator<=(const AStarNode& other) const { return this->dKf <= other.dKf; }

                bool operator>(const AStarNode& other) const { return this->dKf > other.dKf; }

                bool operator>=(const AStarNode& other) const { return this->dKf >= other.dKf; }

                /******************************************************************************
                 * @brief Overloaded equality operator for AStarNode struct. This overload is used
                 *          to see if two nodes have matching coordinates.
                 *
                 * @param other - The other AStarNode in the comparison.
                 *
                 * @author Kai Shafe (kasq5m@umsystem.edu)
                 * @date 2024-02-15
                 ******************************************************************************/

                bool operator==(const AStarNode& other) const
                {
                    return this->stNodeLocation.dEasting == other.stNodeLocation.dEasting && this->stNodeLocation.dNorthing == other.stNodeLocation.dNorthing;
                }
        };
    }    // namespace nodes
}    // namespace pathplanners

#endif
