/******************************************************************************
 * @brief Implements the ASTAR path finder class within the pathplanners
 *      namespace.
 *
 * @file AStar.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-01
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

/******************************************************************************
 * @brief
 *
 *
 * @author Kai Shafe (kasq5m@umsystem.edu)
 * @date 2024-02-02
 ******************************************************************************/
#include "AStar.h"

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
     * @brief Construct a new AStar::AStar object.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-02-01
     ******************************************************************************/
    AStar::AStar()
    {
        // Nothing to do yet.
    }

    /******************************************************************************
     * @brief Destroy the AStar::AStar object.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-02-01
     ******************************************************************************/
    AStar::~AStar()
    {
        // Nothing to destroy yet.
    }

    /******************************************************************************
     * @brief Called in the obstacle avoidance state to plan a path around obstacles
     *      blocking our path.
     *
     * @param vObstacles - A vector containing ObjectData objects from the ZEDCam class.
     *
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-02
     ******************************************************************************/
    void AStar::PlanAvoidancePath(std::vector<sl::ObjectData> vObstacles)
    {
        // Translate Object data from camera and construct obstacle nodes
        // Stores Data in m_vObstacles
        UpdateObstacles(vObstacles);

        // Create Start and Goal nodes
        geoops::UTMCoordinate currentLocation = globals::g_pNavigationBoard->GetUTMData();
        geoops::UTMCoordinate goalLocation = m_StartNode = nodes::AStarNode()

        // Create Open and Closed Lists
    }
}    // namespace pathplanners
