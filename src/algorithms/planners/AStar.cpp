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

#include "AStar.h"
#include "../../AutonomyGlobals.h"

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
     * @brief Helper functor implemented for std::make_heap vOpenList in the
     *      PlanAvoidanceRoute() method to make it a min-heap.
     *
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-02
     ******************************************************************************/
    struct AStar::NodeGreaterThan
    {
            bool operator()(const nodes::AStarNode& lhs, const nodes::AStarNode& rhs) const { return lhs.dKf > rhs.dKf; }
    };

    /******************************************************************************
     * @brief Struct to represent the obstacles that need to be avoided by the
     *      PlanAvoidanceRoute method. fRadius is meant to represent the estimated size
     *      of the obstacle in meters.
     *
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-06
     ******************************************************************************/
    struct AStar::Obstacle
    {
            geoops::UTMCoordinate stCenterPoint;
            float fRadius;
    };

    /******************************************************************************
     * @brief Helper function to destroy objects from m_vObstacles.
     *
     * @todo Implement this.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-02
     ******************************************************************************/
    void AStar::ClearObstacleData() {}

    /******************************************************************************
     * @brief Intended to be called as a helper function by the PlanAvoidancePath method.
     *      This method takes in a vector of ObjectData objects and translates them to
     *      a UTMCoordinate and estimated size that is stored in the m_vObstacles vector.
     *
     * @param vObstacles - A vector reference containing ObjectData objects from the ZEDCam class.
     *
     * @todo Implement this.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-02
     ******************************************************************************/
    void AStar::UpdateObstacleData(const std::vector<sl::ObjectData>& vObstacles) {}

    /******************************************************************************
     * @brief Helper function for the PlanAvoidancePath method. This method takes in
     *      a UTMCoordinate reference and uses class member variables to mutate the
     *      m_stGoalNode object's coordinates to represent the nearest boundary point.
     *
     *      Idea - Get bearing angle from trig, use angle to determine edge node
     *
     * @param stGoalCoordinate - UTMCoordinate reference representing the current rover destination.
     *
     * @todo Implement this.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-02
     ******************************************************************************/
    void AStar::FindNearestBoundaryPoint(const geoops::UTMCoordinate& stGoalCoordinate) {}

    /******************************************************************************
     * @brief Helper function used to translate a UTMCoordinate's dEasting and dNorthing
     *      values into a string that can be hashed for the unordered_map data structure
     *      for O(1) lookup of nodes at a particular location.
     *
     * @param stToTranslate - A UTMCoordinate struct reference containing the data to translate.
     * @param szTranslated - A string to be mutated to contain the translated coordinate.
     *
     * @todo This should provide unique strings within our operation scope without having to
     *      do too many operations. Check with Clayton to see if this is viable.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-05
     ******************************************************************************/
    void AStar::UTMCoordinateToString(const geoops::UTMCoordinate& stToTranslate, std::string& szTranslation)
    {
        szTranslation = std::to_string(stToTranslate.dEasting);
        szTranslation.append(std::to_string(stToTranslate.dNorthing));
    }

    /******************************************************************************
     * @brief Helper function used to determine if a potential UTMCoordinate is valid.
     *      Returns False if there is an obstacle is blocking the node or the node is
     *      outside the max boundary. Returns True otherwise, representing the node is a
     *      valid path to consider. To save memory and compute time, we only evaluate the
     *      doubles representing the coordinate.
     *
     * @param dEasting - A const double reference representing a dEasting to evaluate.
     * @param dNorthing - A const double reference representing a dNorthing to evaluate.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-06
     ******************************************************************************/
    bool AStar::ValidCoordinate(const double& dEasting, const double& dNorthing)
    {
        // Boundary check (Figure out how to override auto-format).
        if (dEasting < (dEasting - m_dMaximumSearchGridSize) || dEasting > (dEasting + m_dMaximumSearchGridSize) || dNorthing < (dNorthing - m_dMaximumSearchGridSize) ||
            dNorthing < (dNorthing + m_dMaximumSearchGridSize))
        {
            return false;
        }
        // For each obstacle.
        for (int i = 0; i < m_vObstacles.size(); i++)
        {
            // Multiplier for avoidance radius.
            double dAvoidanceRadius = m_fAvoidanceMultiplier * m_vObstacles[i].fRadius;
            // Create obstacle borders.
            double dEastObstacleBorder  = m_vObstacles[i].stCenterPoint.dEasting + dAvoidanceRadius;
            double dWestObstacleBorder  = m_vObstacles[i].stCenterPoint.dEasting - dAvoidanceRadius;
            double dNorthObstacleBorder = m_vObstacles[i].stCenterPoint.dNorthing + dAvoidanceRadius;
            double dSouthObstacleBorder = m_vObstacles[i].stCenterPoint.dNorthing - dAvoidanceRadius;

            // Return false if node is within obstacle border.
            if (dEasting > dWestObstacleBorder && dEasting < dEastObstacleBorder && dNorthing < dNorthObstacleBorder && dNorthing > dSouthObstacleBorder)
            {
                return false;
            }
            return true;
        }
    }

    /******************************************************************************
     * @brief Helper function used to round UTMCoordinates to the nearest m_dNodeSize to avoid
     *      rounding errors when trying to determine if two nodes have the same location.
     *
     * @param stCoordinateToRound - A UTMCoordinate reference that will have its dNorthing and dEasting values
     *                              mutated to round them to the nearest m_dNodeSize.
     *
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-12
     ******************************************************************************/
    void AStar::RoundUTMCoordinate(geoops::UTMCoordinate& stRoundCoordinate)
    {
        stRoundCoordinate.dEasting  = std::round(stRoundCoordinate.dEasting / m_dNodeSize) * m_dNodeSize;
        stRoundCoordinate.dNorthing = std::round(stRoundCoordinate.dNorthing / m_dNodeSize) * m_dNodeSize;
    }

    /******************************************************************************
     * @brief Called in the obstacle avoidance state to plan a path around obstacles
     *      blocking our path.
     *
     * @param vObstacles - A vector reference containing ObjectData objects from the ZEDCam class.
     *
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-02
     ******************************************************************************/
    void AStar::PlanAvoidancePath(const std::vector<sl::ObjectData>& vObstacles)
    {
        // Translate Object data from camera and construct obstacle nodes.
        // Stores Data in m_vObstacles.
        UpdateObstacleData(vObstacles);

        // Create Start and Goal nodes
        geoops::UTMCoordinate stStartCoordinate  = globals::g_pNavigationBoard->GetUTMData();
        WaypointHandler::Waypoint stGoalWaypoint = globals::g_pWaypointHandler->PeekNextWaypoint();
        geoops::UTMCoordinate stGoalCoordinate   = stGoalWaypoint.GetUTMCoordinate();
        // Map the goalLocation to an edge node based on maximum search size.
        // FindNearestBoundaryPoint(stGoalCoordinate);    // Not implemented yet...
        // Round Coordinates.
        RoundUTMCoordinate(stStartCoordinate);
        RoundUTMCoordinate(stGoalCoordinate);
        // Create AStarNodes.
        m_stStartNode = nodes::AStarNode(nullptr, stStartCoordinate);
        m_stGoalNode  = nodes::AStarNode(nullptr, stGoalCoordinate);

        // -------------------A* algorithm-------------------
        // Create Open and Closed Lists.
        // Using an additional unordered map is memory inefficient but allows for O(1)
        //  lookup of nodes based on their position rather than iterating over the heap.
        // Carefully manage nodes between 'lists' to ensure data is consistent.

        // Open list implemented as a min-heap queue for O(1) retrieval of the node with min dKf value.
        // C++ utilizes the '*_heap' family of functions which operate on vectors.
        std::vector<nodes::AStarNode> vOpenList;
        std::make_heap(vOpenList.begin(), vOpenList.end(), NodeGreaterThan());
        // Unordered map of coordinates for open list for O(1) lookup.
        std::unordered_map<std::string, double> stdOpenListLookup;
        // Vector containing nodes on the closed list.
        std::vector<nodes::AStarNode> vClosedList;
        // Unordered map of coordinates for closed list for O(1) lookup.
        std::unordered_map<std::string, double> stdClosedList;
        // Place Starting node on open list.
        vOpenList.push_back(m_stStartNode);
        // TODO: Is this necessary since this will be the only node?
        // std::push_heap(vOpenList.begin(), vOpenList.end(), NodeGreaterThan());
        // Translate start node to string and add location on open list lookup map.
        std::string szLocationString;
        UTMCoordinateToString(m_stStartNode.stNodeLocation, szLocationString);
        stdOpenListLookup.emplace(std::make_pair(szLocationString, 0.0));

        // While open list is not empty:
        while (!vOpenList.empty())
        {
            // Retrieve node with the minimum dKf on open list (Q).
            std::pop_heap(vOpenList.begin(), vOpenList.end(), NodeGreaterThan());
            nodes::AStarNode nextParent = vOpenList.back();
            // Pop Q off open list.
            vOpenList.pop_back();
            // Generate Q's 8 successors (neighbors), setting parent to Q.
            std::vector<nodes::AStarNode> vSuccessors;
            double dWestOffset  = nextParent.stNodeLocation.dEasting - m_dNodeSize;
            double dEastOffset  = nextParent.stNodeLocation.dEasting + m_dNodeSize;
            double dSouthOffset = nextParent.stNodeLocation.dNorthing - m_dNodeSize;
            double dNorthOffset = nextParent.stNodeLocation.dNorthing + m_dNodeSize;
            for (double dEastingOffset = dWestOffset; dEastingOffset < dEastOffset; dEastingOffset += m_dNodeSize)
            {
                for (double dNorthingOffset = dSouthOffset; dNorthingOffset < dNorthOffset; dNorthingOffset += m_dNodeSize)
                {
                    // Skip duplicating the parent node.
                    // TODO: Optimize this with a counter instead of multiple evals.
                    if (dEastingOffset == nextParent.stNodeLocation.dEasting && dNorthingOffset == nextParent.stNodeLocation.dNorthing)
                    {
                        continue;
                    }
                    // Check for valid coordinate (check for boundary and obstacles).
                    if (!ValidCoordinate(dEastingOffset, dNorthingOffset))
                    {
                        continue;
                    }
                    // Copy most data from parent coordinate.
                    geoops::UTMCoordinate successorCoordinate = nextParent.stNodeLocation;
                    // Adjust Easting and Northing offsets.
                    successorCoordinate.dEasting  = dEastingOffset;
                    successorCoordinate.dNorthing = dNorthingOffset;
                    RoundUTMCoordinate(successorCoordinate);
                    // Create successor node, initialize values to 0 (done by constructor).
                    nodes::AStarNode nextSuccessor(&nextParent, successorCoordinate);
                    // Copy successor node to vector.
                    vSuccessors.emplace_back(nextSuccessor);
                }
            }
            // For each successor:
            for (char i = 0; i < vSuccessors.size(); i++)
            {
                // If successor = goal, stop search.
                if (vSuccessors[i].stNodeLocation == m_stGoalNode.stNodeLocation)
                {
                    // TODO: Build path (vector of AStarNodes) by back-tracing parents.
                    return;
                }

                // Create and format lookup string.
                std::string szSuccessorLookup;
                UTMCoordinateToString(vSuccessors[i].stNodeLocation, szSuccessorLookup);

                // Compute dKg, dKh, and dKf for successor.
                // Calculate successor previous path cost.
                double dSuccessorKg = nextParent.dKg + m_dNodeSize;
                // Calculate successor future path cost through Diagonal distance heuristic.
                // TODO: Move to helper function for readability?
                double dGoalEastingDistance  = std::abs(vSuccessors[i].stNodeLocation.dEasting - m_stGoalNode.stNodeLocation.dEasting);
                double dGoalNorthingDistance = std::abs(vSuccessors[i].stNodeLocation.dNorthing - m_stGoalNode.stNodeLocation.dNorthing);
                double dSuccessorKh          = m_dNodeSize * (dGoalEastingDistance + dGoalNorthingDistance) +
                                      (m_dSqrtNodeSize - 2 * m_dNodeSize) * std::min(dGoalEastingDistance, dGoalNorthingDistance);
                // f = g + h
                double dSuccessorKf = dSuccessorKg + dSuccessorKh;

                // If a node with the same position as successor is in the open list and has a lower dKf, skip this successor.
                if (stdOpenListLookup.contains(szSuccessorLookup))
                {
                    if (stdOpenListLookup[szSuccessorLookup] < vSuccessors[i].dKf)
                    {
                        continue;
                    }
                }
                // If a node with the same position as successor is in the closed list and has a lower dKf, skip this successor.
                if (stdClosedList.contains(szSuccessorLookup))
                {
                    if (stdClosedList[szSuccessorLookup] < vSuccessors[i].dKf)
                    {
                        continue;
                    }
                }
                // Otherwise add successor node to open list.
                // Add lookup string and dKf value to lookup map.
                stdOpenListLookup.emplace(std::make_pair(szSuccessorLookup, vSuccessors[i].dKf));
                // Push to heap.
                vOpenList.push_back(vSuccessors[i]);
                std::push_heap(vOpenList.begin(), vOpenList.end(), NodeGreaterThan());
            }    // End For(each successor).
            // Create and format lookup string.
            std::string szParentLookup;
            UTMCoordinateToString(nextParent.stNodeLocation, szParentLookup);
            // Push lookup string and dKf value to lookup map.
            stdClosedList.emplace(std::make_pair(szParentLookup, nextParent.dKf));
            // Push Q to the closed list.
            vClosedList.emplace_back(nextParent);
        }    // End While(!vOpenList.empty)
    }
}    // namespace pathplanners
