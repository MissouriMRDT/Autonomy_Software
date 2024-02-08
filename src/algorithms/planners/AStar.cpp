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
     * @brief Helper function implemented for std::priority_queue pqOpenList in the
     *      PlanAvoidanceRoute() method.
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
     * @brief Helper function used to determine if a node is valid. Returns True if no
     *      obstacle is blocking the node, returns false if the node is within an object radius.
     *
     * @param stCheckNode - A const node reference containing a node to evaluate.
     *
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-06
     ******************************************************************************/
    bool AStar::ValidNode(const nodes::AStarNode& stCheckNode)
    {
        // For each obstacle
        for (int i = 0; i < m_vObstacles.size(); i++)
        {
            // Multiplier for avoidance radius
            double dAvoidanceSize = m_fAvoidanceMultiplier * m_vObstacles[i].fRadius;
            // Create obstacle borders
            double dEastObstacleBorder  = m_vObstacles[i].stCenterPoint.dEasting + dAvoidanceSize;
            double dWestObstacleBorder  = m_vObstacles[i].stCenterPoint.dEasting - dAvoidanceSize;
            double dNorthObstacleBorder = m_vObstacles[i].stCenterPoint.dNorthing + dAvoidanceSize;
            double dSouthObstacleBorder = m_vObstacles[i].stCenterPoint.dNorthing - dAvoidanceSize;

            // Return false if node is within obstacle border
            if (stCheckNode.stNodeLocation.dEasting > dWestObstacleBorder && stCheckNode.stNodeLocation.dEasting < dEastObstacleBorder &&
                stCheckNode.stNodeLocation.dNorthing < dNorthObstacleBorder && stCheckNode.stNodeLocation.dNorthing > dSouthObstacleBorder)
            {
                return false;
            }
            return true;
        }
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
        // Translate Object data from camera and construct obstacle nodes
        // Stores Data in m_vObstacles
        UpdateObstacleData(vObstacles);

        // Create Start and Goal nodes
        geoops::UTMCoordinate stStartCoordinate  = globals::g_pNavigationBoard->GetUTMData();
        WaypointHandler::Waypoint stGoalWaypoint = globals::g_pWaypointHandler->PeekNextWaypoint();
        geoops::UTMCoordinate stGoalCoordinate   = stGoalWaypoint.GetUTMCoordinate();
        // Map the goalLocation to an edge node based on maximum search size
        // FindNearestBoundaryPoint(stGoalCoordinate);    // Not implemented yet...
        m_stStartNode = nodes::AStarNode(nullptr, stStartCoordinate);
        m_stGoalNode  = nodes::AStarNode(nullptr, stGoalCoordinate);

        // -------------------A* algorithm-------------------
        // Create Open and Closed Lists
        // Using an additional unordered map is memory inefficient but allows for O(1)
        // Carefully manage nodes between 'lists' to ensure data is consistent

        // Open list implemented as a min-heap queue for O(1) retrieval of min
        // TODO: Change to heap and manage manually
        std::priority_queue<nodes::AStarNode, std::vector<nodes::AStarNode>, NodeGreaterThan> pqOpenList;
        // Unordered map of coordinates for open list for O(1) lookup
        std::unordered_map<std::string, double> stdOpenListLookup;
        // Vector containing nodes on the closed list
        std::vector<nodes::AStarNode> vClosedList;
        // Unordered map of coordinates for closed list for O(1) lookup
        std::unordered_map<std::string, double> stdClosedList;
        // Place Starting node on open list
        pqOpenList.emplace(m_stStartNode);
        // Translate to string and add location on open list map
        std::string szLocationString;
        UTMCoordinateToString(m_stStartNode.stNodeLocation, szLocationString);
        stdOpenListLookup.emplace(std::make_pair(szLocationString, 0.0));

        // While open list is not empty:
        while (!pqOpenList.empty())
        {
            // Remove node with least dKf on open list (Q)
            nodes::AStarNode nextParent = pqOpenList.top();
            // Pop Q off open list
            pqOpenList.pop();
            // Generate Q's 8 successors (neighbors) setting parent to Q
            std::vector<nodes::AStarNode> vSuccessors;
            double dWestOffset  = nextParent.stNodeLocation.dEasting - m_dNodeSize;
            double dEastOffset  = nextParent.stNodeLocation.dEasting + m_dNodeSize;
            double dSouthOffset = nextParent.stNodeLocation.dNorthing - m_dNodeSize;
            double dNorthOffset = nextParent.stNodeLocation.dNorthing + m_dNodeSize;
            for (double dEastingOffset = dWestOffset; dEastingOffset < dEastOffset; dEastingOffset += m_dNodeSize)
            {
                for (double dNorthingOffset = dSouthOffset; dNorthingOffset < dNorthOffset; dNorthingOffset += m_dNodeSize)
                {
                    // Skip duplicating the parent node
                    if (dEastingOffset == nextParent.stNodeLocation.dEasting && dNorthingOffset == nextParent.stNodeLocation.dNorthing)
                    {
                        continue;
                    }
                    // Boundary check (Make this more readable)
                    if (dEastingOffset < (m_stStartNode.stNodeLocation.dEasting - m_dMaximumSearchGridSize) ||
                        dEastingOffset > (m_stStartNode.stNodeLocation.dEasting + m_dMaximumSearchGridSize) ||
                        dNorthingOffset < (m_stStartNode.stNodeLocation.dNorthing - m_dMaximumSearchGridSize) ||
                        dNorthingOffset < (m_stStartNode.stNodeLocation.dNorthing + m_dMaximumSearchGridSize))
                    {
                        continue;
                    };
                    // TODO: Obstacle check
                    // Copy most data from parent coordinate
                    geoops::UTMCoordinate successorCoordinate = nextParent.stNodeLocation;
                    // Adjust Easting and Northing offsets
                    // TODO: Determine if these need to be rounded to the nearest m_dNodeSize meter
                    successorCoordinate.dEasting  = dEastingOffset;
                    successorCoordinate.dNorthing = dNorthingOffset;
                    // Create successor node, initialize values to 0 for now (done by constructor)
                    nodes::AStarNode nextSuccessor(&nextParent, successorCoordinate);
                    // Copy node to vector
                    vSuccessors.emplace_back(nextSuccessor);
                }
            }
            // For each successor
            for (char i = 0; i < vSuccessors.size(); i++)
            {
                // If successor = goal, stop search
                if (vSuccessors[i].stNodeLocation == m_stGoalNode.stNodeLocation)
                {
                    // TODO: Build path (vector of AStarNodes) by back-tracing parents
                    return;
                }

                // Create and format lookup string
                std::string szSuccessorLookup;
                UTMCoordinateToString(vSuccessors[i].stNodeLocation, szSuccessorLookup);

                // Else compute dKg and dKf for successor if not on closed list
                // Obstacles are already filtered out of successor nodes, so no check is needed
                if (!stdClosedList.contains(szSuccessorLookup))
                {
                    // Calculate successor previous path cost
                    double dSuccessorKg = nextParent.dKg + m_dNodeSize;
                    // Calculate successor future path cost through Diagonal distance heuristic
                    // Move to helper function for readability?
                    double dGoalEastingDistance  = std::abs(vSuccessors[i].stNodeLocation.dEasting - m_stGoalNode.stNodeLocation.dEasting);
                    double dGoalNorthingDistance = std::abs(vSuccessors[i].stNodeLocation.dNorthing - m_stGoalNode.stNodeLocation.dNorthing);
                    double dSuccessorKh          = m_dNodeSize * (dGoalEastingDistance + dGoalNorthingDistance) +
                                          (m_dSqrtNodeSize - 2 * m_dNodeSize) * std::min(dGoalEastingDistance, dGoalNorthingDistance);
                    // f = g + h
                    double dSuccessorKf = dSuccessorKg + dSuccessorKh;

                    // Add node to open list if location hasn't been visited
                    UTMCoordinateToString(vSuccessors[i].stNodeLocation, szSuccessorLookup);
                    if (!stdOpenListLookup.contains(szSuccessorLookup))
                    {
                        stdOpenListLookup.emplace(std::make_pair(szSuccessorLookup, vSuccessors[i].dKf));
                        pqOpenList.emplace(vSuccessors[i]);
                    }
                    // Compare Kf value and put the node with the lower value on the open list
                    else
                    {
                        // TODO: Change priority queue to heap and manage manually since the following isn't ideal.
                        // Do we even need to do this?
                        std::vector<nodes::AStarNode> vNodesToPutBack;
                        // Extract nodes from priority queue until matching coordinate is found
                        while (vNodesToPutBack.size() < pqOpenList.size())
                        {
                            // Pop a node
                            nodes::AStarNode stCheckNode = pqOpenList.top();
                            pqOpenList.pop();
                            // Look for matching coordinate
                            if (stCheckNode.stNodeLocation.dEasting == vSuccessors[i].stNodeLocation.dEasting &&
                                stCheckNode.stNodeLocation.dNorthing == vSuccessors[i].stNodeLocation.dNorthing)
                            {
                                // Replace node if successor Kf value is lower indicating a more optimal path
                                if (stCheckNode.dKf > vSuccessors[i].dKf)
                                {
                                    pqOpenList.emplace(vSuccessors[i]);
                                    stdOpenListLookup.erase(szSuccessorLookup);
                                    stdOpenListLookup.emplace(std::make_pair(szSuccessorLookup, vSuccessors[i].dKf));
                                }
                                // Otherwise put node into vNodesToPutBack
                                else
                                {
                                    vNodesToPutBack.emplace_back(stCheckNode);
                                }
                                // No need to continue after match is found
                                break;
                            }
                            // Add non-matching nodes to vNodesToPutBack
                            else
                            {
                                vNodesToPutBack.emplace_back(stCheckNode);
                            }
                        }
                        // Put nodes back
                        while (vNodesToPutBack.size())
                        {
                            pqOpenList.emplace(vNodesToPutBack.back());
                            vNodesToPutBack.pop_back();
                        }
                    }
                    // If a node with the same position as successor is in the closed list and has a lower dKf, skip this successor
                    if (stdClosedList.contains(szSuccessorLookup))
                    {
                        if (stdClosedList[szSuccessorLookup] < vSuccessors[i].dKf)
                        {
                            continue;
                        }
                    }
                    // Otherwise add successor node to open list
                    pqOpenList.emplace(vSuccessors[i]);
                    stdOpenListLookup.emplace(std::make_pair(szSuccessorLookup, vSuccessors[i].dKf));
                }
            }    // End For(each successor)
            // Push Q to the closed list
            vClosedList.emplace_back(nextParent);
            std::string szParentLookup;
            UTMCoordinateToString(nextParent.stNodeLocation, szParentLookup);
            stdClosedList.emplace(std::make_pair(szParentLookup, nextParent.dKf));
        }    // End While(!pqOpenList.empty)
    }
}    // namespace pathplanners
