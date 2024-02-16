/******************************************************************************
 * @brief Implements the ASTAR path finder class within the pathplanners
 *      namespace.
 *
 * @file AStar.cpp
 * @author Kai Shafe (kasq5m@umsystem.edu)
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
     * @todo Determine if this needs to be public for error handling.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-02
     ******************************************************************************/
    void AStar::ClearObstacleData()
    {
        m_vObstacles.clear();
    }

    /******************************************************************************
     * @brief Intended to be called as a helper function by the PlanAvoidancePath method.
     *      This method takes in a vector of ObjectData objects and translates them to
     *      a UTMCoordinate and estimated size that is stored in the m_vObstacles vector.
     *
     * @param vObstacles - A vector reference containing ObjectData objects from the ZEDCam class.
     *
     * @todo Validate data being pulled from ObjectData structs.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-15
     ******************************************************************************/
    void AStar::UpdateObstacleData(const std::vector<sl::ObjectData>& vObstacles)
    {
        // Remove stale obstacle data.
        ClearObstacleData();
        // For each object in vObstacles:
        for (size_t i = 0; i < vObstacles.size(); i++)
        {
            // Create Obstacle struct.
            Obstacle stObstacleToAdd;
            // Extract coordinate data from ObjectData struct.
            stObstacleToAdd.stCenterPoint.dEasting  = vObstacles[i].position.x;
            stObstacleToAdd.stCenterPoint.dNorthing = vObstacles[i].position.y;
            // Extract size data from ObjectData and calculate size of obstacle.
            // Assuming worst case scenario and calculating the maximum diagonal as object radius, optimize later?
            stObstacleToAdd.fRadius = std::sqrt(std::pow(vObstacles[i].dimensions.x, 2) + std::pow(vObstacles[i].dimensions.y, 2));
            // Copy Obstacle data to m_vObstacles for use in PlanAvoidancePath().
            m_vObstacles.emplace_back(stObstacleToAdd);
        }
    }

    /******************************************************************************
     * @brief This method is intended to be called when a new obstacle is detected
     *          from the ZedCam to add a new obstacle to be considered in path finding.
     *
     * @param stObstacle - A reference to an ObjectData struct representing the obstacle
     *                      to add.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-15
     ******************************************************************************/
    void AStar::AddObstacle(const sl::ObjectData& stObstacle)
    {
        // Create Obstacle struct.
        Obstacle stObstacleToAdd;
        // Extract coordinate data from ObjectData struct.
        stObstacleToAdd.stCenterPoint.dEasting  = stObstacle.position.x;
        stObstacleToAdd.stCenterPoint.dNorthing = stObstacle.position.y;
        // Extract size data from ObjectData and calculate size of obstacle.
        // Assuming worst case scenario and calculating the maximum diagonal as object radius, optimize later?
        stObstacleToAdd.fRadius = std::sqrt(std::pow(stObstacle.dimensions.x, 2) + std::pow(stObstacle.dimensions.y, 2));
        // Copy Obstacle data to m_vObstacles for use in PlanAvoidancePath().
        m_vObstacles.emplace_back(stObstacleToAdd);
    }

    /******************************************************************************
     * @brief Helper function for the PlanAvoidancePath method. This method takes in
     *      a UTMCoordinate reference and uses class member variables to mutate the
     *      m_stGoalNode object's coordinates to represent the nearest boundary point.
     *
     * @pre - m_stStartNode has been initialized with a UTMCoordinate representing the
     *          rover's current location.
     *
     * @param stGoalCoordinate - UTMCoordinate reference representing the current rover destination.
     *
     * @todo Test me!
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-015
     ******************************************************************************/
    void AStar::FindNearestBoundaryPoint(const geoops::UTMCoordinate& stGoalCoordinate)
    {
        // Determine components of the distance vector formed by the current location and goal.
        const double dDeltaX         = stGoalCoordinate.dEasting - m_stStartNode.stNodeLocation.dEasting;
        const double dDeltaY         = stGoalCoordinate.dNorthing - m_stStartNode.stNodeLocation.dNorthing;
        const double dAbsoluteDeltaX = std::abs(dDeltaX);
        const double dAbsoluteDeltaY = std::abs(dDeltaY);
        short sDirection;
        // Determine which component is major.
        // If |X| is longer than |Y|.
        if (dAbsoluteDeltaX > dAbsoluteDeltaY)
        {
            // Calculate scale ratio of distance vectors (big / small).
            const double dVectorRatio = dAbsoluteDeltaX / constants::ASTAR_MAXIMUM_SEARCH_GRID;
            // Determine +/- value of major component for boundary distance vector.
            sDirection = dDeltaX / dAbsoluteDeltaX;
            // Calculate goal node X component to be the boundary value.
            m_stGoalNode.stNodeLocation.dEasting = m_stStartNode.stNodeLocation.dEasting + sDirection * constants::ASTAR_MAXIMUM_SEARCH_GRID;
            // Determine +/- value of minor component for boundary distance vector.
            // Edge case of dDeltaY = 0, set sDirection to 0.
            (dDeltaY) ? sDirection = dDeltaY / dAbsoluteDeltaY : sDirection = 0;
            // Calculate goal node Y axis with scale ratio.
            m_stGoalNode.stNodeLocation.dNorthing = m_stStartNode.stNodeLocation.dNorthing + sDirection * dVectorRatio * dDeltaY;
        }
        // Else if |Y| is longer than |X|.
        else if (dAbsoluteDeltaX < dAbsoluteDeltaY)
        {
            // Calculate scale ratio of distance vectors (big / small).
            const double dVectorRatio = dAbsoluteDeltaY / constants::ASTAR_MAXIMUM_SEARCH_GRID;
            // Determine +/- value of major component for boundary distance vector.
            sDirection = dDeltaY / dAbsoluteDeltaY;
            // Calculate goal node Y component to be the boundary value.
            m_stGoalNode.stNodeLocation.dNorthing = m_stStartNode.stNodeLocation.dNorthing + sDirection * constants::ASTAR_MAXIMUM_SEARCH_GRID;
            // Determine +/- value of minor component for boundary distance vector.
            // Edge case of dDeltaX = 0, set sDirection to 0.
            (dDeltaX) ? sDirection = dDeltaX / dAbsoluteDeltaX : sDirection = 0;
            // Calculate goal node X axis with scale ratio.
            m_stGoalNode.stNodeLocation.dEasting = m_stStartNode.stNodeLocation.dEasting + sDirection * dVectorRatio * dDeltaX;
        }
        // Else |X| = |Y|, so pick a corner.
        else
        {
            // Determine +/- value of X component.
            sDirection = dDeltaX / dAbsoluteDeltaX;
            // Calculate goal node X component to be the boundary value.
            m_stGoalNode.stNodeLocation.dEasting = m_stStartNode.stNodeLocation.dEasting + sDirection * constants::ASTAR_MAXIMUM_SEARCH_GRID;
            // Determine +/- value of Y component.
            sDirection = dDeltaY / dAbsoluteDeltaY;
            // Calculate goal node Y component to be the boundary value.
            m_stGoalNode.stNodeLocation.dNorthing = m_stStartNode.stNodeLocation.dNorthing + sDirection * constants::ASTAR_MAXIMUM_SEARCH_GRID;
        }
        // In all cases, round the goal node's UTMCoordinate to align with grid for equality comparisons.
        RoundUTMCoordinate(m_stGoalNode.stNodeLocation);

        // Handle edge case of an obstacle blocking the goal coordinate.
        // For each obstacle:
        for (size_t i = 0; i < m_vObstacles.size(); i++)
        {
            // Multiplier for avoidance radius.
            double dAvoidanceRadius = constants::ASTAR_AVOIDANCE_MULTIPLIER * m_vObstacles[i].fRadius;
            // Create obstacle borders.
            double dEastObstacleBorder  = m_vObstacles[i].stCenterPoint.dEasting + dAvoidanceRadius;
            double dWestObstacleBorder  = m_vObstacles[i].stCenterPoint.dEasting - dAvoidanceRadius;
            double dNorthObstacleBorder = m_vObstacles[i].stCenterPoint.dNorthing + dAvoidanceRadius;
            double dSouthObstacleBorder = m_vObstacles[i].stCenterPoint.dNorthing - dAvoidanceRadius;

            // If goal node coordinate is within X axis obstacle borders.
            if (dWestObstacleBorder < m_stGoalNode.stNodeLocation.dEasting && m_stGoalNode.stNodeLocation.dEasting < dEastObstacleBorder)
            {
                {
                    // Shift goal coordinate along X axis to avoid obstacle.
                    if (m_stGoalNode.stNodeLocation.dEasting > m_vObstacles[i].stCenterPoint.dEasting)
                    {
                        m_stGoalNode.stNodeLocation.dEasting = dEastObstacleBorder + constants::ASTAR_NODE_SIZE;
                    }
                    else
                    {
                        m_stGoalNode.stNodeLocation.dEasting = dWestObstacleBorder - constants::ASTAR_NODE_SIZE;
                    }
                    RoundUTMCoordinate(m_stGoalNode.stNodeLocation);
                }
                // If goal node coordinate is within Y axis obstacle borders.
                if (dNorthObstacleBorder < m_stGoalNode.stNodeLocation.dNorthing && m_stGoalNode.stNodeLocation.dNorthing > dSouthObstacleBorder)
                {
                    // Shift goal coordinate along Y axis to avoid obstacle.
                    if (m_stGoalNode.stNodeLocation.dNorthing > m_vObstacles[i].stCenterPoint.dNorthing)
                    {
                        m_stGoalNode.stNodeLocation.dNorthing = dNorthObstacleBorder + constants::ASTAR_NODE_SIZE;
                    }
                    else
                    {
                        m_stGoalNode.stNodeLocation.dNorthing = dSouthObstacleBorder - constants::ASTAR_NODE_SIZE;
                    }
                    RoundUTMCoordinate(m_stGoalNode.stNodeLocation);
                }
            }
        }
    }

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
     * @todo Test me!
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-06
     ******************************************************************************/
    bool AStar::ValidCoordinate(const double& dEasting, const double& dNorthing)
    {
        // Boundary check (Figure out how to override auto-format).
        if (dEasting < (dEasting - constants::ASTAR_MAXIMUM_SEARCH_GRID) || dEasting > (dEasting + constants::ASTAR_MAXIMUM_SEARCH_GRID) ||
            dNorthing < (dNorthing - constants::ASTAR_MAXIMUM_SEARCH_GRID) || dNorthing < (dNorthing + constants::ASTAR_MAXIMUM_SEARCH_GRID))
        {
            return false;
        }
        // For each obstacle.
        for (size_t i = 0; i < m_vObstacles.size(); i++)
        {
            // Multiplier for avoidance radius.
            double dAvoidanceRadius = constants::ASTAR_AVOIDANCE_MULTIPLIER * m_vObstacles[i].fRadius;
            // Create obstacle borders.
            double dEastObstacleBorder  = m_vObstacles[i].stCenterPoint.dEasting + dAvoidanceRadius;
            double dWestObstacleBorder  = m_vObstacles[i].stCenterPoint.dEasting - dAvoidanceRadius;
            double dNorthObstacleBorder = m_vObstacles[i].stCenterPoint.dNorthing + dAvoidanceRadius;
            double dSouthObstacleBorder = m_vObstacles[i].stCenterPoint.dNorthing - dAvoidanceRadius;

            // Return false if node is within obstacle borders.
            if (dWestObstacleBorder < dEasting && dEasting < dEastObstacleBorder && dNorthObstacleBorder < dNorthing && dNorthing > dSouthObstacleBorder)
            {
                return false;
            }
        }
        return true;
    }

    /******************************************************************************
     * @brief Helper function used to round UTMCoordinates to the nearest constants::ASTAR_NODE_SIZE to avoid
     *      rounding errors when trying to determine if two nodes have the same location.
     *
     * @param stCoordinateToRound - A UTMCoordinate reference that will have its dNorthing and dEasting values
     *                              mutated to round them to the nearest constants::ASTAR_NODE_SIZE.
     *
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-12
     ******************************************************************************/
    void AStar::RoundUTMCoordinate(geoops::UTMCoordinate& stRoundCoordinate)
    {
        stRoundCoordinate.dEasting  = std::round(stRoundCoordinate.dEasting / constants::ASTAR_NODE_SIZE) * constants::ASTAR_NODE_SIZE;
        stRoundCoordinate.dNorthing = std::round(stRoundCoordinate.dNorthing / constants::ASTAR_NODE_SIZE) * constants::ASTAR_NODE_SIZE;
    }

    /******************************************************************************
     * @brief Helper function used to calculate ASTAR node heuristic distance value.
     *          This implementation uses the diagonal distance heuristic for efficiency.
     *          (Mostly implemented to clean up the PlanAvoidanceRoute method).
     *
     * @pre - Assumes stNodeToCalculate's and m_stGoalNode's UTMCoordinates have been initialized with the
     *          node's location and the nearest boundary point's coordinate respectively.
     *          .
     * @param stNodeToCalculate - A const AStarNode reference.
     *
     * @return - Returns a double representing the Kh value for stNodeToCalculate.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-12
     ******************************************************************************/
    double AStar::CalculateNodeHValue(const nodes::AStarNode& stNodeToCalculate)
    {
        double dGoalEastingDistance  = std::abs(stNodeToCalculate.stNodeLocation.dEasting - m_stGoalNode.stNodeLocation.dEasting);
        double dGoalNorthingDistance = std::abs(stNodeToCalculate.stNodeLocation.dNorthing - m_stGoalNode.stNodeLocation.dNorthing);
        return constants::ASTAR_NODE_SIZE * (dGoalEastingDistance + dGoalNorthingDistance) +
               (constants::ASTAR_SQRT_NODE_SIZE - 2 * constants::ASTAR_NODE_SIZE) * std::min(dGoalEastingDistance, dGoalNorthingDistance);
    }

    /******************************************************************************
     * @brief Called when a goal node has been reached. Recursively builds a vector of
     *          UTMCoordinates by tracing the parent pointers of AStarNodes.
     *          This function then saves that vector to m_vPathNodes.
     *
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-13
     ******************************************************************************/
    void AStar::ConstructPath(const nodes::AStarNode& stEndNode)
    {
        // Copy node UTMCoordinate data to m_vPathNodes.
        m_vPathCoordinates.emplace_back(stEndNode.stNodeLocation);
        // Base case: Check for origin node.
        if (stEndNode.stParentNode == nullptr)
        {
            return;
        }
        // Recursive case: call ConstructPath on parent pointer.
        ConstructPath(*stEndNode.stParentNode);
        return;
    }

    /******************************************************************************
     * @brief Called in the obstacle avoidance state to plan a path around obstacles
     *      blocking our path.
     *
     * @param vObstacles - A vector reference containing ObjectData objects from the ZEDCam class.
     *
     * @return - A vector of UTMCoordinates representing the path calculated by ASTAR.
     *
     * @todo Build a visualizer for testing.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-02
     ******************************************************************************/
    std::vector<geoops::UTMCoordinate> AStar::PlanAvoidancePath(const std::vector<sl::ObjectData>& vObstacles)
    {
        // Translate Object data from camera and construct obstacle nodes.
        // Stores Data in m_vObstacles.
        UpdateObstacleData(vObstacles);

        // Create Start and Goal nodes.
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
            double dWestOffset  = nextParent.stNodeLocation.dEasting - constants::ASTAR_NODE_SIZE;
            double dEastOffset  = nextParent.stNodeLocation.dEasting + constants::ASTAR_NODE_SIZE;
            double dSouthOffset = nextParent.stNodeLocation.dNorthing - constants::ASTAR_NODE_SIZE;
            double dNorthOffset = nextParent.stNodeLocation.dNorthing + constants::ASTAR_NODE_SIZE;
            // Counter for avoiding parent duplication
            ushort usParentTracker = 0;
            for (double dEastingOffset = dWestOffset; dEastingOffset < dEastOffset; dEastingOffset += constants::ASTAR_NODE_SIZE)
            {
                for (double dNorthingOffset = dSouthOffset; dNorthingOffset < dNorthOffset; dNorthingOffset += constants::ASTAR_NODE_SIZE)
                {
                    // Skip duplicating the parent node.
                    // Implemented with a counter to avoid evaluating coordinates.
                    usParentTracker++;
                    if (usParentTracker == 5)
                    {
                        continue;
                    }
                    // Check for valid coordinate (check for boundary and obstacles).
                    if (!ValidCoordinate(dEastingOffset, dNorthingOffset))
                    {
                        continue;
                    }
                    // Copy data from parent coordinate.
                    geoops::UTMCoordinate successorCoordinate = nextParent.stNodeLocation;
                    // Adjust Easting and Northing offsets to create new coordinate.
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
            for (size_t i = 0; i < vSuccessors.size(); i++)
            {
                // If successor = goal, stop search.
                if (vSuccessors[i] == m_stGoalNode)
                {
                    ConstructPath(vSuccessors[i]);
                    return m_vPathCoordinates;
                }

                // Create and format lookup string.
                std::string szSuccessorLookup;
                UTMCoordinateToString(vSuccessors[i].stNodeLocation, szSuccessorLookup);

                // Compute dKg, dKh, and dKf for successor.
                // Calculate successor previous path cost.
                vSuccessors[i].dKg = nextParent.dKg + constants::ASTAR_NODE_SIZE;
                // Calculate successor future path cost through diagonal distance heuristic.
                vSuccessors[i].dKh = CalculateNodeHValue(vSuccessors[i]);
                // f = g + h
                vSuccessors[i].dKf = vSuccessors[i].dKg + vSuccessors[i].dKh;

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
