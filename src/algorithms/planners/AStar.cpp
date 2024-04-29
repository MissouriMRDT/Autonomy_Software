/******************************************************************************
 * @brief Implements the ASTAR path finder class within the pathplanners
 *      namespace.
 *
 * @file AStar.cpp
 * @author Kai Shafe (kasq5m@umsystem.edu), clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-01
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "AStar.h"
#include "../../AutonomyConstants.h"

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
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-02
     ******************************************************************************/
    void AStar::ClearObstacleData()
    {
        m_vObstacles.clear();
    }

    /******************************************************************************
     * @brief This method clears any saved obstacles in AStar, takes in a vector of
     *      sl::ObjectData objects and translates them to a UTMCoordinate and estimated
     *      size that is stored in the m_vObstacles vector.
     *
     * @param vObstacles - A vector reference containing ObjectData objects from the ZEDCam class.
     * @param bClearObstacles - T/F indicating whether or not internal obstacle data should be cleared.
     *
     * @todo Validate data being pulled from ObjectData structs.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-15
     ******************************************************************************/
    void AStar::UpdateObstacleData(const std::vector<sl::ObjectData>& vObstacles, const bool& bClearObstacles)
    {
        // Remove stale obstacle data.
        if (bClearObstacles)
        {
            ClearObstacleData();
        }
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
     * @brief This method clears any saved obstacles in AStar, takes in a vector of
     *      AStar::Obstacle and saves a copy to the m_vObstacles vector.
     *
     * @param vObstacles - A vector reference containing ObjectData objects from the ZEDCam class.
     * @param bClearObstacles - T/F indicating whether or not internal obstacle data should be cleared.
     *
     * @todo Validate data being pulled from ObjectData structs.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-15
     ******************************************************************************/
    void AStar::UpdateObstacleData(const std::vector<Obstacle>& vObstacles, const bool& bClearObstacles)
    {
        // Remove stale obstacle data.
        if (bClearObstacles)
        {
            ClearObstacleData();
        }
        // For each object in vObstacles:
        for (size_t i = 0; i < vObstacles.size(); i++)
        {
            m_vObstacles.emplace_back(vObstacles[i]);
        }
    }

    /******************************************************************************
     * @brief This method is intended to be called when a new obstacle is detected
     *      from the ZedCam to add a new obstacle to be considered in path finding.
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
     * @brief This method is intended to be called when a new obstacle is detected
     *      to add a new obstacle to be considered in path finding.
     *
     * @param stObstacle - A reference to an ObjectData struct representing the obstacle
     *                      to add.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-15
     ******************************************************************************/
    void AStar::AddObstacle(const Obstacle& stObstacle)
    {
        m_vObstacles.emplace_back(stObstacle);
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
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-15
     ******************************************************************************/
    geoops::UTMCoordinate AStar::FindNearestBoundaryPoint(const geoops::UTMCoordinate& stGoalCoordinate)
    {
        // Create return value.
        geoops::UTMCoordinate stBoundaryCoordinate = stGoalCoordinate;
        // Determine components of the distance vector formed by the current location and goal.
        const double dDeltaX         = stGoalCoordinate.dEasting - m_stStartNode.stNodeLocation.dEasting;
        const double dDeltaY         = stGoalCoordinate.dNorthing - m_stStartNode.stNodeLocation.dNorthing;
        const double dAbsoluteDeltaX = std::abs(dDeltaX);
        const double dAbsoluteDeltaY = std::abs(dDeltaY);
        short sDirection;

        // Only calculate the boundary point if the goal is not within the search grid.
        if (dAbsoluteDeltaX > constants::ASTAR_MAXIMUM_SEARCH_GRID || dAbsoluteDeltaY > constants::ASTAR_MAXIMUM_SEARCH_GRID)
        {
            // Determine which component is major.
            // If |X| is longer than |Y|.
            if (dAbsoluteDeltaX > dAbsoluteDeltaY)
            {
                // Calculate scale ratio of distance vectors (big / small).
                const double dVectorRatio = dAbsoluteDeltaX / constants::ASTAR_MAXIMUM_SEARCH_GRID;
                // Determine +/- value of major component for boundary distance vector.
                sDirection = dDeltaX / dAbsoluteDeltaX;
                // Calculate goal node X component to be the boundary value.
                stBoundaryCoordinate.dEasting = m_stStartNode.stNodeLocation.dEasting + sDirection * constants::ASTAR_MAXIMUM_SEARCH_GRID;
                // Determine +/- value of minor component for boundary distance vector.
                // Edge case of dDeltaY = 0, set sDirection to 0.
                (dDeltaY) ? sDirection = dDeltaY / dAbsoluteDeltaY : sDirection = 0;
                // Calculate goal node Y axis with scale ratio.
                stBoundaryCoordinate.dNorthing = m_stStartNode.stNodeLocation.dNorthing + sDirection * dVectorRatio * dDeltaY;
            }
            // Else if |Y| is longer than |X|.
            else if (dAbsoluteDeltaX < dAbsoluteDeltaY)
            {
                // Calculate scale ratio of distance vectors (big / small).
                const double dVectorRatio = dAbsoluteDeltaY / constants::ASTAR_MAXIMUM_SEARCH_GRID;
                // Determine +/- value of major component for boundary distance vector.
                sDirection = dDeltaY / dAbsoluteDeltaY;
                // Calculate goal node Y component to be the boundary value.
                stBoundaryCoordinate.dNorthing = m_stStartNode.stNodeLocation.dNorthing + sDirection * constants::ASTAR_MAXIMUM_SEARCH_GRID;
                // Determine +/- value of minor component for boundary distance vector.
                // Edge case of dDeltaX = 0, set sDirection to 0.
                (dDeltaX) ? sDirection = dDeltaX / dAbsoluteDeltaX : sDirection = 0;
                // Calculate goal node X axis with scale ratio.
                stBoundaryCoordinate.dEasting = m_stStartNode.stNodeLocation.dEasting + sDirection * dVectorRatio * dDeltaX;
            }
            // Else |X| = |Y|, so pick a corner.
            else
            {
                // Determine +/- value of X component.
                sDirection = dDeltaX / dAbsoluteDeltaX;
                // Calculate goal node X component to be the boundary value.
                stBoundaryCoordinate.dEasting = m_stStartNode.stNodeLocation.dEasting + sDirection * constants::ASTAR_MAXIMUM_SEARCH_GRID;
                // Determine +/- value of Y component.
                sDirection = dDeltaY / dAbsoluteDeltaY;
                // Calculate goal node Y component to be the boundary value.
                stBoundaryCoordinate.dNorthing = m_stStartNode.stNodeLocation.dNorthing + sDirection * constants::ASTAR_MAXIMUM_SEARCH_GRID;
            }
        }

        // In all cases, round the goal node's UTMCoordinate to align with grid for equality comparisons.
        stBoundaryCoordinate = RoundUTMCoordinate(stBoundaryCoordinate);

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
            if (dWestObstacleBorder < stBoundaryCoordinate.dEasting && stBoundaryCoordinate.dEasting < dEastObstacleBorder)
            {
                // Shift goal coordinate along X axis to avoid obstacle.
                if (stBoundaryCoordinate.dEasting > m_vObstacles[i].stCenterPoint.dEasting)
                {
                    stBoundaryCoordinate.dEasting = dEastObstacleBorder + constants::ASTAR_NODE_SIZE;
                }
                else
                {
                    stBoundaryCoordinate.dEasting = dWestObstacleBorder - constants::ASTAR_NODE_SIZE;
                }
                stBoundaryCoordinate = RoundUTMCoordinate(stBoundaryCoordinate);
            }

            // If goal node coordinate is within Y axis obstacle borders.
            if (dNorthObstacleBorder < m_stGoalNode.stNodeLocation.dNorthing && m_stGoalNode.stNodeLocation.dNorthing > dSouthObstacleBorder)
            {
                // Shift goal coordinate along Y axis to avoid obstacle.
                if (stBoundaryCoordinate.dNorthing > m_vObstacles[i].stCenterPoint.dNorthing)
                {
                    stBoundaryCoordinate.dNorthing = dNorthObstacleBorder + constants::ASTAR_NODE_SIZE;
                }
                else
                {
                    stBoundaryCoordinate.dNorthing = dSouthObstacleBorder - constants::ASTAR_NODE_SIZE;
                }
                stBoundaryCoordinate = RoundUTMCoordinate(stBoundaryCoordinate);
            }
        }
        // Return rounded coordinate.
        return stBoundaryCoordinate;
    }

    /******************************************************************************
     * @brief Helper function used to translate a UTMCoordinate's dEasting and dNorthing
     *      values into a string that can be hashed for the unordered_map data structure
     *      for O(1) lookup of nodes at a particular location.
     *
     * @param stToTranslate - A UTMCoordinate struct reference containing the data to translate.
     *
     * @return - A string containing the translated coordinate.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-05
     ******************************************************************************/
    std::string AStar::UTMCoordinateToString(const geoops::UTMCoordinate& stToTranslate)
    {
        std::string szTranslation = std::to_string(stToTranslate.dEasting);
        szTranslation.append(std::to_string(stToTranslate.dNorthing));
        return szTranslation;
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

        // Boundary check (Returns true if params indicate a coordinate inside of the search grid).
        if (dEasting >= (m_stStartNode.stNodeLocation.dEasting - constants::ASTAR_MAXIMUM_SEARCH_GRID - constants::ASTAR_NODE_SIZE) &&
            dEasting <= (m_stStartNode.stNodeLocation.dEasting + constants::ASTAR_MAXIMUM_SEARCH_GRID + constants::ASTAR_NODE_SIZE) &&
            dNorthing >= (m_stStartNode.stNodeLocation.dNorthing - constants::ASTAR_MAXIMUM_SEARCH_GRID - constants::ASTAR_NODE_SIZE) &&
            dNorthing <= (m_stStartNode.stNodeLocation.dNorthing + constants::ASTAR_MAXIMUM_SEARCH_GRID + constants::ASTAR_NODE_SIZE))
        {
            return true;
        }
        // Return false if boundary check failed.
        return false;
    }

    /******************************************************************************
     * @brief Helper function used to round UTMCoordinates to the nearest constants::ASTAR_NODE_SIZE to avoid
     *      rounding errors when trying to determine if two nodes have the same location.
     *
     * @param stCoordinateToRound - A UTMCoordinate reference that will have its dNorthing and dEasting values
     *                              mutated to round them to the nearest constants::ASTAR_NODE_SIZE.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-12
     ******************************************************************************/
    geoops::UTMCoordinate AStar::RoundUTMCoordinate(const geoops::UTMCoordinate& stCoordinateToRound)
    {
        geoops::UTMCoordinate stRounded = geoops::UTMCoordinate(stCoordinateToRound);
        stRounded.dEasting              = std::round(stCoordinateToRound.dEasting / constants::ASTAR_NODE_SIZE) * constants::ASTAR_NODE_SIZE;
        stRounded.dNorthing             = std::round(stCoordinateToRound.dNorthing / constants::ASTAR_NODE_SIZE) * constants::ASTAR_NODE_SIZE;
        return stRounded;
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
        // Base case: Check for origin node.
        if (stEndNode.pParentNode == nullptr)
        {
            // Start copying
            m_vPathCoordinates.emplace_back(stEndNode.stNodeLocation);
            return;
        }
        // Recursive case: call ConstructPath on parent pointer.
        ConstructPath(*stEndNode.pParentNode);
        // Copy node UTMCoordinate data to m_vPathNodes.
        m_vPathCoordinates.emplace_back(stEndNode.stNodeLocation);
        return;
    }

    /******************************************************************************
     * @brief Called in the obstacle avoidance state to plan a path around obstacles
     *      blocking our path.
     *
     * @param stStartCoordinate - A UTMCoordinate reference that represents the start location.
     * @param stGoalCoordinate - A UTMCoordinate reference that represents the goal location.
     * @param vObstacles - A vector reference containing ObjectData objects from the ZEDCam class,
     *                      defaults to an empty vector.
     *
     * @return - A vector of UTMCoordinates representing the path calculated by ASTAR.
     *
     * @todo Build a visualizer for testing.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-02
     ******************************************************************************/
    std::vector<geoops::UTMCoordinate> AStar::PlanAvoidancePath(const geoops::UTMCoordinate& stStartCoordinate,
                                                                const geoops::UTMCoordinate& stGoalCoordinate,
                                                                const std::vector<sl::ObjectData>& vObstacles)
    {
        // Translate Object data from camera and construct obstacle nodes.
        // Stores Data in m_vObstacles.
        UpdateObstacleData(vObstacles);

        // Create start node.
        m_stStartNode = nodes::AStarNode(nullptr, stStartCoordinate);
        // Map the goalLocation to an edge node based on maximum search size.
        geoops::UTMCoordinate stRoundedGoal(FindNearestBoundaryPoint(stGoalCoordinate));
        // Create goal node.
        m_stGoalNode = nodes::AStarNode(nullptr, stRoundedGoal);

        // -------------------A* algorithm-------------------
        // Create Open and Closed Lists.
        // Using an additional unordered map is memory inefficient but allows for O(1)
        //  lookup of nodes based on their position rather than iterating over the heap.
        // Carefully manage nodes between 'lists' to ensure data is consistent.

        // Open list implemented as a min-heap queue for O(1) retrieval of the node with min dKf value.
        // C++ utilizes the '*_heap' family of functions which operate on vectors.
        std::vector<nodes::AStarNode> vOpenList;
        std::make_heap(vOpenList.begin(), vOpenList.end(), std::greater<nodes::AStarNode>());
        // Unordered map of coordinates for open list for O(1) lookup.
        std::unordered_map<std::string, double> umOpenListLookup;

        // Vector containing pointers to nodes on the closed list.
        // This vector also contains the nodes that will be copied to m_vPathCoordinates.
        std::vector<std::shared_ptr<nodes::AStarNode>> vClosedList;

        // Unordered map of coordinates for closed list for O(1) lookup.
        std::unordered_map<std::string, double> umClosedList;

        // Place Starting node on open list.
        vOpenList.push_back(m_stStartNode);
        // Translate start node to string and add location on open list lookup map.
        std::string szLocationString = UTMCoordinateToString(m_stStartNode.stNodeLocation);
        umOpenListLookup.emplace(std::make_pair(szLocationString, 0.0));

        // While open list is not empty:
        while (!vOpenList.empty())
        {
            // Retrieve node with the minimum dKf on open list (Q).
            std::pop_heap(vOpenList.begin(), vOpenList.end(), std::greater<nodes::AStarNode>());
            nodes::AStarNode stNextParent = vOpenList.back();
            // Pop Q off open list.
            vOpenList.pop_back();
            // Put Q on closed list to allocate parent pointers of successors.
            // Note: make_shared creates a copy of stNextParent on the heap, and points to that copy.
            vClosedList.push_back(std::make_shared<nodes::AStarNode>(stNextParent));

            // Generate Q's 8 successors (neighbors), setting parent to Q.
            std::vector<nodes::AStarNode> vSuccessors;
            double dWestOffset  = stNextParent.stNodeLocation.dEasting - constants::ASTAR_NODE_SIZE;
            double dEastOffset  = stNextParent.stNodeLocation.dEasting + constants::ASTAR_NODE_SIZE;
            double dSouthOffset = stNextParent.stNodeLocation.dNorthing - constants::ASTAR_NODE_SIZE;
            double dNorthOffset = stNextParent.stNodeLocation.dNorthing + constants::ASTAR_NODE_SIZE;

            // Counter for avoiding parent duplication.
            ushort usSuccessorTracker = 0;
            for (double dEastingOffset = dWestOffset; dEastingOffset <= dEastOffset; dEastingOffset += constants::ASTAR_NODE_SIZE)
            {
                for (double dNorthingOffset = dSouthOffset; dNorthingOffset <= dNorthOffset; dNorthingOffset += constants::ASTAR_NODE_SIZE)
                {
                    // Skip duplicating the parent node.
                    // Implemented with a counter to avoid evaluating coordinates.
                    usSuccessorTracker++;
                    if (usSuccessorTracker == 5)
                    {
                        continue;
                    }

                    // Check for valid coordinate (check for boundary and obstacles).
                    if (!ValidCoordinate(dEastingOffset, dNorthingOffset))
                    {
                        continue;
                    }

                    // Otherwise create the successor.
                    // Copy data from parent coordinate.
                    geoops::UTMCoordinate stSuccessorCoordinate(stNextParent.stNodeLocation);

                    // Adjust Easting and Northing offsets to create new coordinate.
                    stSuccessorCoordinate.dEasting  = dEastingOffset;
                    stSuccessorCoordinate.dNorthing = dNorthingOffset;
                    RoundUTMCoordinate(stSuccessorCoordinate);
                    // Create successor node, initialize values to 0 (done by constructor).
                    nodes::AStarNode stNextSuccessor(vClosedList.back(), stSuccessorCoordinate);
                    // Copy successor node to vector.
                    vSuccessors.emplace_back(stNextSuccessor);
                }
            }

            // For each successor:
            for (size_t i = 0; i < vSuccessors.size(); i++)
            {
                // If successor distance to goal is less than the node size, stop search.
                if (geoops::CalculateGeoMeasurement(vSuccessors[i].stNodeLocation, m_stGoalNode.stNodeLocation).dDistanceMeters < constants::ASTAR_NODE_SIZE)
                {
                    ConstructPath(vSuccessors[i]);
                    return m_vPathCoordinates;
                }

                // Create and format lookup string.
                std::string szSuccessorLookup = UTMCoordinateToString(vSuccessors[i].stNodeLocation);

                // Compute dKg, dKh, and dKf for successor.
                // Calculate successor previous path cost.
                vSuccessors[i].dKg = stNextParent.dKg + constants::ASTAR_NODE_SIZE;
                // Calculate successor future path cost through geo measurement.
                vSuccessors[i].dKh = geoops::CalculateGeoMeasurement(vSuccessors[i].stNodeLocation, m_stGoalNode.stNodeLocation).dDistanceMeters;
                // f = g + h
                vSuccessors[i].dKf = vSuccessors[i].dKg + vSuccessors[i].dKh;

                // If a node with the same position as successor is in the open list and has a lower dKf, skip this successor.
                if (umOpenListLookup.count(szSuccessorLookup))
                {
                    if (umOpenListLookup[szSuccessorLookup] <= vSuccessors[i].dKf)
                    {
                        continue;
                    }
                }

                // If a node with the same position as successor is in the closed list and has a lower dKf, skip this successor.
                if (umClosedList.count(szSuccessorLookup))
                {
                    if (umClosedList[szSuccessorLookup] <= vSuccessors[i].dKf)
                    {
                        continue;
                    }
                }

                // Otherwise add successor node to open list.
                // Add lookup string and dKf value to lookup map.
                umOpenListLookup.emplace(std::make_pair(szSuccessorLookup, vSuccessors[i].dKf));
                // Push to heap.
                vOpenList.push_back(vSuccessors[i]);
                std::push_heap(vOpenList.begin(), vOpenList.end(), std::greater<nodes::AStarNode>());
            }    // End For (each successor).

            // Create and format lookup string.
            std::string szParentLookup = UTMCoordinateToString(stNextParent.stNodeLocation);
            // Push lookup string and dKf value to lookup map.
            umClosedList.emplace(std::make_pair(szParentLookup, stNextParent.dKf));
        }    // End While(!vOpenList.empty).

        // Function has failed to find a valid path.
        LOG_ERROR(logging::g_qSharedLogger,
                  "ASTAR Failed to find a path from UTM point ({}, {}) to UTM point ({}, {})",
                  m_stStartNode.stNodeLocation.dEasting,
                  m_stStartNode.stNodeLocation.dNorthing,
                  m_stGoalNode.stNodeLocation.dEasting,
                  m_stGoalNode.stNodeLocation.dNorthing);
        // Return empty vector and handle outside of class.
        return m_vPathCoordinates;
    }
}    // namespace pathplanners
