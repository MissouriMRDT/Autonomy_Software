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
     * @author Sam Nolte (samnolte0302@gmail.com)
     * @date 2024-11-18
     ******************************************************************************/
    AStar::AStar()
    {
        // Initialize Member
        m_vPathCoordinates = std::vector<geoops::UTMCoordinate>();
        m_vObstacles       = std::vector<Obstacle>();
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
     *
     * @param stStartCoordinate - A Waypoint reference that represents the start location.
     * @param stGoalCoordinate - A Waypoint reference that represents the goal location.
     * @return std::vector<geoops::Waypoint> - A vector of Waypoints representing the path calculated by ASTAR.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-21
     ******************************************************************************/
    std::vector<geoops::Waypoint> AStar::PlanAvoidancePath(const geoops::Waypoint& stStartCoordinate, const geoops::Waypoint& stGoalCoordinate)
    {
        // Call the UTMCoordinate version of PlanAvoidancePath.
        return PlanAvoidancePath(stStartCoordinate.GetUTMCoordinate(), stGoalCoordinate.GetUTMCoordinate());
    }

    /******************************************************************************
     * @brief Called in the obstacle avoidance state to plan a path around obstacles
     *      blocking our path.
     *
     * @param stStartCoordinate - A UTMCoordinate reference that represents the start location.
     * @param stGoalCoordinate - A UTMCoordinate reference that represents the goal location.
     * @return - A vector of Waypoints representing the path calculated by ASTAR.
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2024-02-02
     ******************************************************************************/
    std::vector<geoops::Waypoint> AStar::PlanAvoidancePath(const geoops::UTMCoordinate& stStartCoordinate, const geoops::UTMCoordinate& stGoalCoordinate)
    {
        // Clear previous path data.
        m_vPathCoordinates.clear();
        // Reset path generation cancellation flag.
        m_bPathGenerationCancelled = false;

        // Submit log message.
        LOG_NOTICE(logging::g_qSharedLogger,
                   "ASTAR has started planning a path up to {} meters long with a node spacing of {} meters.",
                   constants::ASTAR_MAX_SEARCH_GRID,
                   constants::ASTAR_NODE_SIZE);
        // Update path plan start time.
        m_tmStartTime = std::chrono::steady_clock::now();

        // Map the start location to a nearby point that isn't overlapping with an obstacle.
        geoops::Waypoint stRoundedStart(FindNearestStartPoint(stStartCoordinate));
        // Create start node.
        m_stStartNode = nodes::AStarNode(nullptr, stRoundedStart.GetUTMCoordinate());
        // Map the goalLocation to an edge node based on maximum search size.
        geoops::Waypoint stRoundedGoal(FindNearestGoalPoint(stGoalCoordinate));
        // Create goal node.
        m_stGoalNode = nodes::AStarNode(nullptr, stRoundedGoal.GetUTMCoordinate());

        // -------------------A* algorithm-------------------
        // Create Open and Closed Lists.
        // Using an additional unordered map is memory inefficient but allows for O(1)
        // lookup of nodes based on their position rather than iterating over the heap.
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
            // Check if path generation has been cancelled.
            if (m_bPathGenerationCancelled)
            {
                // Submit log message.
                LOG_WARNING(logging::g_qSharedLogger, "ASTAR path generation has been cancelled.");
                // Clear path coordinates.
                m_vPathCoordinates.clear();
                // Return empty path.
                return m_vPathCoordinates;
            }

            // Check if we have exceeded the maximum search time.
            std::chrono::steady_clock::time_point tmCurrentTime = std::chrono::steady_clock::now();
            std::chrono::duration<double> dElapsedTime          = std::chrono::duration_cast<std::chrono::duration<double>>(tmCurrentTime - m_tmStartTime);
            if (dElapsedTime.count() > constants::ASTAR_MAX_SEARCH_TIME)
            {
                // Submit log message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "ASTAR has exceeded the maximum search time of {} seconds. Path planning has been aborted.",
                            constants::ASTAR_MAX_SEARCH_TIME);
                // Return empty path.
                return m_vPathCoordinates;
            }

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

            // Counter for avoiding parent duplication.
            for (int nEastingDirection = -1; nEastingDirection <= 1; nEastingDirection += 1)
            {
                for (int nNorthingDirection = -1; nNorthingDirection <= 1; nNorthingDirection += 1)
                {
                    // Skip parent node.
                    if (nEastingDirection == 0 && nNorthingDirection == 0)
                    {
                        continue;
                    }

                    // Calculate successor coordinates.
                    double dSuccessorEasting  = stNextParent.stNodeLocation.dEasting + (nEastingDirection * constants::ASTAR_NODE_SIZE);
                    double dSuccessorNorthing = stNextParent.stNodeLocation.dNorthing + (nNorthingDirection * constants::ASTAR_NODE_SIZE);
                    // Check for valid coordinate (check for boundary and obstacles).
                    if (!ValidCoordinate(dSuccessorEasting, dSuccessorNorthing))
                    {
                        continue;
                    }

                    // Copy data from parent coordinate.
                    geoops::UTMCoordinate stSuccessorCoordinate = stNextParent.stNodeLocation;

                    // Adjust Easting and Northing offsets to create new coordinate.
                    stSuccessorCoordinate.dEasting  = dSuccessorEasting;
                    stSuccessorCoordinate.dNorthing = dSuccessorNorthing;
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
                // Vars for distance evaluation.
                bool bAtGoal          = false;
                double dDeltaEasting  = 0;
                double dDeltaNorthing = 0;

                // If successor distance to goal is less than the node size, stop search.
                // Try to calculate GeoMeasurement:
                geoops::GeoMeasurement stDistanceToGoal = geoops::CalculateGeoMeasurement(vSuccessors[i].stNodeLocation, m_stGoalNode.stNodeLocation);
                bool bGeoSuccess                        = stDistanceToGoal.dDistanceMeters > 0.01;

                // If this succeeds, use the GeoMeasurement distance.
                if (bGeoSuccess)
                {
                    // Round the calculated distance to the nearest half meter.
                    stDistanceToGoal.dDistanceMeters = std::round(stDistanceToGoal.dDistanceMeters * 2) / 2;
                    bAtGoal                          = stDistanceToGoal.dDistanceMeters < constants::ASTAR_NODE_SIZE;
                }
                // Otherwise manually check for goal boundaries:
                else
                {
                    dDeltaEasting  = std::abs(vSuccessors[i].stNodeLocation.dEasting - m_stGoalNode.stNodeLocation.dEasting);
                    dDeltaNorthing = std::abs(vSuccessors[i].stNodeLocation.dNorthing - m_stGoalNode.stNodeLocation.dNorthing);
                    bAtGoal        = dDeltaEasting < constants::ASTAR_NODE_SIZE && dDeltaNorthing < constants::ASTAR_NODE_SIZE;
                }

                // Construct and return path if we have reached the goal.
                if (bAtGoal)
                {
                    // Construct path from goal node.
                    ConstructPath(vSuccessors[i]);
                    // Calculate elapsed time.
                    std::chrono::steady_clock::time_point tmEndTime = std::chrono::steady_clock::now();
                    std::chrono::duration<double> dElapsedTime      = std::chrono::duration_cast<std::chrono::duration<double>>(tmEndTime - m_tmStartTime);
                    // Submit log message.
                    LOG_NOTICE(logging::g_qSharedLogger,
                               "ASTAR has successfully planned a path from UTM point ({}, {}) to UTM point ({}, {}) in {} seconds.",
                               m_stStartNode.stNodeLocation.dEasting,
                               m_stStartNode.stNodeLocation.dNorthing,
                               m_stGoalNode.stNodeLocation.dEasting,
                               m_stGoalNode.stNodeLocation.dNorthing,
                               dElapsedTime.count());
                    // Return path.
                    return m_vPathCoordinates;
                }

                // Create and format lookup string.
                std::string szSuccessorLookup = UTMCoordinateToString(vSuccessors[i].stNodeLocation);

                // Compute dKg, dKh, and dKf for successor.
                // Calculate successor previous path cost.
                vSuccessors[i].dKg = stNextParent.dKg + constants::ASTAR_NODE_SIZE;

                // Calculate successor future path cost through geo measurement if successful:
                if (bGeoSuccess)
                {
                    vSuccessors[i].dKh = stDistanceToGoal.dDistanceMeters;
                }
                // Otherwise calculate euclidean distance manually.
                else
                {
                    vSuccessors[i].dKh = std::sqrt(std::pow(dDeltaEasting, 2) + std::pow(dDeltaNorthing, 2));
                }

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

        // Calculate elapsed time.
        std::chrono::steady_clock::time_point tmEndTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> dElapsedTime      = std::chrono::duration_cast<std::chrono::duration<double>>(tmEndTime - m_tmStartTime);

        // Function has failed to find a valid path.
        LOG_ERROR(logging::g_qSharedLogger,
                  "After {} seconds, ASTAR Failed to find a path from UTM point ({}, {}) to UTM point ({}, {})",
                  dElapsedTime.count(),
                  m_stStartNode.stNodeLocation.dEasting,
                  m_stStartNode.stNodeLocation.dNorthing,
                  m_stGoalNode.stNodeLocation.dEasting,
                  m_stGoalNode.stNodeLocation.dNorthing);
        // Return empty vector and handle outside of class.
        return m_vPathCoordinates;
    }

    /******************************************************************************
     * @brief Called in the obstacle avoidance state to plan a path around obstacles
     *
     * @param stStartCoordinate - A GPSCoordinate reference that represents the start location.
     * @param stGoalCoordinate - A GPSCoordinate reference that represents the goal location.
     * @return std::vector<geoops::Waypoint> - A vector of Waypoints representing the path calculated by ASTAR.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-21
     ******************************************************************************/
    std::vector<geoops::Waypoint> AStar::PlanAvoidancePath(const geoops::GPSCoordinate& stStartCoordinate, const geoops::GPSCoordinate& stGoalCoordinate)
    {
        // Convert the GPS coordinates to UTM coordinates.
        geoops::UTMCoordinate stStartUTM = geoops::ConvertGPSToUTM(stStartCoordinate);
        geoops::UTMCoordinate stGoalUTM  = geoops::ConvertGPSToUTM(stGoalCoordinate);

        // Call the UTMCoordinate version of PlanAvoidancePath.
        return PlanAvoidancePath(stStartUTM, stGoalUTM);
    }

    /******************************************************************************
     * @brief Cancels the path generation process.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-21
     ******************************************************************************/
    void AStar::CancelPathGeneration()
    {
        m_bPathGenerationCancelled = true;
    }

    /******************************************************************************
     * @brief Adds new obstacle data to the class member variable m_vObstacles.
     *
     * @param stObstacle - A Waypoint representing the obstacle to add to the path.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-23
     ******************************************************************************/
    void AStar::UpsertObstacleData(const geoops::Waypoint& stObstacle)
    {
        // Only add obstacles if they are not already in the vector.
        std::vector<geoops::Waypoint>::iterator stdIter =
            std::find_if(m_vObstacles.begin(),
                         m_vObstacles.end(),
                         [&stObstacle](const geoops::Waypoint& stExistingObstacle) { return stExistingObstacle == stObstacle; });

        if (stdIter == m_vObstacles.end())
        {
            m_vObstacles.push_back(stObstacle);
        }
    }

    /******************************************************************************
     * @brief Adds new obstacle data to the class member variable m_vObstacles.
     *
     * @param stObstacle - A UTMCoordinate representing the obstacle to add to the path.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-23
     ******************************************************************************/
    void AStar::UpsertObstacleData(const geoops::UTMCoordinate& stObstacle)
    {
        // Only add obstacles if they are not already in the vector.
        std::vector<geoops::Waypoint>::iterator stdIter =
            std::find_if(m_vObstacles.begin(),
                         m_vObstacles.end(),
                         [&stObstacle](const geoops::Waypoint& stExistingObstacle) { return stExistingObstacle.GetUTMCoordinate() == stObstacle; });

        if (stdIter == m_vObstacles.end())
        {
            m_vObstacles.push_back(geoops::Waypoint(stObstacle));
        }
    }

    /******************************************************************************
     * @brief Adds new obstacle data to the class member variable m_vObstacles.
     *
     * @param stObstacle - A GPSCoordinate representing the obstacle to add to the path.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-23
     ******************************************************************************/
    void AStar::UpsertObstacleData(const geoops::GPSCoordinate& stObstacle)
    {
        // Convert the GPS coordinate to a UTM coordinate.
        geoops::UTMCoordinate stUTMObstacle = geoops::ConvertGPSToUTM(stObstacle);
        // Only add obstacles if they are not already in the vector.
        std::vector<geoops::Waypoint>::iterator stdIter =
            std::find_if(m_vObstacles.begin(),
                         m_vObstacles.end(),
                         [&stUTMObstacle](const geoops::Waypoint& stExistingObstacle) { return stExistingObstacle.GetUTMCoordinate() == stUTMObstacle; });

        if (stdIter == m_vObstacles.end())
        {
            m_vObstacles.push_back(geoops::Waypoint(stUTMObstacle));
        }
    }

    /******************************************************************************
     * @brief Adds new obstacle data to the class member variable m_vObstacles.
     *    Also checks if the obstacle is already in the vector and skips it if it is.
     *
     * @param vObstacles - A vector of Waypoints representing the obstacles to add to the path.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-21
     ******************************************************************************/
    void AStar::UpsertObstacleData(const std::vector<geoops::Waypoint>& vObstacles)
    {
        // Only add obstacles if they are not already in the vector.
        for (const geoops::Waypoint& stObstacle : vObstacles)
        {
            this->UpsertObstacleData(stObstacle);
        }
    }

    /******************************************************************************
     * @brief Adds new obstacle data to the class member variable m_vObstacles.
     *
     * @param vObstacles - A vector of UTMCoordinates representing the obstacles to add to the path.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-21
     ******************************************************************************/
    void AStar::UpsertObstacleData(const std::vector<geoops::UTMCoordinate>& vObstacles)
    {
        // Only add obstacles if they are not already in the vector.
        for (const geoops::UTMCoordinate& stObstacle : vObstacles)
        {
            this->UpsertObstacleData(stObstacle);
        }
    }

    /******************************************************************************
     * @brief Adds new obstacle data to the class member variable m_vObstacles.
     *
     * @param vObstacles - A vector of GPSCoordinates representing the obstacles to add to the path.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-21
     ******************************************************************************/
    void AStar::UpsertObstacleData(const std::vector<geoops::GPSCoordinate>& vObstacles)
    {
        // Only add obstacles if they are not already in the vector.
        for (const geoops::GPSCoordinate& stObstacle : vObstacles)
        {
            this->UpsertObstacleData(stObstacle);
        }
    }

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
     * @brief Getter for the path calculated by ASTAR.
     *
     * @return std::vector<geoops::UTMCoordinate> - A vector of UTMCoordinates representing the path calculated by ASTAR.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-09
     ******************************************************************************/
    std::vector<geoops::Waypoint> AStar::GetPath() const
    {
        return m_vPathCoordinates;
    }

    /******************************************************************************
     * @brief Getter for the current obstacle data.
     *
     * @return std::vector<AStar::Obstacle> - A vector of Obstacle structs representing the obstacles in the path.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-09
     ******************************************************************************/
    std::vector<geoops::Waypoint> AStar::GetObstacleData() const
    {
        return m_vObstacles;
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
    geoops::Waypoint AStar::FindNearestGoalPoint(const geoops::UTMCoordinate& stGoalCoordinate)
    {
        // Round the goal coordinate to align with the grid.
        geoops::UTMCoordinate stRoundedGoal = stGoalCoordinate;
        RoundUTMCoordinate(stRoundedGoal);

        // Create return value.
        geoops::UTMCoordinate stBoundaryCoordinate = stRoundedGoal;
        // Determine components of the distance vector formed by the current location and goal.
        const double dDeltaX = stRoundedGoal.dEasting - m_stStartNode.stNodeLocation.dEasting;
        const double dDeltaY = stRoundedGoal.dNorthing - m_stStartNode.stNodeLocation.dNorthing;

        // Only calculate the boundary point if the goal is not within the search grid.
        if (std::fabs(dDeltaX) > constants::ASTAR_MAX_SEARCH_GRID || std::fabs(dDeltaY) > constants::ASTAR_MAX_SEARCH_GRID)
        {
            // Calculate the slope of the line formed by the goal and the current location.
            const double dSlope = std::fabs(dDeltaY / dDeltaX);
            // Calculate the angle of the line formed by the goal and the current location.
            const double dAngle = std::atan(dSlope);
            // Calculate the boundary point's X and Y components.
            const double dBoundaryX = m_stStartNode.stNodeLocation.dEasting + (constants::ASTAR_MAX_SEARCH_GRID * std::cos(dAngle)) * (dDeltaX < 0 ? -1 : 1);
            const double dBoundaryY = m_stStartNode.stNodeLocation.dNorthing + (constants::ASTAR_MAX_SEARCH_GRID * std::sin(dAngle)) * (dDeltaY < 0 ? -1 : 1);
            // Set the boundary point's coordinates.
            stBoundaryCoordinate.dEasting  = dBoundaryX;
            stBoundaryCoordinate.dNorthing = dBoundaryY;

            // Submit log message.
            geoops::GeoMeasurement stMeasurement = geoops::CalculateGeoMeasurement(m_stStartNode.stNodeLocation, stBoundaryCoordinate);
            LOG_WARNING(logging::g_qSharedLogger,
                        "The goal node was adjusted from UTM point ({}, {}) to UTM point ({}, {}) to stay within the search grid of {} meters. The distance between the "
                        "original goal and the boundary point is {} meters.",
                        stRoundedGoal.dEasting,
                        stRoundedGoal.dNorthing,
                        stBoundaryCoordinate.dEasting,
                        stBoundaryCoordinate.dNorthing,
                        constants::ASTAR_MAX_SEARCH_GRID,
                        stMeasurement.dDistanceMeters);
        }

        // Handle edge case of an obstacle blocking the goal coordinate.
        bool bGoalBlocked = true;
        /*
         * This loop will check if the goal node is within the avoidance radius of any obstacle.
         * If it is, the goal node will be shifted along the X and Y axes to avoid the obstacle.
         * Then the loop will recheck all obstacles to ensure the new goal node is not blocked.
         */
        while (bGoalBlocked)
        {
            bGoalBlocked = false;
            // For each obstacle:
            for (size_t i = 0; i < m_vObstacles.size(); i++)
            {
                // Multiplier for avoidance radius.
                double dAvoidanceRadius = constants::ASTAR_AVOIDANCE_MULTIPLIER * m_vObstacles[i].dRadius;
                // Create obstacle borders.
                double dEastObstacleBorder  = m_vObstacles[i].GetUTMCoordinate().dEasting + dAvoidanceRadius;
                double dWestObstacleBorder  = m_vObstacles[i].GetUTMCoordinate().dEasting - dAvoidanceRadius;
                double dNorthObstacleBorder = m_vObstacles[i].GetUTMCoordinate().dNorthing + dAvoidanceRadius;
                double dSouthObstacleBorder = m_vObstacles[i].GetUTMCoordinate().dNorthing - dAvoidanceRadius;

                // If goal node coordinate is within obstacle borders.
                if (dWestObstacleBorder < stBoundaryCoordinate.dEasting && stBoundaryCoordinate.dEasting < dEastObstacleBorder &&
                    dSouthObstacleBorder < stBoundaryCoordinate.dNorthing && stBoundaryCoordinate.dNorthing < dNorthObstacleBorder)
                {
                    bGoalBlocked = true;
                    // Shift goal coordinate along X axis to avoid obstacle.
                    if (stBoundaryCoordinate.dEasting > m_vObstacles[i].GetUTMCoordinate().dEasting)
                    {
                        stBoundaryCoordinate.dEasting = dEastObstacleBorder + constants::ASTAR_NODE_SIZE * 2;
                    }
                    else
                    {
                        stBoundaryCoordinate.dEasting = dWestObstacleBorder - constants::ASTAR_NODE_SIZE * 2;
                    }
                    // Shift goal coordinate along Y axis to avoid obstacle.
                    if (stBoundaryCoordinate.dNorthing > m_vObstacles[i].GetUTMCoordinate().dNorthing)
                    {
                        stBoundaryCoordinate.dNorthing = dNorthObstacleBorder + constants::ASTAR_NODE_SIZE * 2;
                    }
                    else
                    {
                        stBoundaryCoordinate.dNorthing = dSouthObstacleBorder - constants::ASTAR_NODE_SIZE * 2;
                    }
                    RoundUTMCoordinate(stBoundaryCoordinate);
                    // Recheck all obstacles after adjusting the coordinate.
                    break;
                }
            }
        }

        // Check if the goal node doesn't equal the original goal node.
        if (stBoundaryCoordinate != stRoundedGoal)
        {
            // Submit log message.
            geoops::GeoMeasurement stMeasurement = geoops::CalculateGeoMeasurement(stRoundedGoal, stBoundaryCoordinate);
            LOG_WARNING(logging::g_qSharedLogger,
                        "The goal node was adjusted from UTM point ({}, {}) to UTM point ({}, {}) to avoid obstacles. The distance between the original goal and the "
                        "adjusted goal is {} meters.",
                        stRoundedGoal.dEasting,
                        stRoundedGoal.dNorthing,
                        stBoundaryCoordinate.dEasting,
                        stBoundaryCoordinate.dNorthing,
                        stMeasurement.dDistanceMeters);
        }

        // Return rounded coordinate.
        return stBoundaryCoordinate;
    }

    /******************************************************************************
     * @brief Helper function to round a UTMCoordinate to align with the grid.
     *
     * @param stStartCoordinate - A UTMCoordinate reference that represents the coordinate to round.
     * @return geoops::Waypoint - A Waypoint struct containing the rounded coordinate.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-02-21
     ******************************************************************************/
    geoops::Waypoint AStar::FindNearestStartPoint(const geoops::UTMCoordinate& stStartCoordinate)
    {
        // Create instance variables.
        geoops::UTMCoordinate stAdjustedStartCoordinate = stStartCoordinate;
        // Round the start coordinate to align with the grid.
        RoundUTMCoordinate(stAdjustedStartCoordinate);
        // Make a copy of the rounded start coordinate so we can compare it to the original start coordinate later.
        geoops::UTMCoordinate stRoundedStart = stAdjustedStartCoordinate;

        // Continue shifting until the adjusted start coordinate no longer overlaps any obstacle.
        bool bStartBlocked = true;
        while (bStartBlocked)
        {
            bStartBlocked = false;

            // Check each obstacle.
            for (size_t i = 0; i < m_vObstacles.size(); i++)
            {
                // Calculate the avoidance radius and obstacle borders.
                double dAvoidanceRadius     = constants::ASTAR_AVOIDANCE_MULTIPLIER * m_vObstacles[i].dRadius;
                double dEastObstacleBorder  = m_vObstacles[i].GetUTMCoordinate().dEasting + dAvoidanceRadius;
                double dWestObstacleBorder  = m_vObstacles[i].GetUTMCoordinate().dEasting - dAvoidanceRadius;
                double dNorthObstacleBorder = m_vObstacles[i].GetUTMCoordinate().dNorthing + dAvoidanceRadius;
                double dSouthObstacleBorder = m_vObstacles[i].GetUTMCoordinate().dNorthing - dAvoidanceRadius;

                // If the start coordinate is inside the obstacle's borders...
                if (dWestObstacleBorder < stAdjustedStartCoordinate.dEasting && stAdjustedStartCoordinate.dEasting < dEastObstacleBorder &&
                    dSouthObstacleBorder < stAdjustedStartCoordinate.dNorthing && stAdjustedStartCoordinate.dNorthing < dNorthObstacleBorder)
                {
                    bStartBlocked = true;

                    // Shift along the X axis: move to just outside the obstacle.
                    if (stAdjustedStartCoordinate.dEasting > m_vObstacles[i].GetUTMCoordinate().dEasting)
                    {
                        stAdjustedStartCoordinate.dEasting = dEastObstacleBorder + constants::ASTAR_NODE_SIZE * 2;
                    }
                    else
                    {
                        stAdjustedStartCoordinate.dEasting = dWestObstacleBorder - constants::ASTAR_NODE_SIZE * 2;
                    }

                    // Shift along the Y axis: move to just outside the obstacle.
                    if (stAdjustedStartCoordinate.dNorthing > m_vObstacles[i].GetUTMCoordinate().dNorthing)
                    {
                        stAdjustedStartCoordinate.dNorthing = dNorthObstacleBorder + constants::ASTAR_NODE_SIZE * 2;
                    }
                    else
                    {
                        stAdjustedStartCoordinate.dNorthing = dSouthObstacleBorder - constants::ASTAR_NODE_SIZE * 2;
                    }

                    // Round the coordinate to align with the grid.
                    RoundUTMCoordinate(stAdjustedStartCoordinate);

                    // Break out of the obstacle loop to recheck all obstacles with the new coordinate.
                    break;
                }
            }
        }

        // Check if the adjusted start coordinate doesn't equal the original start coordinate.
        if (stAdjustedStartCoordinate != stRoundedStart)
        {
            // Submit log message.
            geoops::GeoMeasurement stMeasurement = geoops::CalculateGeoMeasurement(stRoundedStart, stAdjustedStartCoordinate);
            LOG_WARNING(logging::g_qSharedLogger,
                        "The start node was adjusted from UTM point ({}, {}) to UTM point ({}, {}) to avoid obstacles. The distance between the original start and the "
                        "adjusted start is {} meters.",
                        stRoundedStart.dEasting,
                        stRoundedStart.dNorthing,
                        stAdjustedStartCoordinate.dEasting,
                        stAdjustedStartCoordinate.dNorthing,
                        stMeasurement.dDistanceMeters);
        }

        // Return the adjusted start point that no longer overlaps any obstacle.
        return stAdjustedStartCoordinate;
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
    bool AStar::ValidCoordinate(const double dEasting, const double dNorthing)
    {
        // For each obstacle.
        for (size_t i = 0; i < m_vObstacles.size(); i++)
        {
            // Multiplier for avoidance radius.
            double dAvoidanceRadius = constants::ASTAR_AVOIDANCE_MULTIPLIER * m_vObstacles[i].dRadius;
            // Create obstacle borders.
            double dEastObstacleBorder  = m_vObstacles[i].GetUTMCoordinate().dEasting + dAvoidanceRadius;
            double dWestObstacleBorder  = m_vObstacles[i].GetUTMCoordinate().dEasting - dAvoidanceRadius;
            double dNorthObstacleBorder = m_vObstacles[i].GetUTMCoordinate().dNorthing + dAvoidanceRadius;
            double dSouthObstacleBorder = m_vObstacles[i].GetUTMCoordinate().dNorthing - dAvoidanceRadius;

            // Return false if node is within obstacle borders.
            if (dWestObstacleBorder < dEasting && dEasting < dEastObstacleBorder && dNorthObstacleBorder > dNorthing && dNorthing > dSouthObstacleBorder)
            {
                return false;
            }
        }

        // Boundary check (Returns true if params indicate a coordinate inside of the search grid).
        if (dEasting >= (m_stStartNode.stNodeLocation.dEasting - constants::ASTAR_MAX_SEARCH_GRID - constants::ASTAR_NODE_SIZE) &&
            dEasting <= (m_stStartNode.stNodeLocation.dEasting + constants::ASTAR_MAX_SEARCH_GRID + constants::ASTAR_NODE_SIZE) &&
            dNorthing >= (m_stStartNode.stNodeLocation.dNorthing - constants::ASTAR_MAX_SEARCH_GRID - constants::ASTAR_NODE_SIZE) &&
            dNorthing <= (m_stStartNode.stNodeLocation.dNorthing + constants::ASTAR_MAX_SEARCH_GRID + constants::ASTAR_NODE_SIZE))
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
    void AStar::RoundUTMCoordinate(geoops::UTMCoordinate& stCoordinateToRound)
    {
        stCoordinateToRound.dEasting  = std::round(stCoordinateToRound.dEasting / constants::ASTAR_NODE_SIZE) * constants::ASTAR_NODE_SIZE;
        stCoordinateToRound.dNorthing = std::round(stCoordinateToRound.dNorthing / constants::ASTAR_NODE_SIZE) * constants::ASTAR_NODE_SIZE;
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
}    // namespace pathplanners
