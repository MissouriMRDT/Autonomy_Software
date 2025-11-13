/******************************************************************************
 * @brief Implementation file for the GeoPlanner class, which provides path planning
 *     functionality using LiDAR data and A* algorithm.
 *
 * @file GeoPlanner.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-09-24
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "GeoPlanner.h"
#include "../../AutonomyNetworking.h"

/******************************************************************************
 * @brief This namespace stores classes, functions, and structs that are used to
 *     implement different path planner algorithms used by the rover to determine
 *     the optimal path to take for any given situation.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-09-24
 ******************************************************************************/
namespace pathplanners
{
    /******************************************************************************
     * @brief Construct a new Geo Planner:: Geo Planner object.
     *
     * @param dTileSize - The size of each tile in meters. Default is 5.0 meters.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-24
     ******************************************************************************/
    GeoPlanner::GeoPlanner(double dTileSize)
    {
        // Initialize member variables.
        m_dTileSize             = dTileSize;
        m_pLiDARHandler         = nullptr;
        m_nStartID              = -1;
        m_nEndID                = -1;
        m_dBeta                 = 0.5;
        m_dMinTravScore         = 0.0;
        m_dSearchRadius         = 3.0;
        m_dMaxSearchTimeSeconds = 120.0;
        m_pPathTracer           = std::make_unique<logging::graphing::PathTracer>("GeoPlanner Path", false);
        m_pKDTree               = std::make_unique<KDTree2D>(PointKDAccessor());

        // Create path plotter layers.
        m_pPathTracer->CreateDotLayer("TerrainPoints", "gray", false);
        m_pPathTracer->CreatePathLayer("RoverPath", "red");

        // Set RoveComm Node callbacks.
        network::g_pRoveCommUDPNode->AddUDPCallback<float>(MinTravScore, manifest::Autonomy::COMMANDS.find("SETMINTRAVSCORE")->second.DATA_ID);
        network::g_pRoveCommUDPNode->AddUDPCallback<float>(BetaBias, manifest::Autonomy::COMMANDS.find("SETBETABIAS")->second.DATA_ID);

        // Log initialization message.
        LOG_INFO(logging::g_qSharedLogger, "GeoPlanner initialized with tile size: {} meters", std::to_string(dTileSize));
    }

    /******************************************************************************
     * @brief Destroy the Geo Planner:: Geo Planner object.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-27
     ******************************************************************************/
    GeoPlanner::~GeoPlanner()
    {
        // Destructor
    }

    /******************************************************************************
     * @brief Plan a path from the start to the end UTM coordinates using A* algorithm.
     *
     * @param pLiDARHandler - Pointer to the LiDARHandler instance for fetching geospatial data.
     * @param stStart - The starting UTM coordinate.
     * @param stEnd - The ending UTM coordinate.
     * @param dBeta - A bias factor for traversal score weighting. Higher values favor safer paths with better trav_scores.
     * @param dSearchRadius - The radius in meters to search for neighboring points during path planning.
     * @param dMaxSearchTimeSeconds - The maximum time in seconds to spend searching for a path.
     * @param bPlotPath - Whether to plot the planned path and terrain in 3D.
     * @return std::vector<geoops::Waypoint> - The planned path waypoints.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-28
     ******************************************************************************/
    std::vector<geoops::Waypoint> GeoPlanner::PlanPath(LiDARHandler* pLiDARHandler,
                                                       const geoops::UTMCoordinate& stStart,
                                                       const geoops::UTMCoordinate& stEnd,
                                                       double dSearchRadius,
                                                       double dMaxSearchTimeSeconds,
                                                       bool bPlotPath)
    {
        // Acquire a mutex lock so we don't try to plan multiple paths at the same time.
        std::lock_guard<std::mutex> lkPathLock(m_muPathGenMutex);

        // Initialize member variables.
        m_pLiDARHandler         = pLiDARHandler;
        m_dSearchRadius         = dSearchRadius;
        m_dMaxSearchTimeSeconds = dMaxSearchTimeSeconds;

        // Submit logger message.
        LOG_NOTICE(logging::g_qSharedLogger,
                   "Starting GeoPlanner path planning from ({:.2f}, {:.2f}) to ({:.2f}, {:.2f}) with beta: {}, search radius: {} meters, min traversal score: {}.",
                   stStart.dEasting,
                   stStart.dNorthing,
                   stEnd.dEasting,
                   stEnd.dNorthing,
                   m_dBeta,
                   m_dSearchRadius,
                   m_dMinTravScore);

        // Validate beta to avoid accidental disabling.
        if (m_dBeta <= 0.0)
        {
            m_dBeta = 0.001;
            LOG_WARNING(logging::g_qSharedLogger, "GeoPlanner: supplied dBeta {} invalid; using fallback 0.001.", m_dBeta);
        }

        // Store the start time.
        std::chrono::time_point<std::chrono::high_resolution_clock> tmStartTime = std::chrono::high_resolution_clock::now();

        // Initialize search for new start and end points.
        this->InitializeSearch(stStart, stEnd);
        // Track time taken to initialize search.
        std::chrono::time_point<std::chrono::high_resolution_clock> tmAfterInit = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dInitDuration                             = tmAfterInit - tmStartTime;
        LOG_INFO(logging::g_qSharedLogger, "GeoPlanner search initialization took {:.6f} seconds.", dInitDuration.count());

        // Run A* search algorithm.
        this->SearchAStar();
        // Track time taken to perform search.
        std::chrono::time_point<std::chrono::high_resolution_clock> tmAfterSearch = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dSearchDuration                             = tmAfterSearch - tmAfterInit;
        LOG_INFO(logging::g_qSharedLogger, "GeoPlanner A* search took {:.6f} seconds.", dSearchDuration.count());

        // Reconstruct the path from the predecessor map.
        std::vector<geoops::Waypoint> vPath = this->ReconstructPath();
        // Track total time taken for path planning.
        std::chrono::time_point<std::chrono::high_resolution_clock> tmEndTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dTotalDuration                          = tmEndTime - tmAfterSearch;
        LOG_INFO(logging::g_qSharedLogger, "GeoPlanner path reconstruction took {:.6f} seconds.", dTotalDuration.count());

        // Log the total time taken for path planning.
        std::chrono::duration<double> dOverallDuration = tmEndTime - tmStartTime;
        LOG_NOTICE(logging::g_qSharedLogger, "GeoPlanner total path planning took {:.6f} seconds. Path is {} waypoints long.", dOverallDuration.count(), vPath.size());

        // Plot the path and terrain if requested.
        if (bPlotPath && !vPath.empty())
        {
            this->PlotPathAndTerrain(vPath);
        }
        else if (bPlotPath && vPath.empty())
        {
            LOG_WARNING(logging::g_qSharedLogger, "No path to plot.");
        }

        return vPath;
    }

    /******************************************************************************
     * @brief Clear all cached tiles and KD-Tree data.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-27
     ******************************************************************************/
    void GeoPlanner::ClearGeoCache()
    {
        // Acquire a mutex lock so we don't try to clear cache while planning a path.
        std::lock_guard<std::mutex> lkResourceLock(m_muPathGenMutex);

        // Clear all cached tiles and KD-Tree data.
        m_umTileMapCache.clear();
    }

    /******************************************************************************
     * @brief Set the size of each tile in meters.
     *
     * @param dTileSize - The new tile size in meters.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-10-28
     ******************************************************************************/
    void GeoPlanner::SetTileSize(double dTileSize)
    {
        m_dTileSize = dTileSize;
    }

    /******************************************************************************
     * @brief Set the minimum traversal score for path planning.
     *
     * @param dMinTravScore - The new minimum traversal score.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-10-28
     ******************************************************************************/
    void GeoPlanner::SetMinTravScore(double dMinTravScore)
    {
        m_dMinTravScore = dMinTravScore;
    }

    /******************************************************************************
     * @brief Set the beta bias for travel scores in path planning.
     *
     * @param dBetaBias - The new beta bias value.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-10-28
     ******************************************************************************/
    void GeoPlanner::SetBetaBias(double dBetaBias)
    {
        m_dBeta = dBetaBias;
    }

    /******************************************************************************
     * @brief Get the size of each tile in meters.
     *
     * @return double - The current tile size in meters.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-10-28
     ******************************************************************************/
    double GeoPlanner::GetTileSize() const
    {
        return m_dTileSize;
    }

    /******************************************************************************
     * @brief Get the minimum traversal score for path planning.
     *
     * @return double - The current minimum traversal score.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-10-28
     ******************************************************************************/
    double GeoPlanner::GetMinTravScore() const
    {
        return m_dMinTravScore;
    }

    /******************************************************************************
     * @brief Get the beta bias for travel scores in path planning.
     *
     * @return double - The current beta bias value.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-10-28
     ******************************************************************************/
    double GeoPlanner::GetBetaBias() const
    {
        return m_dBeta;
    }

    /******************************************************************************
     * @brief Initialize the search by caching the start and end tiles and setting up initial states.
     *
     * @param stStart - The starting UTM coordinate.
     * @param stEnd - The ending UTM coordinate.
     * @return true - Initialization successful.
     * @return false - Initialization failed due to invalid start or end points.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-27
     ******************************************************************************/
    bool GeoPlanner::InitializeSearch(const geoops::UTMCoordinate& stStart, const geoops::UTMCoordinate& stEnd)
    {
        // Clear previous search data.
        m_umAllStates.clear();
        m_umPredecessors.clear();
        m_usClosedSet.clear();
        while (!m_pqOpenSetNextBest.empty())
        {
            m_pqOpenSetNextBest.pop();
        }
        // Reset start and end IDs.
        m_nStartID = -1;
        m_nEndID   = -1;
        // Clear KD-Tree.
        m_usKDTreeInsertedTiles.clear();
        m_pKDTree->clear();

        // Cache the start and end tiles and update ID values.
        PlannerState stStartState = FindClosestLiDARPoint(stStart);
        PlannerState stEndState   = FindClosestLiDARPoint(stEnd);
        m_nStartID                = stStartState.nID;
        m_nEndID                  = stEndState.nID;

        // Check if valid start and end points were found.
        if (m_nStartID == -1 || m_nEndID == -1)
        {
            LOG_ERROR(logging::g_qSharedLogger, "Invalid start or end point for path planning. Start ID: {}, End ID: {}.", m_nStartID, m_nEndID);
            return false;
        }

        // Log the chosen start and end UTM positions.
        LOG_INFO(logging::g_qSharedLogger,
                 "GeoPlanner initialized search with Start ID: {} at ({:.2f}, {:.2f}), End ID: {} at ({:.2f}, {:.2f}).",
                 m_nStartID,
                 stStartState.dEasting,
                 stStartState.dNorthing,
                 m_nEndID,
                 stEndState.dEasting,
                 stEndState.dNorthing);

        return true;
    }

    /******************************************************************************
     * @brief Perform the A* search algorithm to find the optimal path.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-28
     ******************************************************************************/
    void GeoPlanner::SearchAStar()
    {
        // Store the start time.
        std::chrono::high_resolution_clock::time_point tmStartTime = std::chrono::high_resolution_clock::now();

        // Initialize the start state.
        PlannerState stStartState = m_umAllStates[m_nStartID];
        stStartState.dGCost       = 0.0;
        // Heuristic cost (Euclidean distance to goal).
        stStartState.dHCost = this->EuclideanDistance(stStartState.dEasting,
                                                      stStartState.dNorthing,
                                                      stStartState.dAltitude,
                                                      m_umAllStates[m_nEndID].dEasting,
                                                      m_umAllStates[m_nEndID].dNorthing,
                                                      m_umAllStates[m_nEndID].dAltitude);
        // Cumulative traversal score starts as the start point's score.
        m_umAllStates[m_nStartID] = stStartState;

        // Push start into the open set priority queue.
        m_pqOpenSetNextBest.push(m_umAllStates[m_nStartID]);

        // Main A* search loop.
        while (!m_pqOpenSetNextBest.empty())
        {
            // Get the node in the open set with the lowest f = g + h cost.
            PlannerState stCurrentState = m_pqOpenSetNextBest.top();
            // Remove the current node from the open set.
            m_pqOpenSetNextBest.pop();

            // Check stale: compare popped.g to canonical g in m_umAllStates.
            std::unordered_map<int, pathplanners::GeoPlanner::PlannerState>::iterator itState = m_umAllStates.find(stCurrentState.nID);
            if (itState == m_umAllStates.end())
            {
                continue;    // This is unexpected, but skip if not found.
            }
            // If the popped state has a higher G cost than the recorded state, it's stale.
            if (stCurrentState.dGCost > itState->second.dGCost + 1e-9)
            {
                // Stale entry: we previously found a better path and pushed that copy.
                continue;
            }

            // If we've already evaluated it (closed set), skip.
            if (m_usClosedSet.find(stCurrentState.nID) != m_usClosedSet.end())
            {
                continue;
            }

            // If we reached the goal.
            if (stCurrentState.nID == m_nEndID)
            {
                LOG_INFO(logging::g_qSharedLogger, "Goal reached in A* search.");
                return;
            }

            // Mark the current node as evaluated by adding it to the closed set.
            m_usClosedSet.insert(stCurrentState.nID);

            // Ensure the tile containing the current state is loaded.
            this->CheckAndLoadTile(stCurrentState);

            // Query the KDTree to get the neighbors in the given search radius.
            std::vector<LiDARHandler::PointRow> vNeighbors;
            KDQueryPoint stQueryPoint{stCurrentState.dEasting, stCurrentState.dNorthing};
            m_pKDTree->find_within_range(stQueryPoint, m_dSearchRadius, std::back_inserter(vNeighbors));

            // Loop through the neighboring points within the radius.
            for (LiDARHandler::PointRow& stPoint : vNeighbors)
            {
                // Skip if this neighbor is already in the closed set.
                if (m_usClosedSet.find(stPoint.nID) != m_usClosedSet.end())
                {
                    continue;    // Already evaluated.
                }

                /*
                    Calculate the tentative G cost for this neighbor.
                */
                // Calculate Euclidean distance to neighbor.
                double dDistance = this->EuclideanDistance(stCurrentState.dEasting,
                                                           stCurrentState.dNorthing,
                                                           stCurrentState.dAltitude,
                                                           stPoint.dEasting,
                                                           stPoint.dNorthing,
                                                           stPoint.dAltitude);
                // Clamp traversal score [0,1] just to be safe.
                double dScore = std::clamp(stPoint.dTraversalScore, 0.0, 1.0);
                // Calculate cost multiplier based on traversal score and beta.
                double dMultiplier = 1.0 + m_dBeta * (1.0 - dScore);    // <1 not needed; this is >=1
                // Calculate tentative G cost.
                double dTentativeGCost = stCurrentState.dGCost + dDistance * dMultiplier;

                // If this neighbor is not in the open set or we found a better path to it.
                std::unordered_map<int, PlannerState>::const_iterator itNeighborState = m_umAllStates.find(stPoint.nID);
                // If this path to neighbor is better, update its state.
                if (itNeighborState == m_umAllStates.end() || dTentativeGCost + 1e-12 < itNeighborState->second.dGCost)
                {
                    // Update the neighbor's state.
                    PlannerState stNeighborState;
                    stNeighborState.nID                   = stPoint.nID;
                    stNeighborState.dEasting              = stPoint.dEasting;
                    stNeighborState.dNorthing             = stPoint.dNorthing;
                    stNeighborState.dAltitude             = stPoint.dAltitude;
                    stNeighborState.nZone                 = std::stoi(stPoint.szZone.substr(0, 2));    // Assuming zone is stored as string.
                    stNeighborState.bInNorthernHemisphere = (stPoint.dNorthing >= 0);                  // Simple check based on northing.
                    stNeighborState.dGCost                = dTentativeGCost;
                    // Heuristic cost (Euclidean distance to goal).
                    stNeighborState.dHCost = this->EuclideanDistance(stNeighborState.dEasting,
                                                                     stNeighborState.dNorthing,
                                                                     stNeighborState.dAltitude,
                                                                     m_umAllStates[m_nEndID].dEasting,
                                                                     m_umAllStates[m_nEndID].dNorthing,
                                                                     m_umAllStates[m_nEndID].dAltitude);

                    // Update the state map.
                    m_umAllStates[stPoint.nID] = stNeighborState;
                    // Update the predecessor map.
                    m_umPredecessors[stPoint.nID] = stCurrentState.nID;

                    // Add the neighbor to the open set. Duplicates are handled by checking the cost when popping.
                    m_pqOpenSetNextBest.push(stNeighborState);
                }
            }

            // Log progress every 100 iterations.
            if (m_usClosedSet.size() % 1000 == 0)
            {
                LOG_INFO(logging::g_qSharedLogger, "A* search progress: {} nodes evaluated.", m_usClosedSet.size());
            }

            // Check if we've exceeded the maximum search time.
            std::chrono::high_resolution_clock::time_point tmCurrentTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> dElapsedTime                   = tmCurrentTime - tmStartTime;
            if (dElapsedTime.count() >= m_dMaxSearchTimeSeconds)
            {
                LOG_WARNING(logging::g_qSharedLogger, "A* search terminated after exceeding max search time of {} seconds.", m_dMaxSearchTimeSeconds);
                return;
            }
        }
    }

    /******************************************************************************
     * @brief Plan a path from the start to the end UTM coordinates using A* algorithm.
     *
     * @return std::vector<geoops::Waypoint> - The planned path as a vector of waypoints.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-27
     ******************************************************************************/
    std::vector<geoops::Waypoint> GeoPlanner::ReconstructPath() const
    {
        // Create instance variables.
        std::vector<geoops::Waypoint> vPath;
        int nCurrentID = m_nEndID;

        // If the goal wasn't reached, return an empty path.
        if (m_umPredecessors.find(m_nEndID) == m_umPredecessors.end())
        {
            LOG_WARNING(logging::g_qSharedLogger, "Path was not found.");
            return {};
        }

        // Backtrack from the end node to the start node using the cameFrom map.
        while (true)
        {
            // Add the current node ID to the path.
            std::unordered_map<int, PlannerState>::const_iterator itPlannerState = m_umAllStates.find(nCurrentID);
            if (itPlannerState != m_umAllStates.end())
            {
                // Found the PlannerState for this point ID.
                const PlannerState& stState = itPlannerState->second;
                // Convert PlannerState to Waypoint and add to path.
                vPath.emplace_back(geoops::UTMCoordinate(stState.dEasting, stState.dNorthing, stState.nZone, stState.bInNorthernHemisphere, stState.dAltitude),
                                   geoops::WaypointType::eNavigationWaypoint,
                                   0.5,
                                   stState.nID);
            }
            else
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "PlannerState for point ID {} not found during path reconstruction.", nCurrentID);
            }

            // If we've reached the start node, break the loop.
            if (nCurrentID == m_nStartID)
            {
                break;
            }

            // Look at the predecessor map to get the parent node ID.
            nCurrentID = m_umPredecessors.at(nCurrentID);
        }

        // Reverse the path to get it from start to end.
        std::reverse(vPath.begin(), vPath.end());

        return vPath;
    }

    /******************************************************************************
     * @brief Check if the tile containing the current state is loaded, and if not, load it.
     *
     * @param stCurrentState - The current planner state.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-25
     ******************************************************************************/
    void GeoPlanner::CheckAndLoadTile(const PlannerState& stCurrentState)
    {
        // Determine which tile the current state is in.
        int nTileX = static_cast<int>(std::floor(stCurrentState.dEasting / m_dTileSize));
        int nTileY = static_cast<int>(std::floor(stCurrentState.dNorthing / m_dTileSize));
        TileKey stTileKey{nTileX, nTileY};

        // Check if the tile is already loaded.
        if (m_umTileMapCache.find(stTileKey) == m_umTileMapCache.end())
        {
            // Tile is not loaded, so we need to load it.
            LiDARHandler::PointFilter stFilter;
            stFilter.dEasting        = (nTileX + 0.5) * m_dTileSize;                                      // Center of the tile in easting.
            stFilter.dNorthing       = (nTileY + 0.5) * m_dTileSize;                                      // Center of the tile in northing.
            stFilter.dRadius         = std::sqrt(2) * (m_dTileSize / 2.0);                                // Radius to cover the entire tile
            stFilter.dTraversalScore = LiDARHandler::PointFilter::Range<double>{m_dMinTravScore, 1.0};    // Only load points with sufficient traversal score.
            std::vector<LiDARHandler::PointRow> vTilePoints = m_pLiDARHandler->GetLiDARData(stFilter);

            // Check if we got any points back.
            if (vTilePoints.empty())
            {
                // Log warning message.
                LOG_WARNING(logging::g_qSharedLogger, "No LiDAR points found in tile ({}, {}).", nTileX, nTileY);
                return;
            }

            // First, we insert the points into the tile cache.
            m_umTileMapCache[stTileKey] = vTilePoints;

            // Log info message.
            LOG_DEBUG(logging::g_qSharedLogger, "Loaded tile ({}, {}) with {} points into cache.", nTileX, nTileY, vTilePoints.size());
        }

        // Check if this tile is already loaded into the KD-Tree.
        if (m_usKDTreeInsertedTiles.find(stTileKey) == m_usKDTreeInsertedTiles.end())
        {
            // Get the points for this tile from the cache.
            std::vector<LiDARHandler::PointRow>& vTilePoints = m_umTileMapCache[stTileKey];
            // Next, we will insert the points into the KD-Tree for fast spatial queries.
            for (LiDARHandler::PointRow& stLiDARPoint : vTilePoints)
            {
                // Insert this point into the KDTree.
                m_pKDTree->insert(stLiDARPoint);

                /*
                    Add tile points with IDs to the all states map.
                */
                // Convert the szZone ("15S") to an integer zone number (15) and hemisphere (true/false, north/south).
                int nZoneNumber            = std::stoi(stLiDARPoint.szZone.substr(0, 2));
                bool bIsNorthernHemisphere = stLiDARPoint.dNorthing >= 0;    // Simple check based on northing.

                // Don't check if it's already there, just assign to overwrite if it is.
                m_umAllStates[stLiDARPoint.nID] = PlannerState{stLiDARPoint.nID,
                                                               stLiDARPoint.dEasting,
                                                               stLiDARPoint.dNorthing,
                                                               stLiDARPoint.dAltitude,
                                                               nZoneNumber,
                                                               bIsNorthernHemisphere,
                                                               std::numeric_limits<double>::infinity(),
                                                               0.0};
            }
            // Mark this tile as loaded into the KD-Tree.
            m_usKDTreeInsertedTiles.insert(stTileKey);

            /*
                Optimize the KD-Tree after bulk insertion.

                We don't want to optimize too often, so we'll just check the count of the
                inserted tiles and optimize every so tiles loaded.
            */
            if (m_usKDTreeInsertedTiles.size() % 100 == 0)
            {
                m_pKDTree->optimize();
                LOG_INFO(logging::g_qSharedLogger, "Optimized KD-Tree after loading {} tiles.", m_usKDTreeInsertedTiles.size());
            }

            // Log info message.
            LOG_DEBUG(logging::g_qSharedLogger, "Loaded tile ({}, {}) with {} points into KD-Tree.", nTileX, nTileY, vTilePoints.size());
        }
    }

    /******************************************************************************
     * @brief Find the closest LiDAR point to the given UTM coordinate.
     *
     * @param stCoordinate - The UTM coordinate to find the closest LiDAR point to.
     * @return GeoPlanner::PlannerState - The PlannerState representing the closest LiDAR point.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-25
     ******************************************************************************/
    GeoPlanner::PlannerState GeoPlanner::FindClosestLiDARPoint(const geoops::UTMCoordinate& stCoordinate)
    {
        // Create instance variables.
        PlannerState stClosestPoint;

        // First, we need to check to make sure that the tile our given coordinate lies in is loaded.
        CheckAndLoadTile(PlannerState{-1, stCoordinate.dEasting, stCoordinate.dNorthing, stCoordinate.dAltitude});

        // Now we can perform a nearest neighbor search in the KD-Tree.
        KDQueryPoint stQueryPoint{stCoordinate.dEasting, stCoordinate.dNorthing};
        std::pair<pathplanners::KDTree2D::const_iterator, pathplanners::PointKDAccessor::result_type> tpResult = m_pKDTree->find_nearest(stQueryPoint);
        if (tpResult.first != m_pKDTree->end())
        {
            // We found a nearest neighbor, populate the PlannerState.
            const LiDARHandler::PointRow& stNearestPoint = *(tpResult.first);
            stClosestPoint.nID                           = stNearestPoint.nID;
            stClosestPoint.dEasting                      = stNearestPoint.dEasting;
            stClosestPoint.dNorthing                     = stNearestPoint.dNorthing;
            stClosestPoint.dAltitude                     = stNearestPoint.dAltitude;
            stClosestPoint.dGCost                        = std::numeric_limits<double>::infinity();
            stClosestPoint.dHCost                        = 0.0;    // H cost will be calculated later.
        }
        else
        {
            // No nearest neighbor found, log an error.
            LOG_ERROR(logging::g_qSharedLogger, "No nearest LiDAR point found for coordinate ({}, {}).", stCoordinate.dEasting, stCoordinate.dNorthing);
        }

        return stClosestPoint;
    }

    /******************************************************************************
     * @brief Plot the given path and the terrain points that the path goes through.
     *
     * @param vPath - The vector of waypoints representing the path to plot.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-25
     ******************************************************************************/
    void GeoPlanner::PlotPathAndTerrain(const std::vector<geoops::Waypoint>& vPath) const
    {
        // Clear the current layers of the path.
        m_pPathTracer->ClearLayer("TerrainPoints");
        m_pPathTracer->ClearLayer("RoverPath");

        // Loop through the path and only display the terrain tiles that the path goes through.
        std::unordered_set<TileKey, TileKeyHash, TileKeyEqual> usTilesToPlot;
        for (const geoops::Waypoint& stWaypoint : vPath)
        {
            // Determine which tile this waypoint is in.
            int nTileX = static_cast<int>(std::floor(stWaypoint.GetUTMCoordinate().dEasting / m_dTileSize));
            int nTileY = static_cast<int>(std::floor(stWaypoint.GetUTMCoordinate().dNorthing / m_dTileSize));
            usTilesToPlot.insert(TileKey{nTileX, nTileY});

            // Also check the surrounding tiles to give some context.
            for (int nXOffset = -1; nXOffset <= 1; ++nXOffset)
            {
                for (int nYOffset = -1; nYOffset <= 1; ++nYOffset)
                {
                    usTilesToPlot.insert(TileKey{nTileX + nXOffset, nTileY + nYOffset});
                }
            }
        }

        // Now plot the points from these tiles.
        for (const TileKey& stTileKey : usTilesToPlot)
        {
            std::unordered_map<TileKey, std::vector<LiDARHandler::PointRow>, TileKeyHash, TileKeyEqual>::const_iterator itCachedTile = m_umTileMapCache.find(stTileKey);
            if (itCachedTile != m_umTileMapCache.end())
            {
                // Create instance variables.
                std::vector<geoops::Waypoint> stTerrainWaypoints;
                const std::vector<LiDARHandler::PointRow>& vTilePoints = itCachedTile->second;

                // Subsample the points if there are too many to plot.
                const size_t nMaxPointsToPlot = 10;
                if (vTilePoints.size() > nMaxPointsToPlot)
                {
                    double dSubsampleFactor = static_cast<double>(vTilePoints.size()) / static_cast<double>(nMaxPointsToPlot);
                    std::vector<LiDARHandler::PointRow> vSubsampledPoints;
                    for (size_t i = 0; i < vTilePoints.size(); i += static_cast<size_t>(dSubsampleFactor))
                    {
                        vSubsampledPoints.push_back(vTilePoints[i]);
                    }
                    // Use the subsampled points for plotting.
                    stTerrainWaypoints.reserve(vSubsampledPoints.size());
                    for (const LiDARHandler::PointRow& stPoint : vSubsampledPoints)
                    {
                        // Create a waypoint from the PointRow struct.
                        geoops::Waypoint stWaypoint{geoops::UTMCoordinate(stPoint.dEasting,
                                                                          stPoint.dNorthing,
                                                                          std::stoi(stPoint.szZone.substr(0, 2)),
                                                                          (stPoint.dNorthing >= 0),
                                                                          stPoint.dAltitude),
                                                    geoops::WaypointType::eNavigationWaypoint,
                                                    0.01,
                                                    stPoint.nID};
                        // Add the waypoint to the terrain waypoints vector.
                        stTerrainWaypoints.push_back(stWaypoint);
                    }
                }
                else
                {
                    // Use all points if under the max limit.
                    stTerrainWaypoints.reserve(vTilePoints.size());

                    for (const LiDARHandler::PointRow& stPoint : vTilePoints)
                    {
                        // Create a waypoint from the PointRow struct.
                        geoops::Waypoint stWaypoint{geoops::UTMCoordinate(stPoint.dEasting,
                                                                          stPoint.dNorthing,
                                                                          std::stoi(stPoint.szZone.substr(0, 2)),
                                                                          (stPoint.dNorthing >= 0),
                                                                          stPoint.dAltitude),
                                                    geoops::WaypointType::eNavigationWaypoint,
                                                    0.01,
                                                    stPoint.nID};
                        // Add the waypoint to the terrain waypoints vector.
                        stTerrainWaypoints.push_back(stWaypoint);
                    }
                }

                // Add the waypoints to the path tracer.
                m_pPathTracer->AddDots(stTerrainWaypoints, "TerrainPoints", 0);
            }
        }

        // Finally, add the planned path to the path tracer.
        m_pPathTracer->AddPathPoints(vPath, "RoverPath", 0);
    }

    /******************************************************************************
     * @brief Calculate the distance between two UTM coordinates.
     *
     * @param dEasting1 - The easting of the first point.
     * @param dNorthing1 - The northing of the first point.
     * @param Altitude1 - The altitude of the first point.
     * @param dEasting2 - The easting of the second point.
     * @param dNorthing2 - The northing of the second point.
     * @param Altitude2 - The altitude of the second point.
     * @return double - The distance between the two points.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-10-20
     ******************************************************************************/
    double GeoPlanner::EuclideanDistance(double dEasting1, double dNorthing1, double Altitude1, double dEasting2, double dNorthing2, double Altitude2) const
    {
        // Calculate distance between two points.
        double dDiffX = dEasting1 - dEasting2;
        double dDiffY = dNorthing1 - dNorthing2;
        double dDiffZ = Altitude1 - Altitude2;

        return std::sqrt(dDiffX * dDiffX + dDiffY * dDiffY + dDiffZ * dDiffZ);
    }
}    // namespace pathplanners
