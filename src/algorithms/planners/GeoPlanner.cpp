/******************************************************************************
 * @brief This file contains the implementation of the GeoPlanner class, which is responsible for
 *       planning geospatial paths using Dijkstra's algorithm with a bias towards travel scores.
 *
 * @file GeoPlanner.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-16
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "GeoPlanner.h"
#include "../../AutonomyLogging.h"

#include <chrono>

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
     * @brief Construct a new Geo Planner:: Geo Planner object.
     *
     * @param dTileSize - The size of the tiles used in the implicit graph representation.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-20
     ******************************************************************************/
    GeoPlanner::GeoPlanner(double dTileSize)
    {
        // Initialize member variables.
        m_nStartID      = -1;                                                                     // Default start ID.
        m_nEndID        = -1;                                                                     // Default end ID.
        m_dBeta         = 1.0;                                                                    // Default bias factor for travel scores.
        m_dMinTravScore = 0.8;                                                                    // Default minimum travel score threshold.
        m_dTileSize     = dTileSize;                                                              // Set the tile size.
        m_pLiDARHandler = nullptr;                                                                // Initialize LiDARHandler pointer to null.
        m_pPathTracer   = std::make_unique<logging::graphing::PathTracer>("Rover Path", true);    // Initialize the path tracer for 3D visualization.

        // Set up the path tracer for 3D visualization.
        m_pPathTracer->CreateDotLayer("Terrain", "gray");
        m_pPathTracer->CreateDotLayer("StartAndEnd", "red");
        m_pPathTracer->CreatePathLayer("Path");
    }

    /******************************************************************************
     * @brief Destroy the GeoPlanner::GeoPlanner object.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-16
     ******************************************************************************/
    GeoPlanner::~GeoPlanner()
    {
        // Nothing to do yet.
    }

    /******************************************************************************
     * @brief Plans a path between two UTM coordinates using the LiDARHandler to fetch geospatial data for
     *      the most optimal path using Dijkstra's algorithm with a bias towards travel scores.
     *
     * @param pLiDARHandler - Pointer to the LiDARHandler instance used to fetch geospatial data.
     * @param stStart - The starting UTM coordinate for the path planning.
     * @param stEnd - The ending UTM coordinate for the path planning.
     * @param dBeta - The bias factor for travel scores in Dijkstra's algorithm.  0.0-1.0, where 0.0 is solely distance based and 1.0 is solely travel score based.
     * @param dSearchRadius - The search radius for finding neighbors.
     * @param dMinTravScore - The minimum travel score threshold for path planning.
     * @param bPlotPath - Whether to plot the path and terrain using the PathTracer.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-16
     ******************************************************************************/
    std::vector<geoops::Waypoint> GeoPlanner::PlanPath(LiDARHandler* pLiDARHandler,
                                                       const geoops::UTMCoordinate& stStart,
                                                       const geoops::UTMCoordinate& stEnd,
                                                       double dBeta,
                                                       double dSearchRadius,
                                                       double dMinTravScore,
                                                       bool bPlotPath)
    {
        // Create instance variables.
        std::vector<geoops::Waypoint> vPath;    // Vector to store the path waypoints.
        std::chrono::time_point<std::chrono::high_resolution_clock> tmStartTime, tmEndTime;
        double dInitTime = 0.0, dSearchTime = 0.0, dReconstructTime = 0.0;

        // Update member variables with the provided parameters.
        m_pLiDARHandler = pLiDARHandler;
        m_dBeta         = dBeta;
        m_dMinTravScore = dMinTravScore;
        m_dSearchRadius = dSearchRadius;

        // Initialize the search state.
        tmStartTime = std::chrono::high_resolution_clock::now();
        this->InitializeSearch(stStart, stEnd);
        tmEndTime = std::chrono::high_resolution_clock::now();
        dInitTime = std::chrono::duration<double>(tmEndTime - tmStartTime).count();

        // Perform the Dijkstra's search.
        tmStartTime = std::chrono::high_resolution_clock::now();
        this->SearchDijkstra();
        tmEndTime   = std::chrono::high_resolution_clock::now();
        dSearchTime = std::chrono::duration<double>(tmEndTime - tmStartTime).count();

        // Reconstruct the path from the start to the end.
        tmStartTime                    = std::chrono::high_resolution_clock::now();
        std::vector<int> vFinalPathIDs = this->ReconstructPath();

        /*
            Convert the path IDs to waypoints.
        */
        // Build a map from path ID to its index in the final path for fast lookup and ordering.
        std::unordered_map<int, size_t> umIDToIndex;
        for (size_t siIter = 0; siIter < vFinalPathIDs.size(); ++siIter)
        {
            umIDToIndex[vFinalPathIDs[siIter]] = siIter;
        }

        // Collect all points from the tile cache that are in the path.
        std::vector<LiDARHandler::PointRow> vPathPoints;
        for (const std::pair<TileKey, std::vector<LiDARHandler::PointRow>>& stdTilePair : m_umTileMapCache)
        {
            for (const LiDARHandler::PointRow& stPoint : stdTilePair.second)
            {
                if (umIDToIndex.count(stPoint.nID))
                {
                    vPathPoints.push_back(stPoint);
                }
            }
        }

        // Sort the points according to their order in the path.
        std::sort(vPathPoints.begin(),
                  vPathPoints.end(),
                  [&](const LiDARHandler::PointRow& stPointA, const LiDARHandler::PointRow& stPointB) { return umIDToIndex[stPointA.nID] < umIDToIndex[stPointB.nID]; });

        // Convert the sorted points to waypoints.
        vPath.reserve(vPathPoints.size());
        for (const LiDARHandler::PointRow& stPoint : vPathPoints)
        {
            bool bWithinNorthernHemisphere = stPoint.szZone.back() == 'N';
            int nZone                      = std::stoi(stPoint.szZone.substr(0, stPoint.szZone.size() - 1));
            geoops::Waypoint stWaypoint(geoops::UTMCoordinate(stPoint.dEasting, stPoint.dNorthing, nZone, bWithinNorthernHemisphere, stPoint.dAltitude),
                                        geoops::WaypointType::eNavigationWaypoint,
                                        0.0,
                                        stPoint.nID);
            vPath.push_back(stWaypoint);
        }

        // Log the timing information.
        tmEndTime        = std::chrono::high_resolution_clock::now();
        dReconstructTime = std::chrono::duration<double>(tmEndTime - tmStartTime).count();
        // Calculate the distance between each waypoint in the path.
        double dTotalDistance = 0.0;
        for (size_t siIter = 1; siIter < vPath.size(); ++siIter)
        {
            double dDistance = std::hypot(vPath[siIter].GetUTMCoordinate().dEasting - vPath[siIter - 1].GetUTMCoordinate().dEasting,
                                          vPath[siIter].GetUTMCoordinate().dNorthing - vPath[siIter - 1].GetUTMCoordinate().dNorthing);
            dTotalDistance += dDistance;
        }

        LOG_NOTICE(logging::g_qSharedLogger,
                   "GeoPlanner found path in {} seconds (Init: {}, Search: {}, Reconstruct: {}). The path is {} waypoints long and covers a distance of {} meters.",
                   dInitTime + dSearchTime + dReconstructTime,
                   dInitTime,
                   dSearchTime,
                   dReconstructTime,
                   vPath.size(),
                   dTotalDistance);

        // If requested, plot the path and terrain using the PathTracer.
        if (bPlotPath)
        {
            // Plot the path and terrain using the PathTracer.
            this->PlotPathAndTerrain(vPath);

            // Log the path plotting.
            LOG_NOTICE(logging::g_qSharedLogger, "GeoPlanner plotted the path and terrain using PathTracer.");
        }

        // Return the planned path.
        return vPath;
    }

    /******************************************************************************
     * @brief Initializes the search state for the path planning algorithm.
     *
     * @param stStart - The starting UTM coordinate for the path planning.
     * @param stEnd - The ending UTM coordinate for the path planning.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-29
     ******************************************************************************/
    void GeoPlanner::InitializeSearch(const geoops::UTMCoordinate& stStart, const geoops::UTMCoordinate& stEnd)
    {
        // Create instance variables.
        geoops::UTMCoordinate stStartCopy = stStart;
        geoops::UTMCoordinate stEndCopy   = stEnd;

        // Initialize the search state.
        m_umTileMapCache.clear();    // Clear any existing tile cache.

        // Set the start and end IDs based on the closest LiDAR points.
        PlannerState stStartState = FindClosestLiDARPoint(stStartCopy);
        PlannerState stEndState   = FindClosestLiDARPoint(stEndCopy);
        m_nStartID                = stStartState.nID;
        m_nEndID                  = stEndState.nID;
        // Update the start and end UTM coord structs with the altitude from the closest found LiDAR point.
        stStartCopy.dAltitude = stStartState.dAltitude;
        stEndCopy.dAltitude   = stEndState.dAltitude;

        // Clear the graph tracer.
        m_pPathTracer->ClearLayer("Path");
        m_pPathTracer->ClearLayer("Terrain");
        m_pPathTracer->ClearLayer("StartAndEnd");
        // Add start and end points to the tracer.
        m_pPathTracer->AddDot(stStartCopy, "StartAndEnd", 0);
        m_pPathTracer->AddDot(stEndCopy, "StartAndEnd", 0);

        // Clean queues.
        while (!m_pqOpenSet.empty())
        {
            m_pqOpenSet.pop();    // Clear the priority queue.
        }

        // Clear maps.
        m_umCosts.clear();           // Clear the cost map.
        m_umPredecessors.clear();    // Clear the predecessors map.
        m_usClosedSet.clear();       // Clear the closed set.

        // Setup costs for the start node and add it to the open set.
        m_umCosts[m_nStartID] = 0.0;                                                                           // Initialize the cost of the start node to 0.
        m_pqOpenSet.push({m_nStartID, stStartCopy.dEasting, stStartCopy.dNorthing, stStartCopy.dAltitude});    // Push the start state into the priority queue.

        // Submit logger message.
        LOG_NOTICE(
            logging::g_qSharedLogger,
            "GeoPlanner initialized search with start location of <{}>, end location of <{}>, beta: {}, search radius: {}, min traversal score: {}. Please be patient...",
            stStartCopy.ToString(),
            stEndCopy.ToString(),
            m_dBeta,
            m_dSearchRadius,
            m_dMinTravScore);
    }

    /******************************************************************************
     * @brief Searches for the optimal path using Dijkstra's algorithm with a bias towards travel scores.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-30
     ******************************************************************************/
    void GeoPlanner::SearchDijkstra()
    {
        // Continue searching while there are states in the open set.
        while (!m_pqOpenSet.empty())
        {
            // Get the state with the lowest cost from the open set.
            PlannerState stCurrentState = m_pqOpenSet.top();
            m_pqOpenSet.pop();    // Remove it from the open set.

            // Check if have already visited this node.
            if (m_usClosedSet.count(stCurrentState.nID))
            {
                continue;    // Skip this state if it has already been processed.
            }

            // Mark this state as processed.
            m_usClosedSet.insert(stCurrentState.nID);

            // Check if we have reached the end node.
            if (stCurrentState.nID == m_nEndID)
            {
                break;
            }

            // Process the neighbors of the current state.
            this->ProcessNeighbors(stCurrentState);
        }
    }

    /******************************************************************************
     * @brief Processes the neighbors of the current state in the path planning algorithm.
     *
     * @param stCurrentState - The current state of the planner containing the easting and northing coordinates.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-29
     ******************************************************************************/
    void GeoPlanner::ProcessNeighbors(const PlannerState& stCurrentState)
    {
        // Calculate the tile X and Y min and max coordinates based on the current state.
        int nTileXMin          = int(std::floor((stCurrentState.dEasting - m_dSearchRadius) / m_dTileSize));
        int nTileXMax          = int(std::floor((stCurrentState.dEasting + m_dSearchRadius) / m_dTileSize));
        int nTileYMin          = int(std::floor((stCurrentState.dNorthing - m_dSearchRadius) / m_dTileSize));
        int nTileYMax          = int(std::floor((stCurrentState.dNorthing + m_dSearchRadius) / m_dTileSize));
        double dSearchRadiusSq = m_dSearchRadius * m_dSearchRadius;

        // Loop through the nearby tiles that we might need to load and process.
        for (int nTileX = nTileXMin; nTileX <= nTileXMax; ++nTileX)
        {
            for (int nTileY = nTileYMin; nTileY <= nTileYMax; ++nTileY)
            {
                // Load the tile if needed.
                double dCenterEasting  = (nTileX + 0.5) * m_dTileSize;
                double dCenterNorthing = (nTileY + 0.5) * m_dTileSize;
                PlannerState stTileCenter{-1, dCenterEasting, dCenterNorthing, 0.0};
                this->CheckAndLoadTile(stTileCenter);

                // Access tile's points.
                std::vector<LiDARHandler::PointRow>& vTilePoints = m_umTileMapCache[{nTileX, nTileY}];

                // Loop through all neighbor points.
                for (const LiDARHandler::PointRow& stNeighborPoint : vTilePoints)
                {
                    // Skip already visited points.
                    if (m_usClosedSet.count(stNeighborPoint.nID))
                    {
                        continue;
                    }

                    // Compute squared distance to avoid sqrt.
                    double dx          = stCurrentState.dEasting - stNeighborPoint.dEasting;
                    double dy          = stCurrentState.dNorthing - stNeighborPoint.dNorthing;
                    double dDistanceSq = dx * dx + dy * dy;

                    if (dDistanceSq <= dSearchRadiusSq)
                    {
                        double dDistance = std::sqrt(dDistanceSq);    // Only if passing the check.
                        this->RelaxEdge(stCurrentState, stNeighborPoint, dDistance);
                    }
                }
            }
        }
    }

    /******************************************************************************
     * @brief Relaxes the edge between the current state and a neighbor point in the path planning algorithm.
     *     This updates the cost to reach the neighbor point if a better path is found.
     *
     * @param stCurrentState - The current state of the planner containing the easting and northing coordinates.
     * @param stNeighborPoint - The neighbor point to relax the edge with.
     * @param dDistance - The distance to the neighbor point from the current state.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-30
     ******************************************************************************/
    void GeoPlanner::RelaxEdge(const PlannerState& stCurrentState, const LiDARHandler::PointRow& stNeighborPoint, double dDistance)
    {
        // Calculate the new cost to reach the neighbor point.
        double dTraversalPenalty = 1.0 - stNeighborPoint.dTraversalScore;
        double dCost             = m_dBeta * dTraversalPenalty + (1.0 - m_dBeta) * dDistance;
        double dAlternativeCost  = stCurrentState.dCost + dCost;

        // Find the node in the cost map.
        std::unordered_map<int, double>::iterator stdIter = m_umCosts.find(stNeighborPoint.nID);
        if (stdIter == m_umCosts.end() || dAlternativeCost < stdIter->second)
        {
            // Update cost and predecessor.
            m_umCosts[stNeighborPoint.nID]        = dAlternativeCost;
            m_umPredecessors[stNeighborPoint.nID] = stCurrentState.nID;

            // Push new state to the open set.
            m_pqOpenSet.emplace(stNeighborPoint.nID, stNeighborPoint.dEasting, stNeighborPoint.dNorthing, stNeighborPoint.dAltitude, dAlternativeCost);
        }
    }

    /******************************************************************************
     * @brief Reconstructs the path from the start to the end point by following the predecessors.
     *    This method builds the path in reverse order and then reverses it to get the correct order.
     *
     * @return std::vector<int> - A vector containing the IDs of the points in the reconstructed path.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-30
     ******************************************************************************/
    std::vector<int> GeoPlanner::ReconstructPath() const
    {
        // Create instance variables.
        std::vector<int> vPath;       // Vector to store the path IDs.
        int nCurrentID = m_nEndID;    // Start from the end ID.

        // Reconstruct the path by following the predecessors.
        while (nCurrentID != m_nStartID && m_umPredecessors.count(nCurrentID))
        {
            vPath.push_back(nCurrentID);                     // Add the current ID to the path.
            nCurrentID = m_umPredecessors.at(nCurrentID);    // Move to the predecessor.
        }

        // Add the start ID to the path.
        vPath.push_back(m_nStartID);
        // Reverse the path to get the correct order.
        std::reverse(vPath.begin(), vPath.end());

        // Return the reconstructed path.
        return vPath;
    }

    /******************************************************************************
     * @brief Checks if the current tile is loaded in the cache, and if not, loads it from the LiDARHandler.
     *
     * @param stCurrentState - The current state of the planner containing the easting and northing coordinates.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-23
     ******************************************************************************/
    void GeoPlanner::CheckAndLoadTile(const GeoPlanner::PlannerState& stCurrentState)
    {
        // Check if the LidarHandler is valid.
        if (!m_pLiDARHandler)
        {
            // If the LiDARHandler is not set, throw an error.
            LOG_ERROR(logging::g_qSharedLogger, "LiDARHandler is not set. Cannot load tile.");
        }

        // Translate the easting and northing from the given UTM coordinate to the tile
        int nTileX = int(std::floor(stCurrentState.dEasting / m_dTileSize));
        int nTileY = int(std::floor(stCurrentState.dNorthing / m_dTileSize));

        // Create a TileKey object for the current tile.
        GeoPlanner::TileKey stTileKey{nTileX, nTileY};

        // Check if the tile is already loaded.
        if (!m_umTileMapCache.count(stTileKey))
        {
            // Load the tile data.
            double dCenterEasting  = (nTileX + 0.5) * m_dTileSize;
            double dCenterNorthing = (nTileY + 0.5) * m_dTileSize;

            // Fetch the points within the tile from the LiDARHandler and add the tile to the cache.
            LiDARHandler::PointFilter stPointFilter;
            stPointFilter.dEasting        = dCenterEasting;
            stPointFilter.dNorthing       = dCenterNorthing;
            stPointFilter.dRadius         = m_dTileSize / 2.0;    // Use half the tile size as the radius.
            stPointFilter.dTraversalScore = LiDARHandler::PointFilter::Range<double>{m_dMinTravScore, 1.0};
            m_umTileMapCache[stTileKey]   = m_pLiDARHandler->GetLiDARData(stPointFilter);
        }
    }

    /******************************************************************************
     * @brief Finds the closest LiDAR point to a given UTM coordinate by iterating through all points in the tile cache.
     *
     * @param stCoordinate - The UTM coordinate to find the closest LiDAR point to.
     * @return GeoPlanner::PlannerState - The state of the closest LiDAR point.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-29
     ******************************************************************************/
    GeoPlanner::PlannerState GeoPlanner::FindClosestLiDARPoint(const geoops::UTMCoordinate& stCoordinate)
    {
        // Create instance variables.
        LiDARHandler::PointRow* stClosestPoint;
        double dBestDistance = std::numeric_limits<double>::infinity();

        // Check if the tile cache needs to be loaded for the given coordinate.
        this->CheckAndLoadTile({-1, stCoordinate.dEasting, stCoordinate.dNorthing, stCoordinate.dAltitude});

        // Iterate through all tiles in the cache.
        for (const std::pair<const TileKey, std::vector<LiDARHandler::PointRow>>& stTilePair : m_umTileMapCache)
        {
            // Iterate through all points in the tile.
            for (const LiDARHandler::PointRow& stPoint : stTilePair.second)
            {
                // Calculate the distance to the current point.
                double dDistance = std::hypot(stPoint.dEasting - stCoordinate.dEasting, stPoint.dNorthing - stCoordinate.dNorthing);

                // If this point is closer than the best found so far, update the closest point.
                if (dDistance < dBestDistance)
                {
                    dBestDistance  = dDistance;
                    stClosestPoint = const_cast<LiDARHandler::PointRow*>(&stPoint);
                }
            }
        }

        // Check if a point was found.
        if (!stClosestPoint)
        {
            // If no point was found, throw an error.
            LOG_ERROR(logging::g_qSharedLogger, "No LiDAR points found in the cache.");
            throw std::runtime_error("No LiDAR points found in the cache.");
        }

        // Return the closest point as a PlannerState.
        return {stClosestPoint->nID, stClosestPoint->dEasting, stClosestPoint->dNorthing, stClosestPoint->dAltitude};
    }

    /******************************************************************************
     * @brief Plots the path and terrain for visualization purposes.
     *
     * @param vPath - The vector of waypoints representing the planned path.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-31
     ******************************************************************************/
    void GeoPlanner::PlotPathAndTerrain(const std::vector<geoops::Waypoint>& vPath) const
    {
        // Add the path points to the path tracer.
        m_pPathTracer->AddPathPoints(vPath, "Path", 0);

        // Use the path points to get the terrain tiles that we need to plot.
        std::unordered_set<GeoPlanner::TileKey, GeoPlanner::TileKeyHash> usTerrainTiles;
        for (const geoops::Waypoint& stWaypoint : vPath)
        {
            // Calculate the tile key for the waypoint.
            int nTileX = int(std::floor(stWaypoint.GetUTMCoordinate().dEasting / m_dTileSize));
            int nTileY = int(std::floor(stWaypoint.GetUTMCoordinate().dNorthing / m_dTileSize));
            GeoPlanner::TileKey stTileKey{nTileX, nTileY};
            usTerrainTiles.insert(stTileKey);
        }

        // Iterate through the terrain tiles and add them to the path tracer.
        for (const GeoPlanner::TileKey& stTileKey : usTerrainTiles)
        {
            // Check if the tile is loaded in the cache.
            if (m_umTileMapCache.count(stTileKey))
            {
                // Convert the tile points to a vector of geoops::Waypoint.
                const std::vector<LiDARHandler::PointRow>& vTilePoints = m_umTileMapCache.at(stTileKey);
                std::vector<geoops::Waypoint> vTileWaypoints;
                vTileWaypoints.reserve(vTilePoints.size());
                for (const LiDARHandler::PointRow& stPoint : vTilePoints)
                {
                    bool bWithinNorthernHemisphere = stPoint.szZone.back() == 'N';
                    int nZone                      = std::stoi(stPoint.szZone.substr(0, stPoint.szZone.size() - 1));
                    geoops::Waypoint stWaypoint(geoops::UTMCoordinate(stPoint.dEasting, stPoint.dNorthing, nZone, bWithinNorthernHemisphere, stPoint.dAltitude),
                                                geoops::WaypointType::eNavigationWaypoint,
                                                0.005,
                                                stPoint.nID);
                    vTileWaypoints.push_back(stWaypoint);
                }

                // Add the points from the tile to the path tracer.
                m_pPathTracer->AddDots(vTileWaypoints, "Terrain", 0.0);
            }
            else
            {
                LOG_WARNING(logging::g_qSharedLogger, "Tile not found in cache. Cannot plot terrain.");
            }
        }
    }
}    // namespace pathplanners
