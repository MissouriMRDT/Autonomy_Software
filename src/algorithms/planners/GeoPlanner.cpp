/******************************************************************************
 * @brief Implementation file for the GeoPlanner class, which provides path planning
 * functionality using LiDAR data and a highly optimized Hierarchical A* algorithm.
 *
 * @file GeoPlanner.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-09-24
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "GeoPlanner.h"
#include "../../AutonomyGlobals.h"
#include "../../AutonomyNetworking.h"

/******************************************************************************
 * @brief This namespace stores classes, functions, and structs that are used to
 * implement different path planner algorithms used by the rover to determine
 * the optimal path to take for any given situation.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-09-24
 ******************************************************************************/
namespace pathplanners
{
    /******************************************************************************
     * @brief Construct a new Geo Planner:: Geo Planner object. Initializes all
     * complex algorithms and mathematical hyperparameters to avoid explicitly
     * hardcoded magic numbers logic.
     *
     * @param dTileSize - The size of each database tile block loaded in meters.
     * @param dGridResolution - Metric scale of an individual discrete costmap cell.
     * @param dHeuristicWeight - The A* bias weight to accelerate goal seeking behaviors.
     * @param dBetaBias - Baseline beta algorithmic multiplier for penalizing bad terrain.
     * @param dMinTravScore - Absolute required lower bound on traversal values to be considered usable pathing terrain.
     * @param nDilationPasses - Number of morphological loop passes executed to fill void structures in sparse point clouds.
     * @param dSafeTravScoreThreshold - Baseline score requirement applied when attempting to snap stray origin coordinates.
     * @param nMaxSpiralSearchRadius - Max concentric rings expanded when searching for safe origin snapping structures.
     * @param siMaxPlotPointsPerTile - Rendering density constraint to prevent visualizer overload.
     * @param dPenaltyScalingFactor - Multiplier intensifying standard penalty math strictly during node score resolution.
     * @param dPenaltyPower - Exponential scaler applied universally to mathematically discourage steep or dangerous zones.
     * @param dPathWaypointTolerance - Native radius value embedded into actively returned planned telemetry sequence nodes.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-24
     ******************************************************************************/
    GeoPlanner::GeoPlanner(double dTileSize,
                           double dGridResolution,
                           double dHeuristicWeight,
                           double dBetaBias,
                           double dMinTravScore,
                           int nDilationPasses,
                           double dSafeTravScoreThreshold,
                           int nMaxSpiralSearchRadius,
                           size_t siMaxPlotPointsPerTile,
                           double dPenaltyScalingFactor,
                           double dPenaltyPower,
                           double dPathWaypointTolerance)
    {
        // Initialize member variables from constructor arguments.
        m_dTileSize               = dTileSize;
        m_dGridResolution         = dGridResolution;
        m_dHeuristicWeight        = dHeuristicWeight;
        m_dBeta                   = dBetaBias;
        m_dMinTravScore           = dMinTravScore;
        m_nDilationPasses         = nDilationPasses;
        m_dSafeTravScoreThreshold = dSafeTravScoreThreshold;
        m_nMaxSpiralSearchRadius  = nMaxSpiralSearchRadius;
        m_siMaxPlotPointsPerTile  = siMaxPlotPointsPerTile;
        m_dPenaltyScalingFactor   = dPenaltyScalingFactor;
        m_dPenaltyPower           = dPenaltyPower;
        m_dPathWaypointTolerance  = dPathWaypointTolerance;

        // Initialize resource pointers and request-specific variables.
        m_pLiDARHandler         = nullptr;
        m_dSearchRadius         = 0.0;
        m_dMaxSearchTimeSeconds = 0.0;
        m_dCorridorPadding      = 0.0;

        // Bind RoveComm UDP Node network callbacks if available.
        if (network::g_pRoveCommUDPNode != nullptr)
        {
            network::g_pRoveCommUDPNode->AddUDPCallback<float>(fnMinTravScoreCallback, manifest::Autonomy::COMMANDS.find("SETMINTRAVSCORE")->second.DATA_ID);
            network::g_pRoveCommUDPNode->AddUDPCallback<float>(fnBetaBiasCallback, manifest::Autonomy::COMMANDS.find("SETBETABIAS")->second.DATA_ID);
        }

        LOG_INFO(logging::g_qSharedLogger, "GeoPlanner initialized successfully with a defined tile fetch size of {} meters.", std::to_string(m_dTileSize));
    }

    /******************************************************************************
     * @brief Destroy the Geo Planner:: Geo Planner object.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-27
     ******************************************************************************/
    GeoPlanner::~GeoPlanner()
    {
        // Smart pointers and standard containers clean themselves up automatically.
    }

    /******************************************************************************
     * @brief Plan an optimal trajectory path from the start UTM to the end UTM
     * coordinate utilizing a 2.5D hierarchical Costmap grid representation.
     *
     * @param pLiDARHandler - Pointer to the LiDARHandler database instance for fetching geospatial data.
     * @param stStart - The desired starting UTM geographic coordinate representation.
     * @param stEnd - The ultimate destination UTM geographic coordinate representation.
     * @param dSearchRadius - Padding base radius used to compute bounds.
     * @param dMaxSearchTimeSeconds - The absolute maximum CPU time in seconds allocated to search attempts.
     * @param dCorridorPadding - The extended corridor buffer dimension appended dynamically outside raw bounds.
     * @return std::vector<geoops::Waypoint> - The sequentially planned traversal path configurations.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-28
     ******************************************************************************/
    std::vector<geoops::Waypoint> GeoPlanner::PlanPath(LiDARHandler* pLiDARHandler,
                                                       const geoops::UTMCoordinate& stStart,
                                                       const geoops::UTMCoordinate& stEnd,
                                                       double dSearchRadius,
                                                       double dMaxSearchTimeSeconds,
                                                       double dCorridorPadding)
    {
        // Acquire a thread mutex lock to prevent concurrent path planning operations from corrupting internal state.
        std::lock_guard<std::mutex> lkPathLock(m_muPathGenMutex);

        // Secure external variables into local class state for this search pass.
        m_pLiDARHandler         = pLiDARHandler;
        m_dSearchRadius         = dSearchRadius;
        m_dMaxSearchTimeSeconds = dMaxSearchTimeSeconds;
        m_dCorridorPadding      = dCorridorPadding;

        LOG_NOTICE(logging::g_qSharedLogger,
                   "Starting GeoPlanner path planning from ({:.2f}, {:.2f}) to ({:.2f}, {:.2f}) with algorithmic beta: {}, minimum score threshold: {}.",
                   stStart.dEasting,
                   stStart.dNorthing,
                   stEnd.dEasting,
                   stEnd.dNorthing,
                   m_dBeta,
                   m_dMinTravScore);

        // Sanity check the beta multiplier to avoid dividing by zero or disabling penalty tracking.
        if (m_dBeta <= 0.0)
        {
            m_dBeta = 0.001;
            LOG_WARNING(logging::g_qSharedLogger, "GeoPlanner: supplied dBeta bias {} is mathematically invalid; utilizing fallback logic 0.001.", m_dBeta);
        }

        // Store execution start time to profile algorithmic bottlenecks.
        std::chrono::time_point<std::chrono::high_resolution_clock> tmStartTime = std::chrono::high_resolution_clock::now();

        // Step 1: Preload the bounding box tiles and construct the dense high-res abstract costmap grid matrices.
        if (!this->PreloadCorridorAndBuildGrid(stStart, stEnd))
        {
            LOG_ERROR(logging::g_qSharedLogger, "Failed to initialize the search grid. Aborting pathfinding routine.");
            return {};
        }

        std::chrono::time_point<std::chrono::high_resolution_clock> tmAfterInit = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dInitDurationSeconds                      = tmAfterInit - tmStartTime;
        LOG_INFO(logging::g_qSharedLogger, "GeoPlanner Grid Generation phase mapped within {:.6f} seconds.", dInitDurationSeconds.count());

        // Step 2: Formally run the highly optimized Weighted A* search logic.
        this->SearchAStar();

        std::chrono::time_point<std::chrono::high_resolution_clock> tmAfterSearch = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dSearchDurationSeconds                      = tmAfterSearch - tmAfterInit;
        LOG_INFO(logging::g_qSharedLogger, "GeoPlanner A* Grid Node Expansion executed within {:.6f} seconds.", dSearchDurationSeconds.count());

        // Step 3: Integrate and rebuild exact geographic sequences tracking backwards from the goal.
        std::vector<geoops::Waypoint> vPath                                   = this->ReconstructPath();

        std::chrono::time_point<std::chrono::high_resolution_clock> tmEndTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dOverallDurationSeconds                 = tmEndTime - tmStartTime;
        LOG_NOTICE(logging::g_qSharedLogger,
                   "GeoPlanner total end-to-end path routing executed within {:.6f} seconds rendering {} waypoints.",
                   dOverallDurationSeconds.count(),
                   vPath.size());

        return vPath;
    }

    /******************************************************************************
     * @brief Explicitly zeroes and reclaims internal spatial database mapping arrays.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-27
     ******************************************************************************/
    void GeoPlanner::ClearGeoCache()
    {
        // Acquire a thread mutex lock to guarantee safety during cache wipes.
        std::lock_guard<std::mutex> lkResourceLock(m_muPathGenMutex);
        m_umTileMapCache.clear();
    }

    /******************************************************************************
     * @brief Sets the dimensional size of database tiles mapped during planning.
     *
     * @param dTileSize - The block size in meters.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-14
     ******************************************************************************/
    void GeoPlanner::SetTileSize(double dTileSize)
    {
        m_dTileSize = dTileSize;
    }

    /******************************************************************************
     * @brief Sets the absolute minimum travel score required for a valid cell.
     *
     * @param dMinTravScore - Minimum permissible score limit.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-14
     ******************************************************************************/
    void GeoPlanner::SetMinTravScore(double dMinTravScore)
    {
        m_dMinTravScore = dMinTravScore;
    }

    /******************************************************************************
     * @brief Sets the algorithm's penalty sensitivity bias multiplier.
     *
     * @param dBetaBias - Baseline beta algorithmic multiplier.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-14
     ******************************************************************************/
    void GeoPlanner::SetBetaBias(double dBetaBias)
    {
        m_dBeta = dBetaBias;
    }

    /******************************************************************************
     * @brief Retrieves the currently configured database tile size constraint.
     *
     * @return double - Tile size mapped dimension in meters.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-14
     ******************************************************************************/
    double GeoPlanner::GetTileSize() const
    {
        return m_dTileSize;
    }

    /******************************************************************************
     * @brief Retrieves the actively configured minimum travel score parameter limit.
     *
     * @return double - The active lowest traversal score permissible.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-14
     ******************************************************************************/
    double GeoPlanner::GetMinTravScore() const
    {
        return m_dMinTravScore;
    }

    /******************************************************************************
     * @brief Retrieves the beta heuristic bias factor configured dynamically.
     *
     * @return double - Scaler factor penalizing bad terrain paths mathematically.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-14
     ******************************************************************************/
    double GeoPlanner::GetBetaBias() const
    {
        return m_dBeta;
    }

    /******************************************************************************
     * @brief Calculates a bounding box based on Start and End coordinates, preloads
     * LiDAR chunks, and structures them down into a contiguous 1D Costmap vector.
     *
     * @param stStart - The designated starting UTM geographic coordinate.
     * @param stEnd - The designated end UTM geographic coordinate.
     * @return true - Grid initialization succeeded.
     * @return false - Total structural invalidation or no usable terrain available.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-14
     ******************************************************************************/
    bool GeoPlanner::PreloadCorridorAndBuildGrid(const geoops::UTMCoordinate& stStart, const geoops::UTMCoordinate& stEnd)
    {
        // Buffer the search bounds to allow the algorithm lateral space to circumvent large topographic obstructions.
        double dPaddingMeters = m_dSearchRadius + m_dCorridorPadding;

        // Establish spatial bounding box limits outlining the entire search corridor.
        double dMinEasting  = std::min(stStart.dEasting, stEnd.dEasting) - dPaddingMeters;
        double dMaxEasting  = std::max(stStart.dEasting, stEnd.dEasting) + dPaddingMeters;
        double dMinNorthing = std::min(stStart.dNorthing, stEnd.dNorthing) - dPaddingMeters;
        double dMaxNorthing = std::max(stStart.dNorthing, stEnd.dNorthing) + dPaddingMeters;

        // Compute the tile keys needed to cover the bounding box.
        int nMinTileX = static_cast<int>(std::floor(dMinEasting / m_dTileSize));
        int nMaxTileX = static_cast<int>(std::floor(dMaxEasting / m_dTileSize));
        int nMinTileY = static_cast<int>(std::floor(dMinNorthing / m_dTileSize));
        int nMaxTileY = static_cast<int>(std::floor(dMaxNorthing / m_dTileSize));

        // Preload all valid database tiles within this geographic block.
        for (int nX = nMinTileX; nX <= nMaxTileX; ++nX)
        {
            for (int nY = nMinTileY; nY <= nMaxTileY; ++nY)
            {
                this->CheckAndLoadTile(nX, nY);
            }
        }

        // Establish the matrix dimensions required for the granular abstract grid.
        m_dGridOriginEasting  = dMinEasting;
        m_dGridOriginNorthing = dMinNorthing;
        m_nGridWidth          = static_cast<int>(std::ceil((dMaxEasting - dMinEasting) / m_dGridResolution));
        m_nGridHeight         = static_cast<int>(std::ceil((dMaxNorthing - dMinNorthing) / m_dGridResolution));

        // Pre-allocate the master costmap vector memory capacity and initialize to empty state (-1.0).
        int nTotalCells = m_nGridWidth * m_nGridHeight;
        m_vCostmap.assign(nTotalCells, GridCell());

        // Overlay raw sparse LiDAR matrices onto the structured grid mapping.
        for (int nX = nMinTileX; nX <= nMaxTileX; ++nX)
        {
            for (int nY = nMinTileY; nY <= nMaxTileY; ++nY)
            {
                TileKey stKey{nX, nY};

                if (m_umTileMapCache.find(stKey) != m_umTileMapCache.end())
                {
                    for (const LiDARHandler::PointRow& stPoint : m_umTileMapCache[stKey])
                    {
                        int nGridX = static_cast<int>((stPoint.dEasting - m_dGridOriginEasting) / m_dGridResolution);
                        int nGridY = static_cast<int>((stPoint.dNorthing - m_dGridOriginNorthing) / m_dGridResolution);

                        // Ensure index bounds are safe before memory injection.
                        if (nGridX >= 0 && nGridX < m_nGridWidth && nGridY >= 0 && nGridY < m_nGridHeight)
                        {
                            int nIdx = GetGridIndex(nGridX, nGridY);

                            // Pessimistic Data Aggregation: We specifically want to take the worst (lowest) traversal score
                            // to ensure obstacles like trees are not masked by overlapping ground returns.
                            if (m_vCostmap[nIdx].dTravScore < 0.0 || stPoint.dTraversalScore < m_vCostmap[nIdx].dTravScore)
                            {
                                m_vCostmap[nIdx].dTravScore            = stPoint.dTraversalScore;
                                m_vCostmap[nIdx].dAltitude             = stPoint.dAltitude;
                                m_vCostmap[nIdx].nClosestPointID       = stPoint.nID;
                                m_vCostmap[nIdx].nZone                 = std::stoi(stPoint.szZone.substr(0, 2));
                                m_vCostmap[nIdx].bInNorthernHemisphere = (stPoint.dNorthing >= 0);
                            }
                        }
                    }
                }
            }
        }

        // Dilate the existing valid cells to bridge structural void gaps in sparse clouds.
        this->FillGridHoles();

        // Overlay dynamic obstacles from the WaypointHandler to ensure stuck state and object detection block paths dynamically in memory.
        std::vector<geoops::Waypoint> vObstacles = globals::g_pWaypointHandler->GetAllObstacles();

        // Loop through the obstacles.
        for (const geoops::Waypoint& stObstacle : vObstacles)
        {
            // Get obstacle UTM coordinate and radius.
            geoops::UTMCoordinate stObsUTM = stObstacle.GetUTMCoordinate();
            double dRadius                 = stObstacle.dRadius;

            // Convert the obstacle's UTM center to grid array coordinates.
            int nObsGridX = static_cast<int>((stObsUTM.dEasting - m_dGridOriginEasting) / m_dGridResolution);
            int nObsGridY = static_cast<int>((stObsUTM.dNorthing - m_dGridOriginNorthing) / m_dGridResolution);

            // Determine the bounding box of the obstacle in grid cells.
            int nRadiusCells = static_cast<int>(std::ceil(dRadius / m_dGridResolution));

            // Looping through the grid cells of the radius.
            for (int nDx = -nRadiusCells; nDx <= nRadiusCells; ++nDx)
            {
                for (int nDy = -nRadiusCells; nDy <= nRadiusCells; ++nDy)
                {
                    // Increase the X and Y values.
                    int nX = nObsGridX + nDx;
                    int nY = nObsGridY + nDy;

                    // Ensure the target indices are within valid 2D grid bounds.
                    if (nX >= 0 && nX < m_nGridWidth && nY >= 0 && nY < m_nGridHeight)
                    {
                        // Make sure that we're checking within the radius with distance formula.
                        double dDistSq = (nDx * m_dGridResolution) * (nDx * m_dGridResolution) + (nDy * m_dGridResolution) * (nDy * m_dGridResolution);

                        // Check if the distance is less than radius squared.
                        if (dDistSq <= (dRadius * dRadius))
                        {
                            // If so, set the grid index to a low trav score.
                            int nIdx                    = GetGridIndex(nX, nY);
                            m_vCostmap[nIdx].dTravScore = 0.01;
                        }
                    }
                }
            }
        }

        // Calculate starting array indices based on geographic coordinate locations.
        int nStartX = static_cast<int>((stStart.dEasting - m_dGridOriginEasting) / m_dGridResolution);
        int nStartY = static_cast<int>((stStart.dNorthing - m_dGridOriginNorthing) / m_dGridResolution);
        int nEndX   = static_cast<int>((stEnd.dEasting - m_dGridOriginEasting) / m_dGridResolution);
        int nEndY   = static_cast<int>((stEnd.dNorthing - m_dGridOriginNorthing) / m_dGridResolution);

        // Enforce strict clamping to prevent edge-case out of bounds index evaluation.
        nStartX = std::clamp(nStartX, 0, m_nGridWidth - 1);
        nStartY = std::clamp(nStartY, 0, m_nGridHeight - 1);
        nEndX   = std::clamp(nEndX, 0, m_nGridWidth - 1);
        nEndY   = std::clamp(nEndY, 0, m_nGridHeight - 1);

        // Perform a safe-snap search to ensure origin/destination points don't land exactly in a red zone or void.
        m_nStartIndex = this->FindNearestValidCell(GetGridIndex(nStartX, nStartY));
        m_nEndIndex   = this->FindNearestValidCell(GetGridIndex(nEndX, nEndY));

        // Abort if no viable terrain whatsoever exists near the required start/end points.
        if (m_nStartIndex == -1 || m_nEndIndex == -1)
        {
            LOG_ERROR(logging::g_qSharedLogger, "GeoPlanner explicit termination: Geographic endpoints have no safe nearby terrain data.");
            return false;
        }

        // Reset fast A* memory trackers to prepare for the active search loop.
        m_vPredecessors.assign(nTotalCells, -1);
        m_vbClosedSet.assign(nTotalCells, false);
        m_vdGCosts.assign(nTotalCells, std::numeric_limits<double>::infinity());

        // Empty the priority queue cleanly.
        while (!m_pqOpenSetNextBest.empty())
        {
            m_pqOpenSetNextBest.pop();
        }

        LOG_INFO(logging::g_qSharedLogger,
                 "Abstract Search Grid Built. Width {} x Height {} (Total: {} cells). Start Index: {}, End Index: {}",
                 m_nGridWidth,
                 m_nGridHeight,
                 nTotalCells,
                 m_nStartIndex,
                 m_nEndIndex);
        return true;
    }

    /******************************************************************************
     * @brief Actively runs a highly optimized Weighted A* pathfinding search upon
     * the 1D Costmap grid utilizing kinematic 3D spatial awareness.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-14
     ******************************************************************************/
    void GeoPlanner::SearchAStar()
    {
        std::chrono::high_resolution_clock::time_point tmStartTime = std::chrono::high_resolution_clock::now();

        // Initialize the origin node states.
        PlannerState stStartState;
        stStartState.nGridIndex = m_nStartIndex;
        stStartState.dGCost     = 0.0;

        int nEndX, nEndY;
        GetGridCoords(m_nEndIndex, nEndX, nEndY);

        // Compute baseline Euclidean heuristic scaled up by heuristic weight.
        int nStartX, nStartY;
        GetGridCoords(m_nStartIndex, nStartX, nStartY);
        stStartState.dHCost =
            m_dHeuristicWeight * EuclideanDistance(nStartX * m_dGridResolution, nStartY * m_dGridResolution, nEndX * m_dGridResolution, nEndY * m_dGridResolution);

        m_vdGCosts[m_nStartIndex] = 0.0;
        m_pqOpenSetNextBest.push(stStartState);

        // Pre-computed lookup tables for rapid 8-way directional neighbor evaluation.
        constexpr int anDx[8]          = {-1, 0, 1, -1, 1, -1, 0, 1};
        constexpr int anDy[8]          = {-1, -1, -1, 0, 0, 1, 1, 1};
        constexpr double adMoveDist[8] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};

        // Main algorithm frontier expansion loop.
        while (!m_pqOpenSetNextBest.empty())
        {
            // Acquire the lowest cost node currently located in the Min-Heap.
            PlannerState stCurrentState = m_pqOpenSetNextBest.top();
            m_pqOpenSetNextBest.pop();

            // Ignore stale states that were updated with a better path later in the queue.
            if (stCurrentState.dGCost > m_vdGCosts[stCurrentState.nGridIndex])
            {
                continue;
            }

            // Immediately exit standard operations if the target goal coordinate was successfully reached.
            if (stCurrentState.nGridIndex == m_nEndIndex)
            {
                LOG_INFO(logging::g_qSharedLogger, "Successfully reached valid goal configuration parameter during A* expansions.");
                return;
            }

            // Mark this specific node index as fully evaluated.
            m_vbClosedSet[stCurrentState.nGridIndex] = true;

            int nCurrentX, nCurrentY;
            GetGridCoords(stCurrentState.nGridIndex, nCurrentX, nCurrentY);

            // Explore all 8 adjacent neighbor grid cells.
            for (int nI = 0; nI < 8; ++nI)
            {
                int nNeighborX = nCurrentX + anDx[nI];
                int nNeighborY = nCurrentY + anDy[nI];

                // Block bounds queries outside of the physical map.
                if (nNeighborX < 0 || nNeighborX >= m_nGridWidth || nNeighborY < 0 || nNeighborY >= m_nGridHeight)
                {
                    continue;
                }

                int nNeighborIdx = GetGridIndex(nNeighborX, nNeighborY);

                // Skip indices that have already been cleanly solved.
                if (m_vbClosedSet[nNeighborIdx])
                {
                    continue;
                }

                // Reference cell parameters.
                const GridCell& stCell        = m_vCostmap[nNeighborIdx];
                const GridCell& stCurrentCell = m_vCostmap[stCurrentState.nGridIndex];

                // Ensure neighbor inherently contains registered geographic parameters and meets minimum score limits.
                if (stCell.dTravScore < 0.0 || stCell.dTravScore < m_dMinTravScore)
                {
                    continue;
                }

                // --- KINEMATIC AWARENESS LOGIC ---
                // Calculate actual true 3D spatial distance incorporating the vertical ascent parameters.
                double dAltDiff    = std::abs(stCell.dAltitude - stCurrentCell.dAltitude);
                double dPlanarDist = adMoveDist[nI] * m_dGridResolution;
                double dTrueDist   = std::sqrt((dPlanarDist * dPlanarDist) + (dAltDiff * dAltDiff));

                // Constrain traversal metrics rigidly to [0,1] bounds simply for mathematical safety.
                double dScore = std::clamp(stCell.dTravScore, 0.0, 1.0);

                // Generate an exponential scaling multiplier. Low traversal scores (e.g., rigid trees or sharp canyons) are
                // inflated heavily. Empowering the multiplier ensures aggressive physical avoidance of high penalties.
                double dPenaltyWeight = std::pow(1.0 - dScore, m_dPenaltyPower);
                double dMultiplier    = 1.0 + (m_dBeta * m_dPenaltyScalingFactor * dPenaltyWeight);

                // Compute exact, penalized aggregate traversal path cost.
                double dTentativeGCost = stCurrentState.dGCost + (dTrueDist * dMultiplier);

                // Adopt spatial progression only if it presents a mathematically optimal routing arrangement.
                if (dTentativeGCost < m_vdGCosts[nNeighborIdx])
                {
                    m_vdGCosts[nNeighborIdx]      = dTentativeGCost;
                    m_vPredecessors[nNeighborIdx] = stCurrentState.nGridIndex;

                    // Package mathematical layout elements for injection to the heap bounds queue.
                    PlannerState stNeighborState;
                    stNeighborState.nGridIndex = nNeighborIdx;
                    stNeighborState.dGCost     = dTentativeGCost;
                    stNeighborState.dHCost =
                        m_dHeuristicWeight *
                        EuclideanDistance(nNeighborX * m_dGridResolution, nNeighborY * m_dGridResolution, nEndX * m_dGridResolution, nEndY * m_dGridResolution);

                    m_pqOpenSetNextBest.push(stNeighborState);
                }
            }

            // Consistently measure real-world time elapsed to terminate algorithms forcibly if CPU limits are breached.
            if (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - tmStartTime).count() >= m_dMaxSearchTimeSeconds)
            {
                LOG_WARNING(logging::g_qSharedLogger, "Grid search forcibly terminated due to exceeding the max search boundary of {} seconds.", m_dMaxSearchTimeSeconds);
                return;
            }
        }
    }

    /******************************************************************************
     * @brief Transforms the abstract 1D computational grid configurations logically
     * backward to rebuild a continuous stream of actionable UTM Geographic sequences.
     *
     * @return std::vector<geoops::Waypoint> - Sequentially ordered map of waypoints establishing optimal path structures.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-14
     ******************************************************************************/
    std::vector<geoops::Waypoint> GeoPlanner::ReconstructPath() const
    {
        std::vector<geoops::Waypoint> vPath;

        // Verify that the final path integration chain was actually established to the end node.
        if (m_vPredecessors[m_nEndIndex] == -1)
        {
            LOG_WARNING(logging::g_qSharedLogger, "Algorithmic resolution fault: Final path integration chain was never properly established.");
            return {};
        }

        int nCurrentIdx = m_nEndIndex;

        // Traverse the hierarchical sequence strictly backwards.
        while (nCurrentIdx != -1)
        {
            int nX, nY;
            GetGridCoords(nCurrentIdx, nX, nY);

            const GridCell& stCell = m_vCostmap[nCurrentIdx];

            // Reapply coordinate origins to map grid positions back to real-world UTM coordinates.
            double dEasting  = m_dGridOriginEasting + (nX * m_dGridResolution);
            double dNorthing = m_dGridOriginNorthing + (nY * m_dGridResolution);

            // Construct waypoint and append to vector.
            vPath.emplace_back(geoops::UTMCoordinate(dEasting, dNorthing, stCell.nZone, stCell.bInNorthernHemisphere, stCell.dAltitude),
                               geoops::WaypointType::eNavigationWaypoint,
                               m_dPathWaypointTolerance,
                               stCell.nClosestPointID);

            // Halt backtracing immediately upon intersecting initial origin index.
            if (nCurrentIdx == m_nStartIndex)
            {
                break;
            }

            nCurrentIdx = m_vPredecessors[nCurrentIdx];
        }

        // Reverse sequence to represent chronology from Start to Goal.
        std::reverse(vPath.begin(), vPath.end());

        return vPath;
    }

    /******************************************************************************
     * @brief Checks if a specific tile is loaded in the cache, and if not, fetches
     * the relevant LiDAR point cloud data into memory.
     *
     * @param nTileX - The exact coordinate identifier X block reference naturally mapping structurally.
     * @param nTileY - The exact coordinate identifier Y block reference naturally mapping structurally.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-14
     ******************************************************************************/
    void GeoPlanner::CheckAndLoadTile(int nTileX, int nTileY)
    {
        TileKey stTileKey{nTileX, nTileY};

        // Check if the tile is already present in the active cache map.
        if (m_umTileMapCache.find(stTileKey) == m_umTileMapCache.end())
        {
            // Set up point filter parameters logically outlining requested search area blocks.
            LiDARHandler::PointFilter stFilter;
            stFilter.dEasting  = (nTileX + 0.5) * m_dTileSize;
            stFilter.dNorthing = (nTileY + 0.5) * m_dTileSize;
            stFilter.dRadius   = std::sqrt(2) * (m_dTileSize / 2.0);

            // Retrieve the point cloud data for the tile from the LiDAR handler.
            std::vector<LiDARHandler::PointRow> vTilePoints = m_pLiDARHandler->GetLiDARData(stFilter);

            // Return if no points were found for the requested tile mapping boundaries.
            if (vTilePoints.empty())
            {
                return;
            }

            // Store the retrieved points natively into the unordered tile cache matrix.
            m_umTileMapCache[stTileKey] = std::move(vTilePoints);
        }
    }

    /******************************************************************************
     * @brief Performs morphological dilation on the costmap to fill in structural
     * voids or gaps caused by sparse LiDAR data collections.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-14
     ******************************************************************************/
    void GeoPlanner::FillGridHoles()
    {
        std::vector<GridCell> vNewCostmap = m_vCostmap;
        constexpr int anDx[8]             = {-1, 0, 1, -1, 1, -1, 0, 1};
        constexpr int anDy[8]             = {-1, -1, -1, 0, 0, 1, 1, 1};

        // Perform processing passes to iteratively stretch valid geographic bounds incrementally.
        for (int nPass = 0; nPass < m_nDilationPasses; ++nPass)
        {
            for (int nY = 0; nY < m_nGridHeight; ++nY)
            {
                for (int nX = 0; nX < m_nGridWidth; ++nX)
                {
                    int nIdx = GetGridIndex(nX, nY);

                    // Check if the target grid cell is currently an empty void.
                    if (m_vCostmap[nIdx].dTravScore < 0.0)
                    {
                        double dBestScore = -1.0;
                        GridCell stBestCell;

                        // Look at neighboring cells to find a valid traversal score to inherit.
                        for (int nI = 0; nI < 8; ++nI)
                        {
                            int nNeighborX = nX + anDx[nI];
                            int nNeighborY = nY + anDy[nI];

                            // Ensure neighbor indices are within valid 2D grid bounds.
                            if (nNeighborX >= 0 && nNeighborX < m_nGridWidth && nNeighborY >= 0 && nNeighborY < m_nGridHeight)
                            {
                                int nNeighborIdx = GetGridIndex(nNeighborX, nNeighborY);
                                if (m_vCostmap[nNeighborIdx].dTravScore > dBestScore)
                                {
                                    dBestScore = m_vCostmap[nNeighborIdx].dTravScore;
                                    stBestCell = m_vCostmap[nNeighborIdx];
                                }
                            }
                        }

                        // Inherit the best adjacent traversal score if a valid neighbor was discovered.
                        if (dBestScore >= 0.0)
                        {
                            vNewCostmap[nIdx] = stBestCell;
                        }
                    }
                }
            }
            // Update the core system array structures with the applied morphological filter outcomes.
            m_vCostmap = vNewCostmap;
        }
    }

    /******************************************************************************
     * @brief Unload tile LiDAR data
     *
     * @param minX - Minimum x coordinate of tile range.
     * @param maxX - Maximum x coordinate of tile range.
     * @param minY - Minimum y coordinate of tile range.
     * @param maxY - Maximum y coordinate of tile range.
     *
     * @author Sam Nolte (samnolte0302@gmail.com)
     * @date 2025-03-01
     ******************************************************************************/
    void GeoPlanner::UnloadLiDARTiles(double minX, double maxX, double minY, double maxY)
    {
        int nMinTileX = static_cast<int>(std::floor(minX / m_dTileSize));
        int nMinTileY = static_cast<int>(std::floor(minY / m_dTileSize));
        int nMaxTileX = static_cast<int>(std::floor(maxX / m_dTileSize));
        int nMaxTileY = static_cast<int>(std::floor(maxY / m_dTileSize));

        std::list<TileKey> tileKeys;
        for (int i = 0; i < nMaxTileX - nMinTileX + 1; ++i)
            for (int j = 0; j < nMaxTileY - nMinTileY + 1; ++j)
                tileKeys.push_back(TileKey{nMinTileX + i, nMinTileY + j});

        for (std::list<TileKey>::iterator it = tileKeys.begin(); it != tileKeys.end(); ++it)
        {
            // Make sure tile is actually loaded
            if (m_umTileMapCache.find(*it) == m_umTileMapCache.end())
            {
                continue;
            }

            m_umTileMapCache.erase(*it);
        }
    }

    /******************************************************************************
     * @brief Expands outward concentrically from a target grid cell to locate the
     * nearest neighboring cell that contains valid and safe traversal structures.
     *
     * @param nStartIndex - Origin array structural identifier originally derived from GPS.
     * @return int - Corrected viable mapping ID index, or -1 representing complete absence of viable data.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-14
     ******************************************************************************/
    int GeoPlanner::FindNearestValidCell(int nStartIndex) const
    {
        // Define an explicit threshold to prevent snapping the origin to extremely hazardous geometry.
        double dSafeThreshold = std::max(m_dMinTravScore, m_dSafeTravScoreThreshold);

        // Verify quickly if the requested starting index is already a valid and thoroughly safe cell.
        if (nStartIndex >= 0 && nStartIndex < static_cast<int>(m_vCostmap.size()) && m_vCostmap[nStartIndex].dTravScore >= 0.0 &&
            m_vCostmap[nStartIndex].dTravScore >= dSafeThreshold)
        {
            return nStartIndex;
        }

        int nStartX, nStartY;
        GetGridCoords(nStartIndex, nStartX, nStartY);

        int nMaxRadius            = m_nMaxSpiralSearchRadius;
        int nBestFallbackIdx      = -1;
        double dBestFallbackScore = -1.0;

        // Spiral search radially outward tracking concentric boundary edges locally.
        for (int nRadius = 1; nRadius <= nMaxRadius; ++nRadius)
        {
            int nBestIdxInRing      = -1;
            double dBestScoreInRing = -1.0;

            for (int nI = -nRadius; nI <= nRadius; ++nI)
            {
                for (int nJ = -nRadius; nJ <= nRadius; ++nJ)
                {
                    // Restrict processing exclusively to parameters located directly at the current radius boundary.
                    if (std::abs(nI) == nRadius || std::abs(nJ) == nRadius)
                    {
                        int nNeighborX = nStartX + nI;
                        int nNeighborY = nStartY + nJ;

                        if (nNeighborX >= 0 && nNeighborX < m_nGridWidth && nNeighborY >= 0 && nNeighborY < m_nGridHeight)
                        {
                            int nIdx      = GetGridIndex(nNeighborX, nNeighborY);
                            double dScore = m_vCostmap[nIdx].dTravScore;

                            if (dScore >= 0.0 && dScore >= m_dMinTravScore)
                            {
                                // Track the absolute best viable layout encountered overall.
                                if (dScore > dBestFallbackScore)
                                {
                                    dBestFallbackScore = dScore;
                                    nBestFallbackIdx   = nIdx;
                                }

                                // Prioritize and track specific indices exceeding the optimal safe threshold.
                                if (dScore >= dSafeThreshold && dScore > dBestScoreInRing)
                                {
                                    dBestScoreInRing = dScore;
                                    nBestIdxInRing   = nIdx;
                                }
                            }
                        }
                    }
                }
            }

            // Immediately yield the optimal safe coordinate natively if one was discovered in this ring.
            if (nBestIdxInRing != -1)
            {
                return nBestIdxInRing;
            }
        }

        // Return the highest scoring viable cell found within the search limits, or -1 if no usable data exists.
        return nBestFallbackIdx;
    }

    /******************************************************************************
     * @brief Calculates the standard 2D Euclidean distance between two geographic coordinates.
     * This establishes mathematically admissible baseline heuristic bounds for A*.
     *
     * @param dEasting1 - The numeric coordinate east bounds establishing start reference.
     * @param dNorthing1 - The numeric coordinate north bounds establishing start reference.
     * @param dEasting2 - The numeric coordinate east bounds establishing end reference.
     * @param dNorthing2 - The numeric coordinate north bounds establishing end reference.
     * @return double - The calculated spatial euclidean distance dimension.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-14
     ******************************************************************************/
    double GeoPlanner::EuclideanDistance(double dEasting1, double dNorthing1, double dEasting2, double dNorthing2) const
    {
        // Evaluate numerical differentials mapping bounds simply and efficiently.
        double dDiffX = dEasting1 - dEasting2;
        double dDiffY = dNorthing1 - dNorthing2;

        return std::sqrt(dDiffX * dDiffX + dDiffY * dDiffY);
    }
}    // namespace pathplanners
