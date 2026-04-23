/******************************************************************************
 * @brief Define the GeoPlanner class
 *
 * @file GeoPlanner.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/
#ifndef GEOPLANNER_H
#define GEOPLANNER_H

#include "../../handlers/LiDARHandler.h"
#include "../../util/GeospatialOperations.hpp"
#include "../../util/logging/PathTracer.hpp"

/// \cond
#include <RoveComm/RoveComm.h>
#include <RoveComm/RoveCommManifest.h>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief This namespace stores classes, functions, and structs that are used to
 * implement different path planner algorithms used by the rover to determine
 * the optimal path to take for any given situation.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-01
 ******************************************************************************/
namespace pathplanners
{
    /******************************************************************************
     * @brief This class implements a geospatial path planner that uses a discrete
     * 2.5D Costmap and a fast Weighted A* algorithm with Kinematic Constraints
     * to find the optimal path through complex terrain between two coordinates.
     * All tunable heuristics and parameters are accessible via constructor initialization.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-07-14
     ******************************************************************************/
    class GeoPlanner
    {
        public:
            ////////////////////////////////////
            // Declare class methods.
            ////////////////////////////////////

            GeoPlanner(double dTileSize               = 50.0,
                       double dGridResolution         = 0.5,
                       double dHeuristicWeight        = 1.5,
                       double dBetaBias               = 1.0,
                       double dMinTravScore           = 0.0,
                       int nDilationPasses            = 2,
                       double dSafeTravScoreThreshold = 0.5,
                       int nMaxSpiralSearchRadius     = 20,
                       size_t siMaxPlotPointsPerTile  = 10,
                       double dPenaltyScalingFactor   = 10.0,
                       double dPenaltyPower           = 2.0,
                       double dPathWaypointTolerance  = 0.5,
                       double dPlotWaypointTolerance  = 0.01);
            ~GeoPlanner();

            std::vector<geoops::Waypoint> PlanPath(LiDARHandler* pLiDARHandler,
                                                   const geoops::UTMCoordinate& stStart,
                                                   const geoops::UTMCoordinate& stEnd,
                                                   double dSearchRadius         = 3.0,
                                                   double dMaxSearchTimeSeconds = 120.0,
                                                   bool bPlotPath               = false,
                                                   double dCorridorPadding      = 100.0);

            void ClearGeoCache();

            ////////////////////////////////////
            // Setters.
            ////////////////////////////////////

            void SetTileSize(double dTileSize);
            void SetMinTravScore(double dMinTravScore);
            void SetBetaBias(double dBetaBias);

            ////////////////////////////////////
            // Getters.
            ////////////////////////////////////

            double GetTileSize() const;
            double GetMinTravScore() const;
            double GetBetaBias() const;

        private:
            ////////////////////////////////////
            // Declare private class structs.
            ////////////////////////////////////

            /******************************************************************************
             * @brief Represents a single cell in our 2.5D discretized Costmap grid.
             * This is used to aggregate sparse point cloud data into a unified,
             * easy-to-search surface for the A* algorithm.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-07-14
             ******************************************************************************/
            struct GridCell
            {
                public:
                    double dTravScore          = -1.0;    // -1.0 indicates an empty/unknown void cell.
                    double dAltitude           = 0.0;     // Altitude of the terrain at this cell mapping.
                    int nClosestPointID        = -1;      // The raw LiDAR point ID that defined this cell.
                    int nZone                  = 0;       // UTM Zone for geographic boundary.
                    bool bInNorthernHemisphere = true;    // Hemisphere marker for accurate UTM conversion.
            };

            /******************************************************************************
             * @brief This struct represents the state of a node in the A* algorithm.
             * Optimized to work with a flat 1D vector instead of a hash map to prevent
             * heavy heap allocations and memory fragmentation.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-07-19
             ******************************************************************************/
            struct PlannerState
            {
                public:
                    int nGridIndex = -1;                                         // The unique identifier/index in the 1D costmap vector.
                    double dGCost  = std::numeric_limits<double>::infinity();    // The cost from the start node to this node.
                    double dHCost  = 0.0;                                        // The heuristic cost from this node to the end node.
            };

            /******************************************************************************
             * @brief This struct is used to compare two PlannerState objects based on their
             * cost to properly sort the priority queue.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-07-19
             ******************************************************************************/
            struct PlannerStateCompare
            {
                public:
                    bool operator()(const PlannerState& stLeftHandSide, const PlannerState& stRightHandSide) const
                    {
                        // Tiebreaker: Compare based on overall F-Cost, lower cost means higher priority in the Min-Heap.
                        return (stLeftHandSide.dGCost + stLeftHandSide.dHCost) > (stRightHandSide.dGCost + stRightHandSide.dHCost);
                    }
            };

            /******************************************************************************
             * @brief This struct represents a tile key in the implicit graph representation.
             * We divide the plan into a regular grid of tiles of size dTileSize*dTileSize meters.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-07-19
             ******************************************************************************/
            struct TileKey
            {
                public:
                    int nX;    // X coordinate of the tile block.
                    int nY;    // Y coordinate of the tile block.

                    /******************************************************************************
                     * @brief Overridden operator equals for TileKey struct.
                     *
                     * @param stOther - The other TileKey struct we are comparing to.
                     * @return true - The two TileKeys are equal.
                     * @return false - The two TileKeys are not equal.
                     *
                     * @author clayjay3 (claytonraycowen@gmail.com)
                     * @date 2025-07-31
                     ******************************************************************************/
                    bool operator==(const TileKey& stOther) const { return nX == stOther.nX && nY == stOther.nY; }
            };

            /******************************************************************************
             * @brief This struct is used to hash TileKey objects for use in unordered maps.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-07-19
             ******************************************************************************/
            struct TileKeyHash
            {
                public:
                    size_t operator()(const TileKey& stKey) const noexcept
                    {
                        // Combine the X and Y integers into one size_t hash using bit shifting.
                        return (std::hash<int>()(stKey.nX) << 16) ^ std::hash<int>()(stKey.nY);
                    }
            };

            /******************************************************************************
             * @brief This struct is used to compare two TileKey objects for equality after
             * a hash collision is triggered.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-07-19
             ******************************************************************************/
            struct TileKeyEqual
            {
                public:
                    bool operator()(const TileKey& stLeftHandSide, const TileKey& stRightHandSide) const noexcept
                    {
                        // Check if both X and Y coordinates are perfectly equal.
                        return (stLeftHandSide.nX == stRightHandSide.nX) && (stLeftHandSide.nY == stRightHandSide.nY);
                    }
            };

            ////////////////////////////////////
            // Declare private methods.
            ////////////////////////////////////

            bool PreloadCorridorAndBuildGrid(const geoops::UTMCoordinate& stStart, const geoops::UTMCoordinate& stEnd);
            void SearchAStar();
            std::vector<geoops::Waypoint> ReconstructPath() const;
            void CheckAndLoadTile(int nTileX, int nTileY);
            void FillGridHoles();
            int FindNearestValidCell(int nStartIndex) const;
            void PlotPathAndTerrain(const std::vector<geoops::Waypoint>& vPath) const;
            double EuclideanDistance(double dEasting1, double dNorthing1, double dEasting2, double dNorthing2) const;

            /******************************************************************************
             * @brief Inline helper to convert 2D grid coordinates to a 1D vector index.
             *
             * @param nX - The X grid coordinate.
             * @param nY - The Y grid coordinate.
             * @return int - The corresponding flat 1D vector index.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-07-14
             ******************************************************************************/
            inline int GetGridIndex(int nX, int nY) const { return nY * m_nGridWidth + nX; }

            /******************************************************************************
             * @brief Inline helper to convert a 1D vector index back into 2D grid coordinates.
             *
             * @param nIndex - The 1D vector index to decode.
             * @param nX - Reference to store the resultant X coordinate.
             * @param nY - Reference to store the resultant Y coordinate.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-07-14
             ******************************************************************************/
            inline void GetGridCoords(int nIndex, int& nX, int& nY) const
            {
                nY = nIndex / m_nGridWidth;
                nX = nIndex % m_nGridWidth;
            }

            /******************************************************************************
             * @brief Callback function used to set the minimum travel score for path planning.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-04-04
             ******************************************************************************/
            const std::function<void(const rovecomm::RoveCommPacket<float>&, const sockaddr_in&)> fnMinTravScoreCallback =
                [this](const rovecomm::RoveCommPacket<float>& stPacket, const sockaddr_in& stdAddr)
            {
                (void) stdAddr;

                // Extract minimum travel score from incoming packet.
                if (stPacket.vData.size() > 0)
                {
                    m_dMinTravScore = static_cast<double>(stPacket.vData[0]);
                    this->ClearGeoCache();

                    LOG_NOTICE(logging::g_qSharedLogger,
                               "Incoming Packet: Setting GeoPlanner minimum travel score to {}. The tile cache has also been cleared.",
                               this->m_dMinTravScore);
                }
            };

            /******************************************************************************
             * @brief Callback function used to set the beta bias for travel scores in path planning.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-04-04
             ******************************************************************************/
            const std::function<void(const rovecomm::RoveCommPacket<float>&, const sockaddr_in&)> fnBetaBiasCallback =
                [this](const rovecomm::RoveCommPacket<float>& stPacket, const sockaddr_in& stdAddr)
            {
                (void) stdAddr;

                // Extract beta bias from incoming packet.
                if (stPacket.vData.size() > 0)
                {
                    m_dBeta = static_cast<double>(stPacket.vData[0]);

                    LOG_NOTICE(logging::g_qSharedLogger, "Incoming Packet: Setting GeoPlanner beta bias to {}", this->m_dBeta);
                }
            };

            ////////////////////////////////////
            // Private member variables.
            ////////////////////////////////////

            // Extracted algorithmic magic numbers / Configurable parameters
            double m_dTileSize;                  // Size of the database grid tiles in meters.
            double m_dGridResolution;            // Metric size of each cell in the discrete costmap (e.g., 0.5m).
            double m_dHeuristicWeight;           // Multiplier for Weighted A* to rapidly accelerate goal-seeking behavior.
            double m_dBeta;                      // Bias factor exponent for traversal score penalties.
            double m_dMinTravScore;              // Minimum traversal score threshold required for a cell to be considered traversable.
            int m_nDilationPasses;               // Number of morphological passes used to bridge sparse LiDAR voids.
            double m_dSafeTravScoreThreshold;    // Minimum score permitted to establish a structurally "safe" origin for snapping.
            int m_nMaxSpiralSearchRadius;        // Maximum rings to expand outward to snap origin to a valid terrain location.
            size_t m_siMaxPlotPointsPerTile;     // Subsampling limit threshold to prevent plot renderer overload.
            double m_dPenaltyScalingFactor;      // Multiplier determining severity of avoidance behavior during A* traversal score checks.
            double m_dPenaltyPower;              // Exponential power applied to scores to make dangerous zones drastically more costly.
            double m_dPathWaypointTolerance;     // Tolerance radius encoded into resultant path waypoint structures.
            double m_dPlotWaypointTolerance;     // Tolerance radius encoded into visual tracer waypoint parameters.

            // Request-specific variables configured during PlanPath
            double m_dSearchRadius;            // Contextual radius for KDTree legacy logic/bounding box padding base.
            double m_dMaxSearchTimeSeconds;    // The absolute maximum CPU time to spend searching for a path in seconds.
            double m_dCorridorPadding;         // Corridor padding radius used to expand the bounding box logic.

            // Resource pointers and synchronization
            LiDARHandler* m_pLiDARHandler;                                   // Pointer to the LiDARHandler instance for fetching geospatial data point clouds.
            std::unique_ptr<logging::graphing::PathTracer> m_pPathTracer;    // Pointer to the path tracer for 3D trajectory visualization.
            std::mutex m_muPathGenMutex;                                     // Mutex to protect concurrent path planning operations.

            // Costmap Grid positional state
            double m_dGridOriginEasting;         // Bottom-left UTM easting bound of the generated grid.
            double m_dGridOriginNorthing;        // Bottom-left UTM northing bound of the generated grid.
            int m_nGridWidth;                    // Total width of the grid in cell count.
            int m_nGridHeight;                   // Total height of the grid in cell count.
            int m_nStartIndex;                   // Flat 1D index of the validated start cell position.
            int m_nEndIndex;                     // Flat 1D index of the validated end cell position.

            std::vector<GridCell> m_vCostmap;    // Flat 1D contiguous vector representing the 2.5D environment for highly optimized caching.

            // Fast A* State Trackers (Using flat contiguous vectors instead of heavily fragmented Hash Maps)
            std::priority_queue<PlannerState, std::vector<PlannerState>, PlannerStateCompare> m_pqOpenSetNextBest;    // The min-heap of active frontier nodes.
            std::vector<int> m_vPredecessors;    // Tracks the index of the parent node to reconstruct the final path.
            std::vector<bool> m_vbClosedSet;     // Tracks which grid cells have already been fully evaluated by the algorithm.
            std::vector<double> m_vdGCosts;      // Stores the best known G-Cost to reach each specific cell index.

            // Implicit graph representation database caches.
            std::unordered_map<TileKey, std::vector<LiDARHandler::PointRow>, TileKeyHash, TileKeyEqual>
                m_umTileMapCache;    // Maps tile keys to arrays of raw LiDAR points.
    };
}    // namespace pathplanners

#endif    // GEOPLANNER_H