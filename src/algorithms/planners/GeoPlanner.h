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
#include <OpenMS/DATASTRUCTURES/KDTree.h>
#include <RoveComm/RoveComm.h>
#include <RoveComm/RoveCommManifest.h>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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
     * @brief Small POD used for KDTree searches (2D point)
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-23
     ******************************************************************************/
    struct KDQueryPoint
    {
        public:
            double dEasting;
            double dNorthing;
    };

    /******************************************************************************
     * @brief Accessor for KDTree that exposes easting/northing for both PointRow and KDQueryPoint.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-09-23
     ******************************************************************************/
    struct PointKDAccessor
    {
        public:
            using result_type = double;

            inline result_type operator()(const LiDARHandler::PointRow& v, size_t idx) const { return (idx == 0) ? v.dEasting : v.dNorthing; }

            inline result_type operator()(const KDQueryPoint& v, size_t idx) const { return (idx == 0) ? v.dEasting : v.dNorthing; }
    };

    // KD-Tree typedef for 2D PointRows using our accessor.
    using KDTree2D = KDTree::KDTree<2, LiDARHandler::PointRow, PointKDAccessor>;

    /******************************************************************************
     * @brief This class implements a geospatial path planner that uses AStar's algorithm
     *       with a bias towards travel scores to find the optimal path between two points.
     *       geospatial data is fetched from the LidarHandler, and the path is planned
     *       using the Eigen library for matrix operations.
     *
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

            GeoPlanner(double dTileSize = 50.0);
            ~GeoPlanner();
            std::vector<geoops::Waypoint> PlanPath(LiDARHandler* pLiDARHandler,
                                                   const geoops::UTMCoordinate& stStart,
                                                   const geoops::UTMCoordinate& stEnd,
                                                   double dSearchRadius         = 2.0,
                                                   double dMaxSearchTimeSeconds = 240.0,
                                                   bool bPlotPath               = false);
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

            /*
             * AStar's algorithm related structs.
             */

            /******************************************************************************
             * @brief This struct represents the state of a node in the path planning algorithm.
             *
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-07-19
             ******************************************************************************/
            struct PlannerState
            {
                public:
                    int nID                    = -1;                                         // The unique identifier for the LiDAR point. (node)
                    double dEasting            = 0.0;                                        // The easting coordinate of the point.
                    double dNorthing           = 0.0;                                        // The northing coordinate of the point.
                    double dAltitude           = 0.0;                                        // The altitude of the point.
                    int nZone                  = 0;                                          // The UTM zone of the point.
                    bool bInNorthernHemisphere = true;                                       // Whether the point is in the northern hemisphere.
                    double dGCost              = std::numeric_limits<double>::infinity();    // The cost from the start node to this node.
                    double dHCost              = 0.0;                                        // The heuristic cost from this node to the end node.
            };

            /******************************************************************************
             * @brief This struct is used to compare two PlannerState objects based on their cost.
             *
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-07-19
             ******************************************************************************/
            struct PlannerStateCompare
            {
                public:
                    bool operator()(const PlannerState& stLeftHandSide, const PlannerState& stRightHandSide) const
                    {
                        // Tiebreaker: Compare based on cost, lower cost means higher priority. We want to keep the right-hand side if it has a lower cost.
                        return (stLeftHandSide.dGCost + stLeftHandSide.dHCost) > (stRightHandSide.dGCost + stRightHandSide.dHCost);
                    }
            };

            /*
             * Implicit graph representation with tiles.
             */

            /******************************************************************************
             * @brief This struct represents a tile key in the implicit graph representation.
             *      We divide the plan into a regular grid of tiles of size dTileSize*dTileSize meters.
             *      A tile covering the real-world X coordinate in [i*dTileSize, (i+1)*dTileSize) and
             *      Y coordinate in [j*dTileSize, (j+1)*dTileSize) is represented by the key (i, j).
             *
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-07-19
             ******************************************************************************/
            struct TileKey
            {
                public:
                    int nX;    // X coordinate of the tile.
                    int nY;    // Y coordinate of the tile.

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
             *      It combines the X and Y coordinates to create a unique hash for each tile key.
             *      We hash stKey.nX and stKey.nY separately and combine them using XOR and bit shifting.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-07-19
             ******************************************************************************/
            struct TileKeyHash
            {
                public:
                    size_t operator()(const TileKey& stKey) const noexcept
                    {
                        // Combine the X and Y ints into one size_t hash.
                        return (std::hash<int>()(stKey.nX) << 16) ^ std::hash<int>()(stKey.nY);
                    }
            };

            /******************************************************************************
             * @brief This struct is used to compare two TileKey objects for equality.
             *     After the hashbuckets narrow down candidates, we need to still confirm
             *    that the X and Y coordinates are equal to ensure they are the same tile.
             *
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-07-19
             ******************************************************************************/
            struct TileKeyEqual
            {
                public:
                    bool operator()(const TileKey& stLeftHandSide, const TileKey& stRightHandSide) const noexcept
                    {
                        // Check if both X and Y coordinates are equal.
                        return (stLeftHandSide.nX == stRightHandSide.nX) && (stLeftHandSide.nY == stRightHandSide.nY);
                    }
            };

            ////////////////////////////////////
            // Declare private methods.
            ////////////////////////////////////

            bool InitializeSearch(const geoops::UTMCoordinate& stStart, const geoops::UTMCoordinate& stEnd);
            void SearchAStar();
            std::vector<geoops::Waypoint> ReconstructPath() const;
            void CheckAndLoadTile(const PlannerState& stCurrentState);
            PlannerState FindClosestLiDARPoint(const geoops::UTMCoordinate& stCoordinate);
            void PlotPathAndTerrain(const std::vector<geoops::Waypoint>& vPath) const;
            double EuclideanDistance(double dEasting1, double dNorthing1, double dAltitude1, double dEasting2, double dNorthing2, double dAltitude2) const;

            /******************************************************************************
             * @brief Callback function used to set the minimum travel score for path planning.
             *
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-04-04
             ******************************************************************************/
            const std::function<void(const rovecomm::RoveCommPacket<float>&, const sockaddr_in&)> MinTravScore =
                [this](const rovecomm::RoveCommPacket<float>& stPacket, const sockaddr_in& stdAddr)
            {
                // Not using this.
                (void) stdAddr;

                // Set minimum travel score from incoming packet.
                if (stPacket.vData.size() > 0)
                {
                    // Set minimum travel score.
                    m_dMinTravScore = static_cast<double>(stPacket.vData[0]);
                    // Clear the tile cache.
                    this->ClearGeoCache();

                    // Submit logger message.
                    LOG_NOTICE(logging::g_qSharedLogger,
                               "Incoming Packet: Setting GeoPlanner minimum travel score to {}. The tile cache has also been cleared.",
                               this->m_dMinTravScore);
                }
            };

            /******************************************************************************
             * @brief Callback function used to set the beta bias for travel scores in path planning.
             *
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-04-04
             ******************************************************************************/
            const std::function<void(const rovecomm::RoveCommPacket<float>&, const sockaddr_in&)> BetaBias =
                [this](const rovecomm::RoveCommPacket<float>& stPacket, const sockaddr_in& stdAddr)
            {
                // Not using this.
                (void) stdAddr;

                // Set minimum travel score from incoming packet.
                if (stPacket.vData.size() > 0)
                {
                    m_dBeta = static_cast<double>(stPacket.vData[0]);
                    // Submit logger message.
                    LOG_NOTICE(logging::g_qSharedLogger, "Incoming Packet: Setting GeoPlanner beta bias to {}", this->m_dBeta);
                }
            };

            ////////////////////////////////////
            // Private member variables.
            ////////////////////////////////////

            int m_nStartID, m_nEndID;                                        // Unique identifiers for the start and end points.
            double m_dBeta;                                                  // Bias factor for travel scores.
            double m_dMinTravScore;                                          // Minimum travel score threshold for path planning.
            double m_dTileSize;                                              // Size of the grid tiles in meters.
            double m_dSearchRadius;                                          // Search radius for finding neighbors.
            double m_dMaxSearchTimeSeconds;                                  // The maximum time to spend searching for a path in seconds.
            LiDARHandler* m_pLiDARHandler;                                   // Pointer to the LiDARHandler instance for fetching geospatial data.
            std::unique_ptr<logging::graphing::PathTracer> m_pPathTracer;    // Path tracer for 3D visualization.
            std::unique_ptr<KDTree2D> m_pKDTree;                             // KD-tree for fast spatial queries over loaded tiles.
            std::mutex m_muPathGenMutex;                                     // Mutex to protect path planning operations.

            // AStar's algorithm related variables.

            /*
             * The core of AStar's algorithm is a priority queue (min-heap) that stores the open set of nodes to be evaluated.
             * The priority queue is ordered by the estimated total cost (f = g + h) of reaching the goal from the start node
             * through each node. We also maintain a map to track the best path to each node (predecessors) and a set of
             * nodes that have already been evaluated (closedSet).
             */
            std::priority_queue<PlannerState, std::vector<PlannerState>, PlannerStateCompare> m_pqOpenSetNextBest;    // Priority queue (min-heap) to be evaled.
            std::unordered_map<int, int> m_umPredecessors;                                                            // Maps point IDs to their predecessor's ID.
            std::unordered_set<int> m_usClosedSet;                                                                    // Set of point IDs that have been evaluated.
            std::unordered_map<int, PlannerState> m_umAllStates;    // Maps point IDs to their corresponding PlannerState.

            // Implicit graph representation with tiles.
            std::unordered_map<TileKey, std::vector<LiDARHandler::PointRow>, TileKeyHash, TileKeyEqual> m_umTileMapCache;    // Maps tile keys to LiDAR points.
            // KD-tree insertion bookkeeping to avoid duplicate inserts and to batch optimizations.
            std::unordered_set<TileKey, TileKeyHash, TileKeyEqual> m_usKDTreeInsertedTiles;    // Tracks which tiles' points have been inserted
            std::mutex m_kdTreeMutex;                                                          // Protects KD-tree and bookkeeping structures
    };
}    // namespace pathplanners

#endif    // GEOPLANNER_H
