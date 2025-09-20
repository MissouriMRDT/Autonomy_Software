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
#include <queue>
#include <unordered_map>
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
     * @brief This class implements a geospatial path planner that uses Dijkstra's algorithm
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

            GeoPlanner(double dTileSize = 5.0);
            ~GeoPlanner();
            std::vector<geoops::Waypoint> PlanPath(LiDARHandler* pLiDARHandler,
                                                   const geoops::UTMCoordinate& stStart,
                                                   const geoops::UTMCoordinate& stEnd,
                                                   double dBeta         = 1.0,
                                                   double dSearchRadius = 3.0,
                                                   double dMinTravScore = 0.8,
                                                   bool bPlotPath       = false);

        private:
            ////////////////////////////////////
            // Declare private class structs.
            ////////////////////////////////////

            /*
             * Dijkstra's algorithm related structs.
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
                    int nID;                                  // The unique identifier for the LiDAR point. (node)
                    double dEasting, dNorthing, dAltitude;    // The easting, northing, and altitude coordinates of the node.
                    double dCost = 0.0;                       // The best known total cost to reach this node so far.
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
                        // Compare based on cost, lower cost means higher priority.
                        return stLeftHandSide.dCost > stRightHandSide.dCost;
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

            void InitializeSearch(const geoops::UTMCoordinate& stStart, const geoops::UTMCoordinate& stEnd);
            void SearchDijkstra();
            void ProcessNeighbors(const PlannerState& stCurrentState);
            void RelaxEdge(const PlannerState& stCurrentState, const LiDARHandler::PointRow& stNeighborPoint, double dDistance);
            std::vector<int> ReconstructPath() const;
            void CheckAndLoadTile(const PlannerState& stCurrentState);
            PlannerState FindClosestLiDARPoint(const geoops::UTMCoordinate& stCoordinate);
            void PlotPathAndTerrain(const std::vector<geoops::Waypoint>& vPath) const;

            ////////////////////////////////////
            // Private member variables.
            ////////////////////////////////////

            int m_nStartID, m_nEndID;                                        // Unique identifiers for the start and end points.
            double m_dBeta;                                                  // Bias factor for travel scores.
            double m_dMinTravScore;                                          // Minimum travel score threshold for path planning.
            double m_dTileSize;                                              // Size of the grid tiles in meters.
            double m_dSearchRadius;                                          // Search radius for finding neighbors.
            LiDARHandler* m_pLiDARHandler;                                   // Pointer to the LiDARHandler instance for fetching geospatial data.
            std::unique_ptr<logging::graphing::PathTracer> m_pPathTracer;    // Path tracer for 3D visualization.

            // Dijkstra's algorithm related variables.

            /*
             * The core of Dijkstra's algorithm is a priority queue that stores the states to be explored.
             * In other words, this is the frontiers of the search.
             * The open set is implemented as a priority queue, where the highest priority
             * is given to the node with the lowest cost. This  queue always contains the unsettled nodes
             * that are candidates for exploration and each time through the loop, the node with the lowest cost is selected for expansion.
             */
            std::priority_queue<PlannerState, std::vector<PlannerState>, PlannerStateCompare> m_pqOpenSet;
            std::unordered_map<int, double> m_umCosts;        // Maps node IDs to their best known costs.
            std::unordered_map<int, int> m_umPredecessors;    // Maps node IDs to their predecessors in the path.
            std::unordered_set<int> m_usClosedSet;            // Maps node IDs to whether they have been processed.

            // Implicit graph representation with tiles.
            std::unordered_map<TileKey, std::vector<LiDARHandler::PointRow>, TileKeyHash, TileKeyEqual> m_umTileMapCache;    // Maps tile keys to LiDAR points.
    };
}    // namespace pathplanners

#endif    // GEOPLANNER_H
