/******************************************************************************
 * @brief Runtime LiDAR database query interface for autonomy systems.
 *
 * Provides spatial lookup capabilities against a preloaded DuckDB database
 * of USGS LAS 1.4 point cloud data. Enables nearby point lookup within a
 * radius from an (Easting, Northing) coordinate for real-time navigation.
 *
 * @file LiDARHandler.h
 * @author ClayJay3 (claytonraycowen@gmail.com), Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef LIDARHANDLER_H
#define LIDARHANDLER_H

#include "../util/GeospatialOperations.hpp"

/// \cond
#include <duckdb.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief The LiDARHandler class manages runtime queries against a LiDAR point
 * cloud database for autonomy navigation.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
class LiDARHandler
{
    public:
        ////////////////////////////////////
        // Declare and define structs
        ////////////////////////////////////

        /******************************************************************************
         * @brief Struct representing a single LiDAR point row from the database.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-01-22
         ******************************************************************************/
        struct PointRow
        {
            public:
                int nID;                         // Unique identifier for the point.
                double dEasting;                 // Easting coordinate in meters.
                double dNorthing;                // Northing coordinate in meters.
                double dAltitude;                // Altitude coordinate in meters.
                std::string szZone;              // UTM zone of the point.
                std::string szClassification;    // Classification of the point (e.g., ground, vegetation).
                double dNormalX;                 // X component of the normal vector.
                double dNormalY;                 // Y component of the normal vector.
                double dNormalZ;                 // Z component of the normal vector.
                double dSlope;                   // Slope of the point.
                double dRoughness;               // Roughness of the point.
                double dCurvature;               // Curvature of the point.
                double dTraversalScore;          // Traversal score for the point.
        };

        /******************************************************************************
         * @brief Struct for filtering LiDAR points during queries.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-01-22
         ******************************************************************************/
        struct PointFilter
        {
            public:
                double dEasting;                                               // Easting coordinate to filter points by.
                double dNorthing;                                              // Northing coordinate to filter points by.
                double dRadius;                                                // Radius in meters to filter points by.
                std::optional<std::string> szClassification = std::nullopt;    // Optional classification to filter points by.

                // Generic min/max pair for each filterable double property.
                template<typename T>
                struct Range
                {
                    public:
                        T tMin;
                        T tMax;
                };

                std::optional<Range<double>> dNormalX        = std::nullopt;    // Optional range for X component of the normal vector.
                std::optional<Range<double>> dNormalY        = std::nullopt;
                std::optional<Range<double>> dNormalZ        = std::nullopt;
                std::optional<Range<double>> dSlope          = std::nullopt;
                std::optional<Range<double>> dRoughness      = std::nullopt;
                std::optional<Range<double>> dCurvature      = std::nullopt;
                std::optional<Range<double>> dTraversalScore = std::nullopt;
        };

        ////////////////////////////////////
        // Declare class methods.
        ////////////////////////////////////

        LiDARHandler();
        LiDARHandler(const LiDARHandler& pOther)            = delete;
        LiDARHandler& operator=(const LiDARHandler& pOther) = delete;
        ~LiDARHandler();
        bool OpenDB(const std::string& szDBPath);
        bool CloseDB();
        std::vector<PointRow> GetLiDARData(const PointFilter& stPointFilter);

        ////////////////////////////////////
        // Getters
        ////////////////////////////////////

        bool IsDBOpen();

    private:
        ////////////////////////////////////
        // Private Methods
        ////////////////////////////////////

        template<typename T>
        void AddRangeFilter(std::vector<std::string>& vClauses,
                            duckdb::vector<duckdb::Value>& vBindValues,
                            const char* pColumn,
                            const std::optional<PointFilter::Range<T>>& stdOptRange);

        ////////////////////////////////////
        // Private Members
        ////////////////////////////////////

        std::unique_ptr<duckdb::DuckDB> m_pDB;          // DuckDB database instance.
        std::unique_ptr<duckdb::Connection> m_pConn;    // DuckDB connection wrapper.
        bool m_bIsDBOpen;
        std::shared_mutex m_muQueryMutex;               // Mutex for thread-safe access to the database.
};

#endif
