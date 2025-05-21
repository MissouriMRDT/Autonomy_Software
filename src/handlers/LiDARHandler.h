/******************************************************************************
 * @brief Runtime LiDAR database query interface for autonomy systems.
 *
 * Provides spatial lookup capabilities against a preloaded SQLite database
 * of USGS LAS 1.4 point cloud data. Enables nearby point lookup within a
 * radius from an (Easting, Northing) coordinate for real-time navigation.
 *
 * @file LiDARHandler.h
 * @author Eli Byrd
 * @date 2025-05-20
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef LIDARHANDLER_H
#define LIDARHANDLER_H

/// \cond
#include <sqlite3.h>
#include <string>
#include <vector>

/// \endcond

class LiDARHandler
{
    public:
        ////////////////////////////////////
        // Structures for LAS 1.4
        ////////////////////////////////////

        /******************************************************************************
         * @brief Structure representing a single parsed LiDAR point.
         *
         * Contains id, easting, northing, altitude, UTM zone, and classification label.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-05-20
         ******************************************************************************/
        struct PointRow
        {
                int nId;
                double dEasting;
                double dNorthing;
                double dAltitude;
                std::string szZone;
                std::string szClassification;
        };

        ////////////////////////////////////
        // Constructors and Destructors
        ////////////////////////////////////

        /******************************************************************************
         * @brief Construct a new LiDAR Loader object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-05-20
         ******************************************************************************/
        LiDARHandler()                                      = default;
        LiDARHandler(const LiDARHandler& pOther)            = delete;
        LiDARHandler& operator=(const LiDARHandler& pOther) = delete;
        ~LiDARHandler();

        ////////////////////////////////////
        // Public Methods
        ////////////////////////////////////
        bool Initialize(const std::string& szDBPath);
        std::vector<PointRow> GetNearbyPoints(double dEasting, double dNorthing, double dRadiusMeters = 5.0);

    private:
        ////////////////////////////////////
        // Private Members
        ////////////////////////////////////
        sqlite3* m_pSQLDatabase       = nullptr;
        sqlite3_stmt* m_pSQLStatement = nullptr;

        ////////////////////////////////////
        // Private Methods
        ////////////////////////////////////
        bool PrepareNearbyStatement();
        void Finalize();
};

#endif
