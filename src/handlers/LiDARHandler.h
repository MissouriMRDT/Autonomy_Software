/******************************************************************************
 * @brief Runtime LiDAR database query interface for autonomy systems.
 *
 * Provides spatial lookup capabilities against a preloaded SQLite database
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

/// \cond
#include <sqlite3.h>
#include <string>
#include <vector>

/// \endcond

class LiDARHandler
{
    public:
        ////////////////////////////////////
        // Declare the PointRow structure to hold point data.
        ////////////////////////////////////

        struct PointRow
        {
            public:
                int nId;
                double dEasting;
                double dNorthing;
                double dAltitude;
                std::string szZone;
                std::string szClassification;
        };

        ////////////////////////////////////
        // Declare class methods.
        ////////////////////////////////////
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
