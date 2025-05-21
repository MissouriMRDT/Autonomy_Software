/******************************************************************************
 * @brief Example usage of the LiDARHandler class.
 *
 * Demonstrates how to query a prebuilt SQLite LiDAR database for nearby
 * points within a specified radius of a UTM coordinate. Results are printed
 * to the console in a readable format.
 *
 * @file GetNearbyLiDAR.hpp
 * @author Eli Byrd
 * @date 2025-05-20
 ******************************************************************************/

#include "../../src/handlers/LiDARHandler.h"
#include "../../src/util/ExampleChecker.h"

/// \cond
#include <iostream>

/// \endcond

/******************************************************************************
 * @brief Example function to demonstrate the usage of LiDARHandler.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 ******************************************************************************/
void RunExample()
{
    // Path to the SQLite database file (relative to te build directory)
    const std::string szDBPath = "../data/LiDAR/data/sqlite/MDRS.db";

    // Create and initialize handler
    LiDARHandler handler;
    if (!handler.Initialize(szDBPath))
    {
        std::cerr << "Failed to initialize LiDARHandler.\n";
        return;
    }

    // Define test location and radius
    double dTestEasting  = 518011.14;
    double dTestNorthing = 4253985.0600000005;
    double dRadiusMeters = 5.0;

    std::cout << "Querying for points within " << dRadiusMeters << " meters of (" << dTestEasting << ", " << dTestNorthing << "):\n\n";

    // Execute query
    std::vector<LiDARHandler::PointRow> vPoints = handler.GetNearbyPoints(dTestEasting, dTestNorthing, dRadiusMeters);

    // Print results
    if (vPoints.empty())
    {
        std::cout << "No points found in that radius.\n";
    }
    else
    {
        // Loop through and print each point
        for (const LiDARHandler::PointRow& stPoint : vPoints)
        {
            std::cout << "ID: " << stPoint.id                               // ID of the point in the database
                      << " | Easting: " << stPoint.easting                  // UTM easting coordinate
                      << " | Northing: " << stPoint.northing                // UTM northing coordinate
                      << " | Altitude: " << stPoint.altitude                // Altitude in meters
                      << " | Zone: " << stPoint.zone                        // UTM zone (e.g., 14T)
                      << " | Class: " << stPoint.classification << "\n";    // Classification label (e.g., "Ground", "Vegetation")
        }

        std::cout << "\nQuery complete. " << vPoints.size() << " result(s) returned.\n";
    }
}
