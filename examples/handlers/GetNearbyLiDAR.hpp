/******************************************************************************
 * @brief Example usage of the LiDARHandler class.
 *
 * Demonstrates how to query a prebuilt SQLite LiDAR database for nearby
 * points within a specified radius of a UTM coordinate. Results are printed
 * to the console in a readable format.
 *
 * @file GetNearbyLiDAR.hpp
 * @author ClayJay3 (claytonraycowen@gmail.com), Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 ******************************************************************************/

#include "../../src/AutonomyConstants.h"
#include "../../src/AutonomyLogging.h"
#include "../../src/handlers/LiDARHandler.h"
#include "../../src/util/ExampleChecker.h"

/// \cond
#include <iostream>

/// \endcond

/******************************************************************************
 * @brief Example function to demonstrate the usage of LiDARHandler.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com), Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 ******************************************************************************/
void RunExample()
{
    // Create and initialize handler
    LiDARHandler handler;
    if (!handler.OpenDB(constants::LIDAR_HANDLER_DB_PATH))
    {
        std::cerr << "Failed to initialize LiDARHandler.\n";
        return;
    }

    // Define test location and radius
    double dTestEasting  = 614058.84;
    double dTestNorthing = 4189968.85;
    double dRadiusMeters = 3.0;
    double dMinTravScore = 0.95;

    LOG_INFO(logging::g_qSharedLogger, "Querying for points within {} meters of ({}, {}, {}):", dRadiusMeters, dTestEasting, dTestNorthing, dMinTravScore);

    // Execute query
    std::vector<LiDARHandler::PointRow> vPoints =
        handler.GetLiDARData({.dEasting        = dTestEasting,
                              .dNorthing       = dTestNorthing,
                              .dRadius         = dRadiusMeters,
                              .dTraversalScore = std::optional<LiDARHandler::PointFilter::Range<double>>({dMinTravScore, 1.0})});

    // Print results
    if (vPoints.empty())
    {
        LOG_INFO(logging::g_qSharedLogger, "No points found in that radius.");
    }
    else
    {
        // Loop through and print each point
        for (const LiDARHandler::PointRow& stPoint : vPoints)
        {
            // Assemble a string to print point data.
            std::string szPointInfo = "Point ID: " + std::to_string(stPoint.nID) + "\n";
            szPointInfo += "Easting: " + std::to_string(stPoint.dEasting) + "\n";
            szPointInfo += "Northing: " + std::to_string(stPoint.dNorthing) + "\n";
            szPointInfo += "Altitude: " + std::to_string(stPoint.dAltitude) + "\n";
            szPointInfo += "Zone: " + stPoint.szZone + "\n";
            szPointInfo += "Classification: " + stPoint.szClassification + "\n";
            szPointInfo +=
                "Normal Vector: (" + std::to_string(stPoint.dNormalX) + ", " + std::to_string(stPoint.dNormalY) + ", " + std::to_string(stPoint.dNormalZ) + ")\n";
            szPointInfo += "Slope: " + std::to_string(stPoint.dSlope) + "\n";
            szPointInfo += "Roughness: " + std::to_string(stPoint.dRoughness) + "\n";
            szPointInfo += "Curvature: " + std::to_string(stPoint.dCurvature) + "\n";
            szPointInfo += "Traversal Score: " + std::to_string(stPoint.dTraversalScore) + "\n";
            szPointInfo += "-----------------------------------\n";

            // Submit logger message.
            LOG_NOTICE(logging::g_qSharedLogger, "{}", szPointInfo);
        }

        LOG_INFO(logging::g_qSharedLogger, "Query complete. {} result(s) returned.", vPoints.size());
    }
}
