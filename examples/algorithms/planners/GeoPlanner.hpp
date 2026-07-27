/******************************************************************************
 * @brief Example file for testing and demonstrating the GeoPlanner and LiDARHandler.
 *
 * @file GeoPlanner.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-31
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../../src/algorithms/planners/GeoPlanner.h"
#include "../../../src/handlers/LiDARHandler.h"
#include "../../../src/util/ExampleChecker.h"

/// \cond

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
    std::unique_ptr<LiDARHandler> pLiDARHandler = std::make_unique<LiDARHandler>();
    if (!pLiDARHandler->OpenDB("../data/LiDAR/data/databases/Fugitive.db"))
    {
        std::cerr << "Failed to initialize LiDARHandler.\n";
        return;
    }

    // Create and initialize the GeoPlanner.
    std::unique_ptr<pathplanners::GeoPlanner> pPlanner = std::make_unique<pathplanners::GeoPlanner>(50);

    geoops::UTMCoordinate stStart{614019.79, 4190069.29, 15, true};
    geoops::UTMCoordinate stEnd{614224.72, 4189924.76, 15, true};

    // Args are: LiDAR handler, start, end, search radius, max search time (s), corridor padding.
    std::vector<geoops::Waypoint> vPath = pPlanner->PlanPath(pLiDARHandler.get(), stStart, stEnd, 1000.0, 5.0, 0.0);

    // Print the number of waypoints in the path.
    std::cout << "Planned path with " << vPath.size() << " waypoints." << std::endl;
}
