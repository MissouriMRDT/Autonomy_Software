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
    if (!pLiDARHandler->OpenDB("../data/LiDAR/data/databases/Rolla.db"))
    {
        std::cerr << "Failed to initialize LiDARHandler.\n";
        return;
    }

    // Create and initialize the GeoPlanner.
    std::unique_ptr<pathplanners::GeoPlanner> pPlanner = std::make_unique<pathplanners::GeoPlanner>(10);

    geoops::UTMCoordinate stStart{606977.35, 4201066.42, 15, true};
    geoops::UTMCoordinate stEnd{606626.97, 4200711.05, 15, true};

    pPlanner->PlanPath(pLiDARHandler.get(), stStart, stEnd, 1.0, 0.5, 0.0, true);
}
