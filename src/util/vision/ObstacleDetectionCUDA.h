/******************************************************************************
 * @brief The header file for the obstacle detection CUDA kernel.
 *
 * @file ObstacleDetectionCUDA.h
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2026-05-11
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/
#ifndef OBSTACLE_DETECTION_CUDA_H
#define OBSTACLE_DETECTION_CUDA_H

#include "../../util/GeospatialOperations.hpp"
#include <cuda_runtime.h>
#include <vector>

/******************************************************************************
 * @brief Namespace for object detection utilities.
 *
 *
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2026-05-11
 ******************************************************************************/
namespace objectdetectutils
{
    // A standard C++ declaration. The NVCC compiler will handle the implementation.
    std::vector<geoops::UTMCoordinate> ExtractObstaclesCUDA(float4* d_f4PointCloud,
                                                            int nTotalPoints,
                                                            const geoops::RoverPose& stCurrentPose,
                                                            double dGridCellSize,
                                                            double dObstacleVarianceThreshold);
}    // namespace objectdetectutils

#endif    // OBSTACLE_DETECTION_CUDA_H
