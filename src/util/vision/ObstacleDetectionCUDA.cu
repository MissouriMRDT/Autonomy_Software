/******************************************************************************
 * @brief Implements the obstacle detection kernel.
 *
 * @file ObstacleDetectionCUDA.cu
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2026-05-11
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/
#include "../NumberOperations.hpp"
#include "ObstacleDetectionCUDA.h"
#include <cfloat>
#include <cmath>

/******************************************************************************
 * @brief - Custom atomicMax for floats (CUDA natively only supports atomic for integers).
 *
 * @param pAddress - The address.
 * @param fVal - The float value.
 * @return __device__ - The max float value.
 *
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2026-05-11
 ******************************************************************************/
__device__ static float atomicMaxFloat(float* pAddress, float fVal)
{
    int* pAddressAsInt = (int*) pAddress;
    int nOld           = *pAddressAsInt;
    int nAssumed;
    do
    {
        nAssumed = nOld;
        nOld     = atomicCAS(pAddressAsInt, nAssumed, __float_as_int(fmaxf(fVal, __int_as_float(nAssumed))));
    } while (nAssumed != nOld);
    return __int_as_float(nOld);
}

// Custom atomicMin for floats
/******************************************************************************
 * @brief - Returns the minimum int as a float.
 *
 * @param pAddress - The address.
 * @param fVal - The float value.
 * @return __device__ -
 *
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2026-05-11
 ******************************************************************************/
__device__ static float atomicMinFloat(float* pAddress, float fVal)
{
    int* pAddressAsInt = (int*) pAddress;
    int nOld           = *pAddressAsInt;
    int nAssumed;
    do
    {
        nAssumed = nOld;
        nOld     = atomicCAS(pAddressAsInt, nAssumed, __float_as_int(fminf(fVal, __int_as_float(nAssumed))));
    } while (nAssumed != nOld);
    return __int_as_float(nOld);
}

/******************************************************************************
 * @brief - The GPU Shader Kernel.
 *
 * @param d_f4PointCloud - The point cloud.
 * @param nTotalPoints - The total number of points.
 * @param d_fGridMinY - The minimum grid value in Y axis.
 * @param d_fGridMaxY - The maximum grid value in Y axis.
 * @param fCellSize - The size of each cell.
 * @param nGridWidth - The width of the grid.
 * @param fCosHeading - The cosine heading.
 * @param fSinHeading - The sine heading.
 * @param dRoverEasting - The rover's easting position.
 * @param dRoverNorthing - The rover's northing position.
 * @param dRoverAltitude - The rover's altitude.
 * @return __global__ - Nothing - calculate the variance grid kernel.
 *
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2026-05-11
 ******************************************************************************/
__global__ void CalculateVarianceGridKernel(const float4* d_f4PointCloud,
                                            int nTotalPoints,
                                            float* d_fGridMinY,
                                            float* d_fGridMaxY,
                                            float fCellSize,
                                            int nGridWidth,
                                            float fCosHeading,
                                            float fSinHeading,
                                            double dRoverEasting,
                                            double dRoverNorthing,
                                            double dRoverAltitude)
{
    // Grab the index.
    int nIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (nIdx >= nTotalPoints)
    {
        return;
    }

    // Extract Left-Handed Y-Up points directly from VRAM.
    float4 f4Point = d_f4PointCloud[nIdx];
    float fLocalX  = f4Point.x;
    float fLocalY  = f4Point.y;
    float fLocalZ  = f4Point.z;

    if (isnan(fLocalZ) || fLocalZ <= 0.0f)
    {
        return;
    }

    // Transform to Global UTM.
    double dEasting  = dRoverEasting + (fLocalZ * fCosHeading + fLocalX * fSinHeading);
    double dNorthing = dRoverNorthing + (fLocalZ * fSinHeading - fLocalX * fCosHeading);
    double dAltitude = dRoverAltitude + fLocalY;

    // Map to local 2D Grid Array around the rover.
    int nGridX   = static_cast<int>(floorf((dEasting - dRoverEasting) / fCellSize));
    int nGridY   = static_cast<int>(floorf((dNorthing - dRoverNorthing) / fCellSize));

    int nOffsetX = nGridX + (nGridWidth / 2);
    int nOffsetY = nGridY + (nGridWidth / 2);

    if (nOffsetX >= 0 && nOffsetX < nGridWidth && nOffsetY >= 0 && nOffsetY < nGridWidth)
    {
        int nFlatGridIdx = (nOffsetY * nGridWidth) + nOffsetX;
        atomicMinFloat(&d_fGridMinY[nFlatGridIdx], static_cast<float>(dAltitude));
        atomicMaxFloat(&d_fGridMaxY[nFlatGridIdx], static_cast<float>(dAltitude));
    }
}

/******************************************************************************
 * @brief - Defines the namespace for object detection utilities.
 *
 *
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2026-05-11
 ******************************************************************************/
namespace objectdetectutils
{
    /******************************************************************************
     * @brief - Will extract the obstacles from the CUDA kernel.
     *
     * @param d_f4PointCloud - The point cloud.
     * @param nTotalPoints - The total number of points.
     * @param stCurrentPose - The current rover pose.
     * @param dGridCellSize - The size of the grid cells.
     * @param dObstacleVarianceThreshold - The variance threshold to be considered an object.
     * @return std::vector<geoops::UTMCoordinate> - The UTM coordinate of obstacles.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-05-11
     ******************************************************************************/
    std::vector<geoops::UTMCoordinate> ExtractObstaclesCUDA(float4* d_f4PointCloud,
                                                            int nTotalPoints,
                                                            const geoops::RoverPose& stCurrentPose,
                                                            double dGridCellSize,
                                                            double dObstacleVarianceThreshold)
    {
        // Define a roughly 25m x 25m local grid.
        int nGridWidth  = static_cast<int>(25.0 / dGridCellSize);
        int nTotalCells = nGridWidth * nGridWidth;

        // Allocate Host (CPU) memory for the final bounds.
        std::vector<float> vGridMinY(nTotalCells, FLT_MAX);
        std::vector<float> vGridMaxY(nTotalCells, -FLT_MAX);

        // Allocate Device (GPU) memory.
        float* d_fGridMinY;
        float* d_fGridMaxY;
        cudaMalloc(&d_fGridMinY, nTotalCells * sizeof(float));
        cudaMalloc(&d_fGridMaxY, nTotalCells * sizeof(float));

        // Copy initial values to GPU.
        cudaMemcpy(d_fGridMinY, vGridMinY.data(), nTotalCells * sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(d_fGridMaxY, vGridMaxY.data(), nTotalCells * sizeof(float), cudaMemcpyHostToDevice);

        // Calculate heading.
        double dAdjustedHeading                 = numops::InputAngleModulus((stCurrentPose.GetCompassHeading() * -1.0) + 90.0, 0.0, 360.0);
        double dHeadingRad                      = dAdjustedHeading * M_PI / 180.0;

        const geoops::UTMCoordinate& stRoverUTM = stCurrentPose.GetUTMCoordinate();

        // Launch Kernel.
        int nThreadsPerBlock = 256;
        int nBlocks          = (nTotalPoints + nThreadsPerBlock - 1) / nThreadsPerBlock;

        CalculateVarianceGridKernel<<<nBlocks, nThreadsPerBlock>>>(d_f4PointCloud,
                                                                   nTotalPoints,
                                                                   d_fGridMinY,
                                                                   d_fGridMaxY,
                                                                   static_cast<float>(dGridCellSize),
                                                                   nGridWidth,
                                                                   static_cast<float>(cos(dHeadingRad)),
                                                                   static_cast<float>(sin(dHeadingRad)),
                                                                   stRoverUTM.dEasting,
                                                                   stRoverUTM.dNorthing,
                                                                   stRoverUTM.dAltitude);

        // Wait for GPU to finish.
        cudaDeviceSynchronize();

        // Copy the incredibly small grid arrays back to CPU.
        cudaMemcpy(vGridMinY.data(), d_fGridMinY, nTotalCells * sizeof(float), cudaMemcpyDeviceToHost);
        cudaMemcpy(vGridMaxY.data(), d_fGridMaxY, nTotalCells * sizeof(float), cudaMemcpyDeviceToHost);

        // Free GPU memory.
        cudaFree(d_fGridMinY);
        cudaFree(d_fGridMaxY);

        std::vector<geoops::UTMCoordinate> vObstacles;

        // Extract obstacles from the tiny CPU grid array.
        for (int nY = 0; nY < nGridWidth; ++nY)
        {
            for (int nX = 0; nX < nGridWidth; ++nX)
            {
                int nIdx    = (nY * nGridWidth) + nX;
                float fMinY = vGridMinY[nIdx];
                float fMaxY = vGridMaxY[nIdx];

                if (fMinY != FLT_MAX && fMaxY != -FLT_MAX)
                {
                    if ((fMaxY - fMinY) > static_cast<float>(dObstacleVarianceThreshold))
                    {
                        // Reverse the array offset to get real global coordinates.
                        double dObsEasting  = stRoverUTM.dEasting + ((nX - (nGridWidth / 2)) * dGridCellSize);
                        double dObsNorthing = stRoverUTM.dNorthing + ((nY - (nGridWidth / 2)) * dGridCellSize);

                        vObstacles.emplace_back(dObsEasting, dObsNorthing, stRoverUTM.nZone, stRoverUTM.bWithinNorthernHemisphere, stRoverUTM.dAltitude);
                    }
                }
            }
        }

        return vObstacles;
    }
}    // namespace objectdetectutils
