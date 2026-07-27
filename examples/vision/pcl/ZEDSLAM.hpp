/******************************************************************************
 * @brief Example file that demonstrates opening and using multiple different
 *      features of the ZED camera and performing a basic SLAM using PCL.
 *
 * @file OpenZEDCam_SLAM.cpp
 * @author clayjay3
 * @date 2023-09-16
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../src/AutonomyConstants.h"
#include "../../../src/AutonomyGlobals.h"
#include "../../../src/AutonomyLogging.h"
#include "../../../src/AutonomyNetworking.h"
#include "../../../src/util/ExampleChecker.h"
#include "../../../src/util/vision/ImageOperations.hpp"

// Standard includes
#include <chrono>
#include <future>

// Include OpenCV headers.
#include <opencv2/core/cuda.hpp>
#include <opencv2/opencv.hpp>

// Include PCL headers.
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

// Declare file constants.
const bool ENABLE_SPATIAL_MAPPING = false;

/******************************************************************************
 * @brief Converts a cv::Mat to a pcl::PointCloud<pcl::PointXYZ>::Ptr.
 *
 * @param cvPointCloud - The cv::Mat to convert.
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr - The converted point cloud.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-03-05
 ******************************************************************************/
pcl::PointCloud<pcl::PointXYZ>::Ptr convertCVMatToPCL(const cv::Mat& cvPointCloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Loop over all pixels
    for (int i = 0; i < cvPointCloud.rows; ++i)
    {
        for (int j = 0; j < cvPointCloud.cols; ++j)
        {
            // Access the 4-channel float vector.
            cv::Vec4f point = cvPointCloud.at<cv::Vec4f>(i, j);
            // Check for valid (finite) points.
            if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]))
            {
                pcl::PointXYZ pclPoint;
                pclPoint.x = point[0];
                pclPoint.y = point[1];
                pclPoint.z = point[2];
                cloud->points.push_back(pclPoint);
            }
        }
    }
    cloud->width  = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    return cloud;
}

/******************************************************************************
 * @brief Main example function.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-03-05
 ******************************************************************************/
void RunExample()
{
    // Initialize and start handlers.
    globals::g_pCameraHandler = new CameraHandler();

    // Get pointer to camera.
    std::shared_ptr<ZEDCamera> pExampleZEDCam1 = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam);
    // Start ZED cam.
    pExampleZEDCam1->Start();

    // Turn on ZED features.
    pExampleZEDCam1->EnablePositionalTracking();
    // Check if we should turn on spatial mapping.
    if (ENABLE_SPATIAL_MAPPING)
    {
        // Enable spatial mapping.
        pExampleZEDCam1->EnableSpatialMapping();
    }

    // Whether this camera hands out GPU or CPU mats. Decides which channels we subscribe to.
    const bool bUsingGPUMem = pExampleZEDCam1->GetUsingGPUMem();

    // Register demand for every channel we read. The camera retrieves NOTHING for a channel with
    // no subscribers, so these handles are what actually turn each retrieval on. Hold them for as
    // long as we want the data.
    // Only the handle matching the camera's memory mode is taken; the other stays
    // default constructed and inactive.
    pubsub::Reader<cv::Mat> subFrameCPU;
    pubsub::Reader<cv::cuda::GpuMat> subFrameGPU;
    if (bUsingGPUMem)
    {
        // Take the GPU channel handle.
        subFrameGPU = pExampleZEDCam1->GetFrameGPUReader();
    }
    else
    {
        // Take the CPU channel handle.
        subFrameCPU = pExampleZEDCam1->GetFrameCPUReader();
    }
    // Only the handle matching the camera's memory mode is taken; the other stays
    // default constructed and inactive.
    pubsub::Reader<cv::Mat> subDepthImageCPU;
    pubsub::Reader<cv::cuda::GpuMat> subDepthImageGPU;
    if (bUsingGPUMem)
    {
        // Take the GPU channel handle.
        subDepthImageGPU = pExampleZEDCam1->GetDepthImageGPUReader();
    }
    else
    {
        // Take the CPU channel handle.
        subDepthImageCPU = pExampleZEDCam1->GetDepthImageCPUReader();
    }
    // Only the handle matching the camera's memory mode is taken; the other stays
    // default constructed and inactive.
    pubsub::Reader<cv::Mat> subPointCloudCPU;
    pubsub::Reader<cv::cuda::GpuMat> subPointCloudGPU;
    if (bUsingGPUMem)
    {
        // Take the GPU channel handle.
        subPointCloudGPU = pExampleZEDCam1->GetPointCloudGPUReader();
    }
    else
    {
        // Take the CPU channel handle.
        subPointCloudCPU = pExampleZEDCam1->GetPointCloudCPUReader();
    }
    pubsub::Reader<ZEDCamera::Pose> subPose = pExampleZEDCam1->GetPoseReader();

    // Declare mats to store our own working copies in.
    cv::Mat cvNormalFrame1;
    cv::Mat cvDepthFrame1;
    cv::Mat cvPointCloud1;
    cv::Mat cvPointCloudColor1;

    // Declare FPS counter.
    IPS FPS = IPS();

    // ----- SLAM Setup -----
    // Global map for accumulated point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclGlobalMap(new pcl::PointCloud<pcl::PointXYZ>);
    // Create a PCL visualizer.
    pcl::visualization::PCLVisualizer::Ptr pclViewer(new pcl::visualization::PCLVisualizer("SLAM Viewer"));
    // pclViewer->setBackgroundColor(0, 0, 0);
    // // For timing the SLAM update (once per second)
    // auto last_slam_time = std::chrono::steady_clock::now();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Whether all three imagery channels produced something we can work with this iteration.
        bool bHaveNewImagery = false;

        // Check if the camera is setup to use CPU or GPU mats.
        if (bUsingGPUMem)
        {
            // Load the newest GPU snapshots ONCE into locals.
            pubsub::Reader<cv::cuda::GpuMat>::SharedSnapshot pFrame      = subFrameGPU.Get();
            pubsub::Reader<cv::cuda::GpuMat>::SharedSnapshot pDepth      = subDepthImageGPU.Get();
            pubsub::Reader<cv::cuda::GpuMat>::SharedSnapshot pPointCloud = subPointCloudGPU.Get();
            if (pFrame != nullptr && pDepth != nullptr && pPointCloud != nullptr)
            {
                // Download memory from GPU mats onto our own mats.
                pFrame->tData.download(cvNormalFrame1);
                pDepth->tData.download(cvDepthFrame1);
                pPointCloud->tData.download(cvPointCloud1);
                bHaveNewImagery = true;
            }
        }
        else
        {
            // Load the newest CPU snapshots ONCE into locals.
            pubsub::Reader<cv::Mat>::SharedSnapshot pFrame      = subFrameCPU.Get();
            pubsub::Reader<cv::Mat>::SharedSnapshot pDepth      = subDepthImageCPU.Get();
            pubsub::Reader<cv::Mat>::SharedSnapshot pPointCloud = subPointCloudCPU.Get();
            if (pFrame != nullptr && pDepth != nullptr && pPointCloud != nullptr)
            {
                // Snapshots are immutable and shared, and the code below writes into these mats,
                // so take our own deep copies rather than aliasing the published buffers.
                pFrame->tData.copyTo(cvNormalFrame1);
                pDepth->tData.copyTo(cvDepthFrame1);
                pPointCloud->tData.copyTo(cvPointCloud1);
                bHaveNewImagery = true;
            }
        }

        // Only do the work once every imagery channel has published.
        if (bHaveNewImagery)
        {

            // Put FPS on normal frame.
            cv::putText(cvNormalFrame1,
                        std::to_string(pExampleZEDCam1->GetIPS().GetExactIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Put FPS on depth frame.
            cv::putText(cvDepthFrame1,
                        std::to_string(pExampleZEDCam1->GetIPS().GetExactIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Split color from point cloud.
            imgops::SplitPointCloudColors(cvPointCloud1, cvPointCloudColor1);

            // Wait for the other info to be copied.
            // Load the newest pose snapshot ONCE into a local. Null until positional tracking has
            // published one.
            pubsub::Reader<ZEDCamera::Pose>::SharedSnapshot pPose = subPose.Get();
            if (pPose != nullptr)
            {
                // Work from the immutable snapshot's data.
                const ZEDCamera::Pose& stPose = pPose->tData;
                LOG_INFO(logging::g_qConsoleLogger,
                         "Positional Tracking: X: {} | Y: {} | Z: {}",
                         stPose.stTranslation.dX,
                         stPose.stTranslation.dY,
                         stPose.stTranslation.dZ);
                LOG_INFO(logging::g_qConsoleLogger,
                         "Positional Orientation: Roll: {} | Pitch: {} | Yaw: {}",
                         stPose.stEulerAngles.dXO,
                         stPose.stEulerAngles.dYO,
                         stPose.stEulerAngles.dZO);
            }

            // Print info.
            LOG_INFO(logging::g_qConsoleLogger, "ZED Getter FPS: {} | 1% Low: {}", pExampleZEDCam1->GetIPS().GetAverageIPS(), pExampleZEDCam1->GetIPS().Get1PercentLow());
            // Check if spatial mapping is enabled.
            if (ENABLE_SPATIAL_MAPPING)
            {
                LOG_INFO(logging::g_qConsoleLogger, "Spatial Mapping State: {}", sl::toString(pExampleZEDCam1->GetSpatialMappingState()).get());
            }

            // Display frames.
            cv::imshow("FRAME1", cvNormalFrame1);
            cv::imshow("DEPTH1", cvDepthFrame1);
            cv::imshow("POINT CLOUD COLOR 1", cvPointCloudColor1);

            // ----- SLAM Processing: once per second -----
            // auto now = std::chrono::steady_clock::now();
            // if (std::chrono::duration_cast<std::chrono::seconds>(now - last_slam_time).count() >= 1)
            // {
            //     last_slam_time = now;
            //     // Convert the acquired cv::Mat point cloud to a PCL point cloud.
            //     pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud = convertCVMatToPCL(cvPointCloud1);

            //     // Optional: Downsample the current scan with a VoxelGrid filter.
            //     pcl::VoxelGrid<pcl::PointXYZ> voxel;
            //     voxel.setInputCloud(current_cloud);
            //     voxel.setLeafSize(0.05f, 0.05f, 0.05f);    // Adjust leaf size as needed.
            //     pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            //     voxel.filter(*filtered_cloud);

            //     // If the global map is empty, initialize it with the current scan.
            //     if (pclGlobalMap->points.empty())
            //     {
            //         *pclGlobalMap = *filtered_cloud;
            //     }
            //     else
            //     {
            //         // Register the current scan to the global map using ICP.
            //         pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            //         icp.setInputSource(filtered_cloud);
            //         icp.setInputTarget(pclGlobalMap);
            //         pcl::PointCloud<pcl::PointXYZ> aligned;
            //         icp.align(aligned);

            //         if (icp.hasConverged())
            //         {
            //             // Get the transformation from ICP.
            //             Eigen::Matrix4f icp_transform = icp.getFinalTransformation();
            //             // Transform the current scan into the global coordinate frame.
            //             pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            //             pcl::transformPointCloud(*filtered_cloud, *transformed_cloud, icp_transform);
            //             // Fuse the transformed scan into the global map.
            //             *pclGlobalMap += *transformed_cloud;
            //         }
            //         else
            //         {
            //             LOG_WARNING(logging::g_qConsoleLogger, "ICP did not converge for the current scan.");
            //         }
            //     }

            //     // Update the PCL visualizer.
            //     pclViewer->removeAllPointClouds();
            //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(pclGlobalMap, 255, 255, 255);
            //     pclViewer->addPointCloud<pcl::PointXYZ>(pclGlobalMap, color_handler, "pclGlobalMap");
            //     pclViewer->spinOnce(10);
            // }
        }

        // Tick FPS counter.
        FPS.Tick();
        // Print FPS of main loop.
        LOG_INFO(logging::g_qConsoleLogger, "Main FPS: {}", FPS.GetAverageIPS());

        char chKey = cv::waitKey(1);
        if (chKey == 27)    // Press 'Esc' key to exit
            break;
    }

    // Close all OpenCV windows.
    cv::destroyAllWindows();

    // Check if spatial mapping is enabled.
    if (ENABLE_SPATIAL_MAPPING)
    {
        // Extract spatial map.
        std::future<sl::Mesh> fuSpatialMap;
        pExampleZEDCam1->ExtractSpatialMapAsync(fuSpatialMap);
        sl::Mesh slSpatialMap = fuSpatialMap.get();
        slSpatialMap.save("test.obj", sl::MESH_FILE_FORMAT::PLY);
    }

    /////////////////////////////////////////
    // Cleanup.
    /////////////////////////////////////////
    // Withdraw our demand so the camera stops retrieving data nobody is reading. This also happens
    // automatically when these handles go out of scope.
    subFrameCPU.Release();
    subFrameGPU.Release();
    subDepthImageCPU.Release();
    subDepthImageGPU.Release();
    subPointCloudCPU.Release();
    subPointCloudGPU.Release();
    subPose.Release();

    // Stop RoveComm quill logging or quill will segfault if trying to output logs to RoveComm.
    network::g_bRoveCommUDPStatus = false;
    network::g_bRoveCommTCPStatus = false;

    // Stop camera threads.
    globals::g_pCameraHandler->StopAllCameras();
}
