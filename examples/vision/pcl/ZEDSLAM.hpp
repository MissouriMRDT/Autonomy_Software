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
#include <pcl-1.15/pcl/common/transforms.h>
#include <pcl-1.15/pcl/filters/voxel_grid.h>
#include <pcl-1.15/pcl/impl/point_types.hpp>
#include <pcl-1.15/pcl/pcl_macros.h>
#include <pcl-1.15/pcl/point_cloud.h>
#include <pcl-1.15/pcl/point_types.h>
#include <pcl-1.15/pcl/registration/icp.h>
#include <pcl-1.15/pcl/visualization/pcl_visualizer.h>

// Declare file constants.
const bool ENABLE_SPATIAL_MAPPING = false;

//------------------------------------------------------------------------------
// Helper function to convert cv::Mat point cloud to a PCL point cloud.
// Assumes cvPointCloud is CV_32FC4 (X, Y, Z, and an extra channel)
// from the ZED SDK.
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

//------------------------------------------------------------------------------
// Main example routine.
void RunExample()
{
    // Initialize and start handlers.
    globals::g_pCameraHandler = new CameraHandler();

    // Get pointer to camera.
    ZEDCamera* ExampleZEDCam1 = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam);
    // Start ZED cam.
    ExampleZEDCam1->Start();

    // Turn on ZED features.
    ExampleZEDCam1->EnablePositionalTracking();
    // Check if we should turn on spatial mapping.
    if (ENABLE_SPATIAL_MAPPING)
    {
        // Enable spatial mapping.
        ExampleZEDCam1->EnableSpatialMapping();
    }

    // Declare mats to store images in.
    cv::Mat cvNormalFrame1;
    cv::Mat cvDepthFrame1;
    cv::Mat cvPointCloud1;
    cv::Mat cvPointCloudColor1;
    cv::cuda::GpuMat cvGPUNormalFrame1;
    cv::cuda::GpuMat cvGPUDepthFrame1;
    cv::cuda::GpuMat cvGPUPointCloud1;
    // Declare other data types to store data in.
    ZEDCam::Pose stPose;

    // Declare FPS counter.
    IPS FPS = IPS();

    // ----- SLAM Setup -----
    // Global map for accumulated point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZ>);
    // Create a PCL visualizer.
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("SLAM Viewer"));
    // viewer->setBackgroundColor(0, 0, 0);
    // // For timing the SLAM update (once per second)
    // auto last_slam_time = std::chrono::steady_clock::now();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Create instance variables.
        std::future<bool> fuFrameCopyStatus;
        std::future<bool> fuDepthCopyStatus;
        std::future<bool> fuPointCloudCopyStatus;

        // Check if the camera is setup to use CPU or GPU mats.
        if (ExampleZEDCam1->GetUsingGPUMem())
        {
            // Grab frames from camera.
            fuFrameCopyStatus      = ExampleZEDCam1->RequestFrameCopy(cvGPUNormalFrame1);
            fuDepthCopyStatus      = ExampleZEDCam1->RequestDepthCopy(cvGPUDepthFrame1, false);
            fuPointCloudCopyStatus = ExampleZEDCam1->RequestPointCloudCopy(cvGPUPointCloud1);
        }
        else
        {
            // Grab frames from camera.
            fuFrameCopyStatus      = ExampleZEDCam1->RequestFrameCopy(cvNormalFrame1);
            fuDepthCopyStatus      = ExampleZEDCam1->RequestDepthCopy(cvDepthFrame1, false);
            fuPointCloudCopyStatus = ExampleZEDCam1->RequestPointCloudCopy(cvPointCloud1);
        }
        // Grab other info from camera.
        std::future<bool> fuPoseCopyStatus = ExampleZEDCam1->RequestPositionalPoseCopy(stPose);

        // Wait for the frames to be copied.
        if (fuFrameCopyStatus.get() && fuDepthCopyStatus.get() && fuPointCloudCopyStatus.get())
        {
            // Check if the camera is setup to use CPU or GPU mats.
            if (ExampleZEDCam1->GetUsingGPUMem())
            {
                // Download memory from GPU mats if necessary.
                cvGPUNormalFrame1.download(cvNormalFrame1);
                cvGPUDepthFrame1.download(cvDepthFrame1);
                cvGPUPointCloud1.download(cvPointCloud1);
            }

            // Put FPS on normal frame.
            cv::putText(cvNormalFrame1,
                        std::to_string(ExampleZEDCam1->GetIPS().GetExactIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Put FPS on depth frame.
            cv::putText(cvDepthFrame1, std::to_string(ExampleZEDCam1->GetIPS().GetExactIPS()), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));

            // Split color from point cloud.
            imgops::SplitPointCloudColors(cvPointCloud1, cvPointCloudColor1);

            // Wait for the other info to be copied.
            if (fuPoseCopyStatus.get())
            {
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
            LOG_INFO(logging::g_qConsoleLogger, "ZED Getter FPS: {} | 1% Low: {}", ExampleZEDCam1->GetIPS().GetAverageIPS(), ExampleZEDCam1->GetIPS().Get1PercentLow());
            // Check if spatial mapping is enabled.
            if (ENABLE_SPATIAL_MAPPING)
            {
                LOG_INFO(logging::g_qConsoleLogger, "Spatial Mapping State: {}", sl::toString(ExampleZEDCam1->GetSpatialMappingState()).get());
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
            //     if (global_map->points.empty())
            //     {
            //         *global_map = *filtered_cloud;
            //     }
            //     else
            //     {
            //         // Register the current scan to the global map using ICP.
            //         pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            //         icp.setInputSource(filtered_cloud);
            //         icp.setInputTarget(global_map);
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
            //             *global_map += *transformed_cloud;
            //         }
            //         else
            //         {
            //             LOG_WARNING(logging::g_qConsoleLogger, "ICP did not converge for the current scan.");
            //         }
            //     }

            //     // Update the PCL visualizer.
            //     viewer->removeAllPointClouds();
            //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(global_map, 255, 255, 255);
            //     viewer->addPointCloud<pcl::PointXYZ>(global_map, color_handler, "global_map");
            //     viewer->spinOnce(10);
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
        ExampleZEDCam1->ExtractSpatialMapAsync(fuSpatialMap);
        sl::Mesh slSpatialMap = fuSpatialMap.get();
        slSpatialMap.save("test.obj", sl::MESH_FILE_FORMAT::PLY);
    }

    /////////////////////////////////////////
    // Cleanup.
    /////////////////////////////////////////
    // Stop RoveComm quill logging or quill will segfault if trying to output logs to RoveComm.
    network::g_bRoveCommUDPStatus = false;
    network::g_bRoveCommTCPStatus = false;

    // Stop camera threads.
    globals::g_pCameraHandler->StopAllCameras();

    // Delete dynamically allocated objects.
    delete globals::g_pCameraHandler;
    delete globals::g_pNavigationBoard;
    delete network::g_pRoveCommUDPNode;
    delete network::g_pRoveCommTCPNode;
    // Set dangling pointers to null.
    globals::g_pCameraHandler   = nullptr;
    globals::g_pNavigationBoard = nullptr;
    network::g_pRoveCommUDPNode = nullptr;
    network::g_pRoveCommTCPNode = nullptr;
}
