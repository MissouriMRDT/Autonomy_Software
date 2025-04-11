/******************************************************************************
 * @brief Example file that demonstrates getting and viewing the point cloud
 *      of the ZEDCam.
 *
 * @file ZEDCAMPointCloudTest.hpp
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclGlobalMap(new pcl::PointCloud<pcl::PointXYZ>);
    // Create a PCL visualizer.
    pcl::visualization::PCLVisualizer::Ptr pclViewer(new pcl::visualization::PCLVisualizer("SLAM Viewer"));
    // pclViewer->setBackgroundColor(0, 0, 0);
    // // For timing the SLAM update (once per second)
    // auto last_slam_time = std::chrono::steady_clock::now();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Create instance variables.
        std::future<bool> fuDepthCopyStatus;
        std::future<bool> fuPointCloudCopyStatus;

        // Check if the camera is setup to use CPU or GPU mats.
        if (pExampleZEDCam1->GetUsingGPUMem())
        {
            // Grab frames from camera.
            fuDepthCopyStatus      = pExampleZEDCam1->RequestDepthCopy(cvGPUDepthFrame1, false);
            fuPointCloudCopyStatus = pExampleZEDCam1->RequestPointCloudCopy(cvGPUPointCloud1);
        }
        else
        {
            // Grab frames from camera.
            fuDepthCopyStatus      = pExampleZEDCam1->RequestDepthCopy(cvDepthFrame1, false);
            fuPointCloudCopyStatus = pExampleZEDCam1->RequestPointCloudCopy(cvPointCloud1);
        }

        // Wait for the frames to be copied.
        if (fuDepthCopyStatus.get() && fuPointCloudCopyStatus.get())
        {
            if (pExampleZEDCam1->GetUsingGPUMem())
            {
                // Download data from GPU matrices.
                cvGPUDepthFrame1.download(cvDepthFrame1);
                cvGPUPointCloud1.download(cvPointCloud1);
            }

            // Convert cv::Mat point cloud (which has 3 channels: X, Y, Z) into a PCL point cloud.
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            for (int i = 0; i < cvPointCloud1.rows; ++i)
            {
                for (int j = 0; j < cvPointCloud1.cols; ++j)
                {
                    // Access the 3D point stored as a cv::Vec3f.
                    cv::Vec3f point = cvPointCloud1.at<cv::Vec3f>(i, j);

                    // Check that the point is valid: some sensors encode invalid points as NaN.
                    if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]))
                    {
                        cloud->points.push_back(pcl::PointXYZ(point[0], point[1], point[2]));
                    }
                }
            }

            // Set the PCL point cloud dimensions.
            cloud->width    = static_cast<uint32_t>(cloud->points.size());
            cloud->height   = 1;
            cloud->is_dense = false;

            // Update the PCL visualizer.
            // If the point cloud already exists in the viewer, update it; otherwise, add it.
            if (!pclViewer->updatePointCloud<pcl::PointXYZ>(cloud, "cloud"))
            {
                pclViewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
                // Optionally, set rendering properties (e.g., point size).
                pclViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
            }

            // Refresh the visualizer to display the new frame.
            pclViewer->spinOnce(1);
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

    /////////////////////////////////////////
    // Cleanup.
    /////////////////////////////////////////
    // Stop RoveComm quill logging or quill will segfault if trying to output logs to RoveComm.
    network::g_bRoveCommUDPStatus = false;
    network::g_bRoveCommTCPStatus = false;

    // Stop camera threads.
    globals::g_pCameraHandler->StopAllCameras();
}
