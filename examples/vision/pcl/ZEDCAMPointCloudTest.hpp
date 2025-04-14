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

    // Create a PCL visualizer.
    pcl::visualization::PCLVisualizer::Ptr pclViewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    // Set background color and initialize camera position
    pclViewer->setBackgroundColor(0, 0, 0);
    pclViewer->initCameraParameters();

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
            pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());
            for (int i = 0; i < cvPointCloud1.rows; ++i)
            {
                for (int j = 0; j < cvPointCloud1.cols; ++j)
                {
                    // Access the 3D point stored as a cv::Vec3f.
                    cv::Vec3f cvPoint = cvPointCloud1.at<cv::Vec3f>(i, j);

                    // Stricter filtering of invalid or distant points
                    if (std::isfinite(cvPoint[0]) && std::isfinite(cvPoint[1]) && std::isfinite(cvPoint[2]) && std::abs(cvPoint[0]) < 10.0 &&
                        std::abs(cvPoint[1]) < 10.0 && std::abs(cvPoint[2]) < 10.0)
                    {
                        pclCloud->points.push_back(pcl::PointXYZ(cvPoint[0], cvPoint[1], cvPoint[2]));
                    }
                }
            }

            // Set the PCL point pclCloud dimensions.
            pclCloud->width    = static_cast<uint32_t>(pclCloud->points.size());
            pclCloud->height   = 1;
            pclCloud->is_dense = false;

            // Clear visualizer completely
            pclViewer->removeAllPointClouds();
            pclViewer->removeAllShapes();

            // Add the filtered point cloud
            pclViewer->addPointCloud<pcl::PointXYZ>(pclCloud, "cloud");
            pclViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

            // Use a longer spin time to ensure proper rendering
            pclViewer->spinOnce(10);
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
