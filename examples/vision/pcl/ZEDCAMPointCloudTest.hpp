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

/// \cond
#include <chrono>
#include <future>
#include <opencv2/core/cuda.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

/// \endcond

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

            // Use PCL to visualize the point cloud.
            static pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());
            static pcl::PointCloud<pcl::PointXYZ>::Ptr pclFilteredPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
            static pcl::PointCloud<pcl::PointXYZ>::Ptr pclDownsampledPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
            static pcl::visualization::PCLVisualizer::Ptr pclViewer;

            // Initialize the PCL viewer on the first run
            if (!pclViewer)
            {
                pclViewer.reset(new pcl::visualization::PCLVisualizer("Depth Point Cloud"));
                pclViewer->setBackgroundColor(0, 0, 0);
                pclViewer->addPointCloud<pcl::PointXYZ>(pclDownsampledPointCloud, "depth_cloud");
                pclViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "depth_cloud");
                pclViewer->addCoordinateSystem(1000.0);
                pclViewer->initCameraParameters();
            }

            // Convert the OpenCV point cloud to PCL format.
            pclCloud->width    = cvPointCloud1.cols;
            pclCloud->height   = cvPointCloud1.rows;
            pclCloud->is_dense = false;
            pclCloud->points.resize(pclCloud->width * pclCloud->height);

            // Copy points from OpenCV format to PCL format.
            for (int nY = 0; nY < cvPointCloud1.rows; ++nY)
            {
                for (int nX = 0; nX < cvPointCloud1.cols; ++nX)
                {
                    cv::Vec4f cvPoint = cvPointCloud1.at<cv::Vec4f>(nY, nX);
                    size_t siIDx      = nY * cvPointCloud1.cols + nX;

                    // Only add points with valid depth.
                    if (cvPoint[2] > 0)
                    {
                        pclCloud->points[siIDx].x = cvPoint[0];
                        pclCloud->points[siIDx].y = cvPoint[1];
                        pclCloud->points[siIDx].z = cvPoint[2];
                    }
                    else
                    {
                        // For invalid points, set to NaN.
                        pclCloud->points[siIDx].x = std::numeric_limits<float>::quiet_NaN();
                        pclCloud->points[siIDx].y = std::numeric_limits<float>::quiet_NaN();
                        pclCloud->points[siIDx].z = std::numeric_limits<float>::quiet_NaN();
                    }
                }
            }

            // Remove NaN points to clean up the cloud
            std::vector<int> vIndices;
            pcl::removeNaNFromPointCloud(*pclCloud, *pclFilteredPointCloud, vIndices);
            std::cout << "Point cloud filtered: " << pclFilteredPointCloud->points.size() << " valid points." << std::endl;

            // Downsample the point cloud for better visualization performance
            pcl::VoxelGrid<pcl::PointXYZ> pclVoxelGrid;
            pclVoxelGrid.setInputCloud(pclFilteredPointCloud);
            pclVoxelGrid.setLeafSize(5.0f, 5.0f, 5.0f);
            pclVoxelGrid.filter(*pclDownsampledPointCloud);
            std::cout << "Point cloud downsampled to " << pclDownsampledPointCloud->points.size() << " points." << std::endl;

            // Update the point cloud in the viewer
            pclViewer->updatePointCloud(pclDownsampledPointCloud, "depth_cloud");
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
