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
#include <libavutil/log.h>    // For av_log_set_level
#include <opencv2/core/cuda.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

/// \endcond

// Global constants
const float VOXEL_GRID_LEAF_SIZE = 0.05f;    // Increased from 0.01f to prevent overflow

/******************************************************************************
 * @brief Mouse callback function for depth image clicks.
 *
 * @param nEvent - The type of mouse event (e.g., left button click).
 * @param nX - The x-coordinate of the mouse event.
 * @param nY - The y-coordinate of the mouse event.
 * @param nFlags - The flags associated with the mouse event.
 * @param pUserData - Pointer to user data (in this case, the depth image).
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-01
 ******************************************************************************/
void DepthMouseCallback(int nEvent, int nX, int nY, int nFlags, void* pUserData)
{
    (void) nFlags;    // Unused parameter
    if (nEvent != cv::EVENT_LBUTTONDOWN)
        return;

    cv::Mat* cvDepthImage = static_cast<cv::Mat*>(pUserData);
    if (nX >= 0 && nY >= 0 && nX < cvDepthImage->cols && nY < cvDepthImage->rows)
    {
        // Handle different depth image types
        float fDepthValue = 0.0f;

        // Check the depth image type and retrieve the depth value accordingly
        fDepthValue = cvDepthImage->at<float>(nY, nX);
        std::cout << "Depth at (" << nX << ", " << nY << "): " << fDepthValue << " m" << std::endl;
    }
}

/******************************************************************************
 * @brief Mouse callback function for point cloud clicks.
 *
 * @param nEvent - The type of mouse event (e.g., left button click).
 * @param nX - The x-coordinate of the mouse event.
 * @param nY - The y-coordinate of the mouse event.
 * @param nFlags - The flags associated with the mouse event.
 * @param pUserData - Pointer to user data (in this case, the point cloud).
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-03
 ******************************************************************************/
void PointCloudMouseCallback(int nEvent, int nX, int nY, int nFlags, void* pUserData)
{
    (void) nFlags;    // Unused parameter
    if (nEvent != cv::EVENT_LBUTTONDOWN)
        return;

    cv::Mat* cvPointCloud = static_cast<cv::Mat*>(pUserData);
    if (nX >= 0 && nY >= 0 && nX < cvPointCloud->cols && nY < cvPointCloud->rows)
    {
        // Handle different point cloud types
        cv::Vec4f cvPoint = cvPointCloud->at<cv::Vec4f>(nY, nX);
        std::cout << "Point at (" << nX << ", " << nY << "): "
                  << "X: " << cvPoint[0] << ", "
                  << "Y: " << cvPoint[1] << ", "
                  << "Z: " << cvPoint[2] << std::endl;
    }
}

/******************************************************************************
 * @brief Suppresses PCL logging messages.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-01
 ******************************************************************************/
void SuppressPCLLogging()
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
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
    // Suppress logging messages
    SuppressPCLLogging();

    // Initialize and start handlers.
    globals::g_pCameraHandler = new CameraHandler();

    // Get pointer to camera.
    std::shared_ptr<ZEDCamera> pExampleZEDCam1 = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam);
    // Start ZED cam.
    pExampleZEDCam1->Start();

    // Whether this camera hands out GPU or CPU mats. Decides which channels we subscribe to.
    const bool bUsingGPUMem = pExampleZEDCam1->GetUsingGPUMem();

    // Register demand for the depth measure and point cloud. The camera calls into the ZED SDK for
    // these ONLY while something is subscribed, so these handles are what turn retrieval on.
    pubsub::Subscription subDepthMeasure =
        bUsingGPUMem ? pExampleZEDCam1->GetDepthMeasureGPUPublisher().Subscribe() : pExampleZEDCam1->GetDepthMeasureCPUPublisher().Subscribe();
    pubsub::Subscription subPointCloud =
        bUsingGPUMem ? pExampleZEDCam1->GetPointCloudGPUPublisher().Subscribe() : pExampleZEDCam1->GetPointCloudCPUPublisher().Subscribe();

    // Declare mats to store our own working copies in.
    cv::Mat cvDepthFrame1;
    cv::Mat cvPointCloud1;

    // Declare FPS counter.
    IPS FPS = IPS();

    // Create a depth display window and set up mouse callback
    cv::namedWindow("Depth Frame", cv::WINDOW_AUTOSIZE);

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Whether both channels produced something we can work with this iteration.
        bool bHaveNewData = false;

        // Check if the camera is setup to use CPU or GPU mats.
        if (bUsingGPUMem)
        {
            // Load the newest GPU snapshots ONCE into locals.
            pubsub::Publisher<cv::cuda::GpuMat>::SharedSnapshot pDepth      = pExampleZEDCam1->GetDepthMeasureGPUPublisher().Get();
            pubsub::Publisher<cv::cuda::GpuMat>::SharedSnapshot pPointCloud = pExampleZEDCam1->GetPointCloudGPUPublisher().Get();
            if (pDepth != nullptr && pPointCloud != nullptr)
            {
                // Download data from GPU matrices onto our own mats.
                pDepth->tData.download(cvDepthFrame1);
                pPointCloud->tData.download(cvPointCloud1);
                bHaveNewData = true;
            }
        }
        else
        {
            // Load the newest CPU snapshots ONCE into locals.
            pubsub::Publisher<cv::Mat>::SharedSnapshot pDepth      = pExampleZEDCam1->GetDepthMeasureCPUPublisher().Get();
            pubsub::Publisher<cv::Mat>::SharedSnapshot pPointCloud = pExampleZEDCam1->GetPointCloudCPUPublisher().Get();
            if (pDepth != nullptr && pPointCloud != nullptr)
            {
                // Snapshots are immutable and shared, and the code below writes into these mats,
                // so take our own deep copies rather than aliasing the published buffers.
                pDepth->tData.copyTo(cvDepthFrame1);
                pPointCloud->tData.copyTo(cvPointCloud1);
                bHaveNewData = true;
            }
        }

        // Only do the display work once both channels have published.
        if (bHaveNewData)
        {

            // Display the depth frame and set up mouse callback
            if (!cvDepthFrame1.empty())
            {
                // Normalize the depth frame for display (depth is stored as 16-bit unsigned int)
                cv::Mat cvDepthDisplay;
                cv::normalize(cvDepthFrame1, cvDepthDisplay, 0, 255, cv::NORM_MINMAX, CV_8U);
                cv::applyColorMap(cvDepthDisplay, cvDepthDisplay, cv::COLORMAP_JET);

                // Show the depth frame
                cv::imshow("Depth Frame", cvDepthDisplay);

                // Set the mouse callback for the depth window
                cv::setMouseCallback("Depth Frame", DepthMouseCallback, &cvDepthFrame1);
                cv::setMouseCallback("Depth Frame", PointCloudMouseCallback, &cvPointCloud1);
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
                pclViewer->addCoordinateSystem(20.0);
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
            // Downsample the point cloud for better visualization performance
            pcl::VoxelGrid<pcl::PointXYZ> pclVoxelGrid;
            pclVoxelGrid.setInputCloud(pclFilteredPointCloud);
            pclVoxelGrid.setLeafSize(VOXEL_GRID_LEAF_SIZE, VOXEL_GRID_LEAF_SIZE, VOXEL_GRID_LEAF_SIZE);
            pclVoxelGrid.filter(*pclDownsampledPointCloud);
            // Update the point cloud in the viewer
            pclViewer->updatePointCloud(pclDownsampledPointCloud, "depth_cloud");
            pclViewer->spinOnce(10);
        }

        // Tick FPS counter.
        FPS.Tick();

        char chKey = cv::waitKey(1);
        if (chKey == 27)    // Press 'Esc' key to exit
            break;
    }

    // Close all OpenCV windows.
    cv::destroyAllWindows();

    /////////////////////////////////////////
    // Cleanup.
    /////////////////////////////////////////
    // Withdraw our demand so the camera stops retrieving depth and point cloud data nobody is
    // reading. This also happens automatically when these handles go out of scope.
    subDepthMeasure.Release();
    subPointCloud.Release();

    // Stop RoveComm quill logging or quill will segfault if trying to output logs to RoveComm.
    network::g_bRoveCommUDPStatus = false;
    network::g_bRoveCommTCPStatus = false;

    // Stop camera threads.
    globals::g_pCameraHandler->StopAllCameras();
}
