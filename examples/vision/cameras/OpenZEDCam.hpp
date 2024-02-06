/******************************************************************************
 * @brief Example file that demonstrates opening and using multiple different
 *      features of the ZED camera.
 *
 * @file OpenZEDCam.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-16
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../src/AutonomyConstants.h"
#include "../../../src/AutonomyGlobals.h"
#include "../../../src/AutonomyLogging.h"
#include "../../../src/util/ExampleChecker.h"
#include "../../../src/util/vision/ImageOperations.hpp"

/// \cond
#include <chrono>
/// \endcond

// Declare file constants.
const bool ENABLE_SPATIAL_MAPPING = false;

/******************************************************************************
 * @brief This example demonstrates the proper way to interact with the CameraHandler.
 *      A pointer to a ZEDCam is retrieved and then a couple of local cv::Mat are created
 *      for storing frames. Then, the frames are passed to the RequestFrameCopy function of the
 *      camera and a future is IMMEDIATELY returned. The method call doesn't wait for frame to be
 *      retrieved/copied before returning. This allows you to request multiple frames/data from the
 *      camera non-sequentially.
 *
 *      Inside the camera thread, the cv::Mat pointer that points to the cv::Mat within THIS class
 *      is written to and an std::promise is set to TRUE. The future that was return now contains this
 *      TRUE value. When the get() method is called on the returned future, the code will block until
 *      the promise is fulfilled (set to TRUE). Once the get() method returns, the cv::Mat within
 *      this class now contains a complete frame and can be display or used in other computer vision
 *      things.
 *
 *      The same exact process happens for the positional tracking pose that is retrieved from the camera.
 *      Multiple other methods of the ZEDCam class work this way as it allows this thread and other threads
 *      to get multiple pieces of from the camera without slowing it down to an unusable speed.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-22
 ******************************************************************************/
void RunExample()
{
    // Initialize and start handlers.
    globals::g_pNavigationBoard = new NavigationBoard();
    globals::g_pCameraHandler   = new CameraHandler();

    // Get pointer to camera.
    ZEDCam* ExampleZEDCam1 = globals::g_pCameraHandler->GetZED(CameraHandler::eHeadMainCam);
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
    sl::Pose slPose;

    // Declare FPS counter.
    IPS FPS = IPS();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Create instance variables.
        std::future<bool> fuFrameCopyStatus;
        std::future<bool> fuDepthCopyStatus;
        std::future<bool> fuPointCloudCopyStatus;

        // Check if the camera is setup to use CPU or GPU mats.
        if (constants::ZED_MAINCAM_USE_GPU_MAT)
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
        std::future<bool> fuPoseCopyStatus = ExampleZEDCam1->RequestPositionalPoseCopy(slPose);

        // Wait for the frames to be copied.
        // std::this_thread::sleep_for(std::chrono::seconds(1));
        if (fuFrameCopyStatus.get() && fuDepthCopyStatus.get() && fuPointCloudCopyStatus.get())
        {
            // Check if the camera is setup to use CPU or GPU mats.
            if (constants::ZED_MAINCAM_USE_GPU_MAT)
            {
                // Download memory from gpu mats if necessary.
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
                // Wait for the
                sl::Translation slTranslation = slPose.getTranslation();
                sl::float3 slEulerAngles      = slPose.getEulerAngles(false);

                LOG_INFO(logging::g_qConsoleLogger, "Positional Tracking: X: {} | Y: {} | Z: {}", slTranslation.x, slTranslation.y, slTranslation.z);
                LOG_INFO(logging::g_qConsoleLogger, "Positional Orientation: Roll: {} | Pitch: {} | Yaw:{}", slEulerAngles[0], slEulerAngles[1], slEulerAngles[2]);
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
    // Stop camera threads.
    globals::g_pCameraHandler->StopAllCameras();

    // Delete dynamically allocated objects.
    delete globals::g_pCameraHandler;
    delete globals::g_pNavigationBoard;
    // Set dangling pointers to null.
    globals::g_pCameraHandler   = nullptr;
    globals::g_pNavigationBoard = nullptr;
}
