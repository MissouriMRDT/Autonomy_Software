/******************************************************************************
 * @brief Example file that demonstrates opening and using multiple different
 *      features of the ZED camera.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-16
 ******************************************************************************/

#include "../../src/AutonomyGlobals.h"
#include "../../src/AutonomyLogging.h"
#include "../../src/util/ExampleChecker.h"
#include "../../src/vision/cameras/ZEDCam.h"

// Declare file constants.
const bool ENABLE_SPATIAL_MAPPING = false;

/******************************************************************************
 * @brief Main example method.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-22
 ******************************************************************************/
void RunExample()
{
    // Initialize and start Threads
    g_pCameraHandler = new CameraHandlerThread();
    g_pCameraHandler->StartAllCameras();

    // Get pointer to camera.
    ZEDCam* TestCamera1 = g_pCameraHandler->GetZED(CameraHandlerThread::eHeadMainCam);
    ZEDCam* TestCamera2 = g_pCameraHandler->GetZED(CameraHandlerThread::eHeadMainCam2);

    // Turn on ZED features.
    TestCamera1->EnablePositionalTracking();
    TestCamera2->EnablePositionalTracking();
    // Check if we should turn on spatial mapping.
    if (ENABLE_SPATIAL_MAPPING)
    {
        // Enable spatial mapping.
        TestCamera1->EnableSpatialMapping();
    }

    // Declare mats to store images in.
    cv::Mat cvNormalFrame1;
    cv::Mat cvDepthFrame1;
    cv::Mat cvNormalFrame2;
    cv::Mat cvDepthFrame2;
    cv::cuda::GpuMat cvGPUNormalFrame1;
    cv::cuda::GpuMat cvGPUDepthFrame1;
    cv::cuda::GpuMat cvGPUNormalFrame2;
    cv::cuda::GpuMat cvGPUDepthFrame2;

    // Declare FPS counter.
    IPS FPS = IPS();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Grab normal frame from camera.
        if (TestCamera1->GrabFrame(cvGPUNormalFrame1) && TestCamera1->GrabDepth(cvGPUDepthFrame1, false))
        {
            // Download memory from gpu mats if necessary.
            cvGPUNormalFrame1.download(cvNormalFrame1);
            cvGPUDepthFrame1.download(cvDepthFrame1);

            // Put FPS on normal frame.
            cv::putText(cvNormalFrame1, std::to_string(TestCamera1->GetIPS().GetExactIPS()), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));

            // Put FPS on depth frame.
            cv::putText(cvDepthFrame1, std::to_string(TestCamera1->GetIPS().GetExactIPS()), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));

            // Display frame.
            cv::imshow("TEST1", cvNormalFrame1);
            cv::imshow("DEPTH1", cvDepthFrame1);

            // Get info about position.
            sl::Pose slPose;
            TestCamera1->GetPositionalPose(slPose);
            sl::Translation slTranslation = slPose.getTranslation();
            sl::float3 slEulerAngles      = slPose.getEulerAngles(false);

            // Print info.
            LOG_INFO(g_qConsoleLogger, "ZED Getter FPS: {} | 1% Low: {}", TestCamera1->GetIPS().GetAverageIPS(), TestCamera1->GetIPS().Get1PercentLow());
            LOG_INFO(g_qConsoleLogger, "Main FPS: {}", FPS.GetExactIPS());
            LOG_INFO(g_qConsoleLogger, "Positional Tracking: X: {} | Y: {} | Z: {}", slTranslation.x, slTranslation.y, slTranslation.z);
            LOG_INFO(g_qConsoleLogger, "Positional Orientation: Roll: {} | Pitch: {} | Yaw:{}", slEulerAngles[0], slEulerAngles[1], slEulerAngles[2]);
            // Check if spatial mapping is enabled.
            if (ENABLE_SPATIAL_MAPPING)
            {
                LOG_INFO(g_qConsoleLogger, "Spatial Mapping State: {}", sl::toString(TestCamera1->GetSpatialMappingState()).get());
            }
        }

        // Grab normal frame from camera.
        if (TestCamera2->GrabFrame(cvGPUNormalFrame2) && TestCamera2->GrabDepth(cvGPUDepthFrame2, false))
        {
            // Download memory from gpu mats if necessary.
            cvGPUNormalFrame2.download(cvNormalFrame2);
            cvGPUDepthFrame2.download(cvDepthFrame2);

            // Put FPS on normal frame.
            cv::putText(cvNormalFrame2, std::to_string(TestCamera2->GetIPS().GetExactIPS()), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));

            // Put FPS on depth frame.
            cv::putText(cvDepthFrame2, std::to_string(TestCamera2->GetIPS().GetExactIPS()), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));

            // Display frame.
            cv::imshow("TEST2", cvNormalFrame2);
            cv::imshow("DEPTH2", cvDepthFrame2);

            // Get info about position.
            sl::Pose slPose;
            TestCamera2->GetPositionalPose(slPose);
            sl::Translation slTranslation = slPose.getTranslation();
            sl::float3 slEulerAngles      = slPose.getEulerAngles(false);

            // Print info.
            LOG_INFO(g_qConsoleLogger, "ZED Getter FPS: {} | 1% Low: {}", TestCamera2->GetIPS().GetAverageIPS(), TestCamera2->GetIPS().Get1PercentLow());
            LOG_INFO(g_qConsoleLogger, "Main FPS: {}", FPS.GetExactIPS());
            LOG_INFO(g_qConsoleLogger, "Positional Tracking: X: {} | Y: {} | Z: {}", slTranslation.x, slTranslation.y, slTranslation.z);
            LOG_INFO(g_qConsoleLogger, "Positional Orientation: Roll: {} | Pitch: {} | Yaw:{}", slEulerAngles[0], slEulerAngles[1], slEulerAngles[2]);
            // Check if spatial mapping is enabled.
            if (ENABLE_SPATIAL_MAPPING)
            {
                LOG_INFO(g_qConsoleLogger, "Spatial Mapping State: {}", sl::toString(TestCamera2->GetSpatialMappingState()).get());
            }
        }

        // Tick FPS counter.
        FPS.Tick();

        char chKey = cv::waitKey(1);
        if (chKey == 27)    // Press 'Esc' key to exit
            break;
    }

    cv::destroyAllWindows();

    // Check if spatial mapping is enabled.
    if (ENABLE_SPATIAL_MAPPING)
    {
        // Extract spatial map.
        std::future<sl::Mesh> fuSpatialMap;
        TestCamera1->ExtractSpatialMapAsync(fuSpatialMap);
        sl::Mesh slSpatialMap = fuSpatialMap.get();
        slSpatialMap.save("test.obj", sl::MESH_FILE_FORMAT::PLY);
    }

    // Delete dynamically allocated memory.
    delete g_pCameraHandler;

    // Set dangling pointers to null.
    g_pCameraHandler = nullptr;
}
