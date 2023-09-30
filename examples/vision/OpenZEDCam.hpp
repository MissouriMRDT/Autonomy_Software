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
    globals::g_pCameraHandler = new CameraHandlerThread();
    globals::g_pCameraHandler->StartAllCameras();

    // Get pointer to camera.
    ZEDCam* TestCamera1 = globals::g_pCameraHandler->GetZED(CameraHandlerThread::eHeadMainCam);

    // Turn on ZED features.
    TestCamera1->EnablePositionalTracking();
    // Check if we should turn on spatial mapping.
    if (ENABLE_SPATIAL_MAPPING)
    {
        // Enable spatial mapping.
        TestCamera1->EnableSpatialMapping();
    }

    // Declare mats to store images in.
    cv::Mat cvNormalFrame1;
    cv::Mat cvDepthFrame1;
    cv::cuda::GpuMat cvGPUNormalFrame1;
    cv::cuda::GpuMat cvGPUDepthFrame1;

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
            LOG_INFO(logging::g_qConsoleLogger, "ZED Getter FPS: {} | 1% Low: {}", TestCamera1->GetIPS().GetAverageIPS(), TestCamera1->GetIPS().Get1PercentLow());
            LOG_INFO(logging::g_qConsoleLogger, "Main FPS: {}", FPS.GetExactIPS());
            LOG_INFO(logging::g_qConsoleLogger, "Positional Tracking: X: {} | Y: {} | Z: {}", slTranslation.x, slTranslation.y, slTranslation.z);
            LOG_INFO(logging::g_qConsoleLogger, "Positional Orientation: Roll: {} | Pitch: {} | Yaw:{}", slEulerAngles[0], slEulerAngles[1], slEulerAngles[2]);
            // Check if spatial mapping is enabled.
            if (ENABLE_SPATIAL_MAPPING)
            {
                LOG_INFO(logging::g_qConsoleLogger, "Spatial Mapping State: {}", sl::toString(TestCamera1->GetSpatialMappingState()).get());
            }
        }

        // Tick FPS counter.
        FPS.Tick();

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
        TestCamera1->ExtractSpatialMapAsync(fuSpatialMap);
        sl::Mesh slSpatialMap = fuSpatialMap.get();
        slSpatialMap.save("test.obj", sl::MESH_FILE_FORMAT::PLY);
    }
}
