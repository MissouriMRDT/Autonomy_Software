/******************************************************************************
 * @brief Example file that demonstrates opening a ZED camera, starting a detector
 *      for that camera, and getting the detections.
 *
 * @file ArucoDetectionZED.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-20
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../src/AutonomyGlobals.h"
#include "../../../src/AutonomyLogging.h"
#include "../../../src/util/ExampleChecker.h"
#include "../../../src/vision/cameras/BasicCam.h"

/******************************************************************************
 * @brief This example demonstrates the proper way to interact with the TagDetectionHandler.
 *      A camera and detector are opened and started, then the detections are requested and logged.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-08
 ******************************************************************************/
void RunExample()
{
    // Initialize and start handlers.
    globals::g_pNavigationBoard     = new NavigationBoard();
    globals::g_pCameraHandler       = new CameraHandler();
    globals::g_pTagDetectionHandler = new TagDetectionHandler();

    // Get pointer to camera.
    ZEDCam* ExampleZEDCam1 = globals::g_pCameraHandler->GetZED(CameraHandler::eHeadMainCam);
    // Start basic cam.
    ExampleZEDCam1->Start();

    // Get pointer to the tag detector for the basic cam.
    TagDetector* ExampleTagDetector1 = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::eHeadMainCam);
    // Start the basic cam detector.
    ExampleTagDetector1->Start();

    // Declare mats to store images in.
    cv::Mat cvNormalFrame1;
    cv::Mat cvDetectionsFrame1;
    cv::cuda::GpuMat cvGPUNormalFrame1;
    // Declare vector to store tag detections in.
    std::vector<arucotag::ArucoTag> vTagDetections1;

    // Declare FPS counter.
    IPS FPS = IPS();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Create instance variables.
        std::future<bool> fuCopyStatus1;
        std::future<bool> fuDetectionCopyStatus1;
        std::future<bool> fuDetectionFrameCopyStatus1;

        // Check if the camera is setup to use CPU or GPU mats.
        if (constants::ZED_MAINCAM_USE_GPU_MAT)
        {
            // Grab normal frame from camera.
            fuCopyStatus1 = ExampleZEDCam1->RequestFrameCopy(cvGPUNormalFrame1);
        }
        else
        {
            // Grab normal frame from camera.
            fuCopyStatus1 = ExampleZEDCam1->RequestFrameCopy(cvNormalFrame1);
        }
        // Get detections overlay frame from detector.
        std::future<bool> fuDetectionFrameCopyStatus1 = ExampleTagDetector1->RequestDetectionOverlayFrame(cvDetectionsFrame1);
        // Grab other info from detector.
        fuDetectionCopyStatus1 = ExampleTagDetector1->RequestDetectedArucoTags(vTagDetections1);

        // Show first frame copy.
        if (fuCopyStatus1.get())
        {
            // Check if the camera is setup to use CPU or GPU mats.
            if (constants::ZED_MAINCAM_USE_GPU_MAT)
            {
                // Download memory from gpu mats if necessary.
                cvGPUNormalFrame1.download(cvNormalFrame1);
            }

            // Put FPS on normal frame.
            cv::putText(cvNormalFrame1,
                        std::to_string(ExampleZEDCam1->GetIPS().GetExactIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Display frame.
            cv::imshow("BasicCamExample Frame1", cvNormalFrame1);
        }

        // Show detections overlay frame.
        if (fuDetectionFrameCopyStatus1.get() && !cvDetectionsFrame1.empty())
        {
            // Put detector FPS on frame.
            cv::putText(cvDetectionsFrame1,
                        std::to_string(ExampleTagDetector1->GetIPS().GetExactIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Display frame.
            cv::imshow("Detections Overlay Frame1", cvDetectionsFrame1);
        }

        // Wait for detections to be copied.
        if (fuDetectionCopyStatus1.get())
        {
            // Print length of detections vector.
            LOG_INFO(logging::g_qConsoleLogger, "Detections1 vector length: {}", vTagDetections1.size());
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
    // Stop camera threads.
    globals::g_pTagDetectionHandler->StopAllDetectors();
    globals::g_pCameraHandler->StopAllCameras();

    // Delete dynamically allocated objects.
    delete globals::g_pCameraHandler;
    delete globals::g_pTagDetectionHandler;
    delete globals::g_pNavigationBoard;

    // Set dangling pointers to null.
    globals::g_pCameraHandler       = nullptr;
    globals::g_pTagDetectionHandler = nullptr;
    glabals::g_pNavigationBoard     = nullptr;
}
