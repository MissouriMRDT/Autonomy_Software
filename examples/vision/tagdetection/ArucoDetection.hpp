/******************************************************************************
 * @brief Example file that demonstrates opening a camera, starting a detector
 *      for that camera, and getting the detections.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-08
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
    globals::g_pCameraHandler       = new CameraHandler();
    globals::g_pTagDetectionHandler = new TagDetectionHandler();

    // Get pointer to camera.
    BasicCam* ExampleBasicCam1 = globals::g_pCameraHandler->GetBasicCam(CameraHandler::eHeadLeftArucoEye);
    // Start basic cam.
    ExampleBasicCam1->Start();

    // Get pointer to the tag detector for the basic cam.
    TagDetector* ExampleTagDetector1 = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::eHeadLeftArucoEye);
    // Start the basic cam detector.
    ExampleTagDetector1->Start();

    // Declare mats to store images in.
    cv::Mat cvNormalFrame1;
    // Declare vector to store tag detections in.
    std::vector<arucotag::ArucoTag> vTagDetections;

    // Declare FPS counter.
    IPS FPS = IPS();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Grab normal frame from camera.
        std::future<bool> fuCopyStatus1 = ExampleBasicCam1->RequestFrameCopy(cvNormalFrame1);
        // Get detections from tag detector for BasicCam.
        std::future<bool> fuDetectionCopyStatus1 = ExampleTagDetector1->RequestDetectedArucoTags(vTagDetections);

        // Show first frame copy.
        if (fuCopyStatus1.get() && !cvNormalFrame1.empty())
        {
            // Put FPS on normal frame.
            cv::putText(cvNormalFrame1,
                        std::to_string(ExampleBasicCam1->GetIPS().GetExactIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Display frame.
            cv::imshow("BasicCamExample Frame1", cvNormalFrame1);
        }

        // Wait for detections to be copied.
        if (fuDetectionCopyStatus1.get())
        {
            // Print length of detections vector.
            LOG_INFO(logging::g_qConsoleLogger, "Detections vector length: {}", vTagDetections.size());
        }

        // Tick FPS counter.
        FPS.Tick();
        // Print FPS of main loop.
        LOG_INFO(logging::g_qConsoleLogger, "Main FPS: {}", FPS.GetAverageIPS());
        // Print camera FPS info.
        LOG_INFO(logging::g_qConsoleLogger,
                 "BasicCam Getter FPS: {} | 1% Low: {}",
                 ExampleBasicCam1->GetIPS().GetAverageIPS(),
                 ExampleBasicCam1->GetIPS().Get1PercentLow());
        // Print detector FPS info.
        LOG_INFO(logging::g_qConsoleLogger, "Detector FPS: {}", ExampleTagDetector1->GetIPS().GetExactIPS());

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
    globals::g_pCameraHandler->StopAllCameras();
    globals::g_pTagDetectionHandler->StopAllDetectors();

    // Delete dynamically allocated objects.
    delete globals::g_pCameraHandler;
    delete globals::g_pTagDetectionHandler;

    // Set dangling pointers to null.
    globals::g_pCameraHandler       = nullptr;
    globals::g_pTagDetectionHandler = nullptr;
}
