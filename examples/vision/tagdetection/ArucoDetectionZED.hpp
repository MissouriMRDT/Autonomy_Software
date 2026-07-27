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
#include "../../../src/AutonomyNetworking.h"
#include "../../../src/util/ExampleChecker.h"
#include "../../../src/vision/cameras/BasicCam.h"

/******************************************************************************
 * @brief This example demonstrates the proper way to interact with a TagDetector that is
 *      running on a ZED camera. A camera and detector are opened and started, then their
 *      published output is read and logged.
 *
 *      The detector already subscribes to its camera internally, so this example only
 *      subscribes to what IT wants to display. Note that we pick the camera's frame
 *      channel to match its configured memory mode (GetUsingGPUMem()); the detector's
 *      overlay channels are always cv::Mat regardless.
 *
 *      All reads are non-blocking Get() calls, so this loop, the detector's loop, and the
 *      camera's loop all run at completely independent rates.
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
    std::shared_ptr<ZEDCamera> ExampleZEDCam1 = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam);
    // Start basic cam.
    ExampleZEDCam1->Start();

    // Get pointer to the tag detector for the basic cam.
    std::shared_ptr<TagDetector> ExampleTagDetector1 = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam);
    // Start the basic cam detector.
    ExampleTagDetector1->Start();

    // Whether this camera hands out GPU or CPU mats. Decides which frame channel we subscribe to.
    const bool bUsingGPUMem = ExampleZEDCam1->GetUsingGPUMem();

    // Register demand for the camera frame we display and the detector's overlay frame. Both are
    // produced only while something is subscribed. The detected-tags channel needs no subscription.
    pubsub::Subscription subCameraFrame = bUsingGPUMem ? ExampleZEDCam1->GetFrameGPUPublisher().Subscribe() : ExampleZEDCam1->GetFrameCPUPublisher().Subscribe();
    pubsub::Subscription subDetectionOverlay = ExampleTagDetector1->GetDetectionOverlayPublisher().Subscribe();

    // Declare mats to draw our annotated copies into.
    cv::Mat cvNormalFrame1;
    cv::Mat cvDetectionsFrame1;

    // Declare FPS counter.
    IPS FPS = IPS();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Whether we managed to load a camera frame this iteration.
        bool bHaveCameraFrame = false;

        // Check if the camera is setup to use CPU or GPU mats.
        if (bUsingGPUMem)
        {
            // Load the newest GPU frame snapshot ONCE into a local.
            pubsub::Publisher<cv::cuda::GpuMat>::SharedSnapshot pFrame = ExampleZEDCam1->GetFrameGPUPublisher().Get();
            if (pFrame != nullptr && !pFrame->tData.empty())
            {
                // Download from GPU memory onto our own mat. Done here, off the camera's critical path.
                pFrame->tData.download(cvNormalFrame1);
                bHaveCameraFrame = true;
            }
        }
        else
        {
            // Load the newest CPU frame snapshot ONCE into a local.
            pubsub::Publisher<cv::Mat>::SharedSnapshot pFrame = ExampleZEDCam1->GetFrameCPUPublisher().Get();
            if (pFrame != nullptr && !pFrame->tData.empty())
            {
                // Snapshots are immutable and shared, so clone before drawing on it.
                cvNormalFrame1   = pFrame->tData.clone();
                bHaveCameraFrame = true;
            }
        }

        // Show the camera frame.
        if (bHaveCameraFrame)
        {
            // Put FPS on normal frame.
            cv::putText(cvNormalFrame1,
                        std::to_string(ExampleZEDCam1->GetIPS().GetExactIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Display frame.
            cv::imshow("ZEDCamExample Frame1", cvNormalFrame1);
        }

        // Load the newest detection overlay snapshot ONCE into a local.
        pubsub::Publisher<cv::Mat>::SharedSnapshot pOverlay = ExampleTagDetector1->GetDetectionOverlayPublisher().Get();
        if (pOverlay != nullptr && !pOverlay->tData.empty())
        {
            // Snapshots are immutable and shared, so clone before drawing on it.
            cvDetectionsFrame1 = pOverlay->tData.clone();

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

        // Load the newest detected tags snapshot and report it.
        pubsub::Publisher<std::vector<tagdetectutils::ArucoTag>>::SharedSnapshot pTags = ExampleTagDetector1->GetDetectedTagsPublisher().Get();
        if (pTags != nullptr)
        {
            // Print length of detections vector. Read straight from the snapshot; no copy needed
            // unless we intended to keep or modify the tags.
            LOG_INFO(logging::g_qConsoleLogger, "Detections1 vector length: {}", pTags->tData.size());
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
    // Withdraw our demand so the camera and detector stop producing data nobody is reading. This
    // also happens automatically when these handles go out of scope.
    subCameraFrame.Release();
    subDetectionOverlay.Release();

    // Stop RoveComm quill logging or quill will segfault if trying to output logs to RoveComm.
    network::g_bRoveCommUDPStatus = false;
    network::g_bRoveCommTCPStatus = false;

    // Stop camera threads.
    globals::g_pTagDetectionHandler->StopAllDetectors();
    globals::g_pCameraHandler->StopAllCameras();
}
