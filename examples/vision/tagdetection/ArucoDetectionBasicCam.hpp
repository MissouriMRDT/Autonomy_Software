/******************************************************************************
 * @brief Example file that demonstrates opening a Basic camera, starting a detector
 *      for that camera, and getting the detections.
 *
 * @file ArucoDetectionBasicCam.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-19
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../src/AutonomyGlobals.h"
#include "../../../src/AutonomyLogging.h"
#include "../../../src/util/ExampleChecker.h"
#include "../../../src/vision/cameras/BasicCam.h"

/******************************************************************************
 * @brief This example demonstrates the proper way to interact with a TagDetector.
 *      A camera and detector are opened and started, then their published output is read
 *      and logged.
 *
 *      A TagDetector is both a consumer and a producer. It subscribes to its camera's
 *      frame channel internally, and publishes its own results on three channels:
 *
 *      - GetDetectedTagsReader()         - the detected tags. Published unconditionally,
 *                                          because the detection pass already computed
 *                                          them, so no Reader is needed to keep it published.
 *      - GetDetectionOverlayReader()     - the annotated frame. Demand gated, because it
 *                                          costs a full-frame clone. Subscribe to enable.
 *      - GetLastGoodOverlayReader()      - the last annotated frame that had detections.
 *                                          Also demand gated.
 *
 *      Reading any of them is a non-blocking Get(), so this loop never waits on the
 *      detector's loop or the camera's.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-08
 ******************************************************************************/
void RunExample()
{
    // Construct a BasicCam directly rather than going through the CameraHandler. The handler's
    // BasicCamName enum currently declares no basic cameras (they are commented out in
    // CameraHandler.h), so there is nothing to fetch from it. Constructing one here keeps this
    // example self contained and working regardless of that configuration.
    std::shared_ptr<BasicCamera> ExampleBasicCam1 = std::make_shared<BasicCam>(0, 1280, 720, 30, PIXEL_FORMATS::eBGR, 90.0, 60.0, false);
    // Start basic cam.
    ExampleBasicCam1->Start();

    // Get pointer to the tag detector for the basic cam.
    std::unique_ptr<TagDetector> ExampleTagDetector1 = std::make_unique<TagDetector>(ExampleBasicCam1);
    // Start the basic cam detector.
    ExampleTagDetector1->Start();

    // Register demand for the camera's frames and the detector's overlay frame. The camera and
    // the detector each produce these only while something is subscribed, so these handles are
    // what turn that work on. The detected-tags channel needs no subscription.
    pubsub::Reader<cv::Mat> rdCameraFrame     = ExampleBasicCam1->GetFrameReader();
    pubsub::Reader<cv::Mat> rdDetectionOverlay = ExampleTagDetector1->GetDetectionOverlayReader();

    // Declare mats to draw our annotated copies into.
    cv::Mat cvNormalFrame1;
    cv::Mat cvDetectionsFrame1;

    // Declare FPS counter.
    IPS FPS = IPS();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Load the newest snapshot of each channel ONCE into a local. All three reads are
        // non-blocking and return null until that producer has published something.
        pubsub::SharedSnapshot<cv::Mat> pCameraFrame = rdCameraFrame.Get();
        pubsub::SharedSnapshot<cv::Mat> pOverlay     = rdDetectionOverlay.Get();
        pubsub::SharedSnapshot<std::vector<tagdetectutils::ArucoTag>> pTags = ExampleTagDetector1->GetDetectedTagsReader().Get();

        // Show the camera frame.
        if (pCameraFrame != nullptr && !pCameraFrame->tData.empty())
        {
            // Snapshots are immutable and shared, so clone before drawing on it.
            cvNormalFrame1 = pCameraFrame->tData.clone();

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

        // Show detections overlay frame.
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

        // Report the detected tags.
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
    rdCameraFrame.Release();
    rdDetectionOverlay.Release();

    // Stop the detector and camera we created, in that order: consumers before producers.
    ExampleTagDetector1->RequestStop();
    ExampleTagDetector1->Join();
    ExampleBasicCam1->RequestStop();
    ExampleBasicCam1->Join();
}
