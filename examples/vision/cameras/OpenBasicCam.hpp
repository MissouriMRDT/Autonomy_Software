/******************************************************************************
 * @brief Example file that demonstrates opening and using multiple different
 *      features of the basic camera.
 *
 * @file OpenBasicCam.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-16
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../src/AutonomyGlobals.h"
#include "../../../src/AutonomyLogging.h"
#include "../../../src/util/ExampleChecker.h"
#include "../../../src/vision/cameras/BasicCam.h"

/******************************************************************************
 * @brief This example demonstrates the proper way to consume frames from a camera.
 *
 *      Cameras hand data out through a publish-latest channel instead of per-consumer
 *      requests. There are only two things a consumer does:
 *
 *      1. Subscribe() once, and hold the returned pubsub::Subscription for as long as
 *         you want the data produced. The camera only reads and publishes a data type
 *         while at least one subscriber is alive, so a channel nobody wants costs
 *         nothing. Letting the Subscription go out of scope withdraws that demand.
 *
 *      2. Get() the newest snapshot whenever you want it. This is a non-blocking read
 *         that never waits on the camera's loop, so your loop rate and the camera's are
 *         completely independent. It returns nullptr until the camera has published its
 *         first frame.
 *
 *      Two rules worth internalizing:
 *
 *      - Load the snapshot ONCE into a local and work from that local. Calling Get()
 *        repeatedly returns whatever is newest each time, which is not a stable frame.
 *      - A published snapshot is immutable and shared with every other consumer. To
 *        modify it, clone it onto your own frame first, as this example does before
 *        drawing text on it.
 *
 *      The snapshot's ullSequence lets you tell whether the camera has actually produced
 *      something new since last time, so you can skip redundant work entirely.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-22
 ******************************************************************************/
void RunExample()
{
    // Initialize basic cam.
    std::shared_ptr<BasicCamera> ExampleBasicCam1 = std::make_unique<BasicCam>(0, 1280, 720, 60, PIXEL_FORMATS::eBGR, 0, 0, false);
    // Start basic cam.
    ExampleBasicCam1->Start();

    // Register demand for this camera's frames. Hold this handle for as long as we want the
    // camera to keep producing; the camera reads nothing while it has no subscribers.
    pubsub::Subscription subFrames = ExampleBasicCam1->GetFramePublisher().Subscribe();

    // Declare a mat to draw our annotated copy into.
    cv::Mat cvDisplayFrame;

    // Track which frame we last processed so we can skip iterations with nothing new.
    unsigned long long ullLastProcessedSequence = 0;

    // Declare FPS counter.
    IPS FPS = IPS();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Load the newest published frame ONCE into a local. Everything below works from this
        // local, so the frame cannot change underneath us mid-iteration.
        pubsub::Publisher<cv::Mat>::SharedSnapshot pFrameSnapshot = ExampleBasicCam1->GetFramePublisher().Get();

        // A null snapshot just means the camera has not published a frame yet (it may still be
        // opening). This is not an error and never blocks; simply try again next iteration.
        if (pFrameSnapshot != nullptr && pFrameSnapshot->ullSequence != ullLastProcessedSequence)
        {
            // Remember which frame we processed so a repeat of it is skipped next time.
            ullLastProcessedSequence = pFrameSnapshot->ullSequence;

            // The snapshot is immutable and shared with every other consumer, so clone it before
            // drawing on it. Doing this on our own thread keeps it off the camera's critical path.
            cvDisplayFrame = pFrameSnapshot->tData.clone();

            // Print info.
            LOG_INFO(logging::g_qConsoleLogger,
                     "BasicCam Getter FPS: {} | 1% Low: {}",
                     ExampleBasicCam1->GetIPS().GetAverageIPS(),
                     ExampleBasicCam1->GetIPS().Get1PercentLow());

            // Put FPS on our own copy of the frame.
            cv::putText(cvDisplayFrame,
                        std::to_string(ExampleBasicCam1->GetIPS().GetExactIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Display frame.
            cv::imshow("BasicCamExample Frame", cvDisplayFrame);
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
    // Withdraw our demand so the camera stops producing frames nobody is reading. This also
    // happens automatically when subFrames goes out of scope.
    subFrames.Release();
    // Stop camera threads.
    globals::g_pCameraHandler->StopAllCameras();
}
