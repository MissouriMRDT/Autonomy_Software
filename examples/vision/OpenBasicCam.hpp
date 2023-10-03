/******************************************************************************
 * @brief Example file that demonstrates opening and using multiple different
 *      features of the basic camera.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-16
 ******************************************************************************/

#include "../../src/AutonomyGlobals.h"
#include "../../src/AutonomyLogging.h"
#include "../../src/util/ExampleChecker.h"
#include "../../src/vision/cameras/BasicCam.h"

/******************************************************************************
 * @brief This example demonstrates the proper way to interact with the CameraHandler.
 *      A pointer to a BasicCam is retrieved and then a couple of local cv::Mat are created
 *      for storing frames. Then, the frames are passed to the RequestFrameCopy function of the
 *      camera and a future is IMMEDIATELY returned. The method call doesn't wait for frame to be
 *      retrieved/copied before returning. This allows you to request multiple frames/data from the
 *      camera non-sequentially.
 *
 *      Inside the camera thread, the cv::Mat pointer that points to the cv::Mat within THIS class
 *      is written to and an std::promise is set to TRUE. The future that was return now contains this
 *      TRUE value. When the get() method is called on the returned future, the code will block until
 *      the promise is fullfilled (set to TRUE). Once the get() method returns, the cv::Mat within
 *      this class now contains a complete frame and can be display or used in other computer vision
 *      things.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-22
 ******************************************************************************/
void RunExample()
{
    // Initialize and start Threads
    globals::g_pCameraHandler = new CameraHandler();

    // Get reference to camera.
    BasicCam* ExampleBasicCam1 = globals::g_pCameraHandler->GetBasicCam(CameraHandler::eHeadLeftArucoEye);
    // Start basic cam.
    ExampleBasicCam1->Start();

    // Declare mats to store images in.
    cv::Mat cvNormalFrame1;
    cv::Mat cvNormalFrame2;

    // Declare FPS counter.
    IPS FPS = IPS();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Grab normal frame from camera.
        std::future<bool> fuCopyStatus1 = ExampleBasicCam1->RequestFrameCopy(cvNormalFrame1);
        std::future<bool> fuCopyStatus2 = ExampleBasicCam1->RequestFrameCopy(cvNormalFrame2);

        // Show first frame copy.
        if (fuCopyStatus1.get() && !cvNormalFrame1.empty())
        {
            // Print info.
            LOG_INFO(logging::g_qConsoleLogger,
                     "BasicCam Getter FPS: {} | 1% Low: {}",
                     ExampleBasicCam1->GetIPS().GetAverageIPS(),
                     ExampleBasicCam1->GetIPS().Get1PercentLow());

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

        // Show second frame copy.
        if (fuCopyStatus2.get() && !cvNormalFrame2.empty())
        {
            // Print info.
            LOG_INFO(logging::g_qConsoleLogger,
                     "BasicCam Getter FPS: {} | 1% Low: {}",
                     ExampleBasicCam1->GetIPS().GetAverageIPS(),
                     ExampleBasicCam1->GetIPS().Get1PercentLow());

            // Put FPS on normal frame.
            cv::putText(cvNormalFrame2,
                        std::to_string(ExampleBasicCam1->GetIPS().GetExactIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Display frame.
            cv::imshow("BasicCamExample Frame2", cvNormalFrame2);
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
}
