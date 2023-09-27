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

    // Get reference to camera.
    BasicCam* TestCamera1 = g_pCameraHandler->GetBasicCam(CameraHandlerThread::eHeadLeftArucoEye);
    // Declare mats to store images in.
    cv::Mat cvNormalFrame1;
    cv::Mat cvNormalFrame2;

    // Declare FPS counter.
    IPS FPS = IPS();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Grab normal frame from camera.
        std::future<bool> fuCopyStatus1 = TestCamera1->RequestFrameCopy(cvNormalFrame1);
        std::future<bool> fuCopyStatus2 = TestCamera1->RequestFrameCopy(cvNormalFrame2);

        // Show first frame copy.
        if (fuCopyStatus1.get() && !cvNormalFrame1.empty())
        {
            // Print info.
            LOG_INFO(g_qConsoleLogger, "BasicCam Getter FPS: {} | 1% Low: {}", TestCamera1->GetIPS().GetAverageIPS(), TestCamera1->GetIPS().Get1PercentLow());

            // Put FPS on normal frame.
            cv::putText(cvNormalFrame1, std::to_string(TestCamera1->GetIPS().GetExactIPS()), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));

            // Display frame.
            cv::imshow("BasicCamExample Frame1", cvNormalFrame1);
        }

        // Show second frame copy.
        if (fuCopyStatus2.get() && !cvNormalFrame2.empty())
        {
            // Print info.
            LOG_INFO(g_qConsoleLogger, "BasicCam Getter FPS: {} | 1% Low: {}", TestCamera1->GetIPS().GetAverageIPS(), TestCamera1->GetIPS().Get1PercentLow());

            // Put FPS on normal frame.
            cv::putText(cvNormalFrame2, std::to_string(TestCamera1->GetIPS().GetExactIPS()), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));

            // Display frame.
            cv::imshow("BasicCamExample Frame2", cvNormalFrame2);
        }

        // Tick FPS counter.
        FPS.Tick();
        // Print FPS of main loop.
        LOG_INFO(g_qConsoleLogger, "Main FPS: {}", FPS.GetAverageIPS());

        char chKey = cv::waitKey(1);
        if (chKey == 27)    // Press 'Esc' key to exit
            break;
    }

    cv::destroyAllWindows();

    // Delete dynamically allocated memory.
    delete g_pCameraHandler;

    // Set dangling pointers to null.
    g_pCameraHandler = nullptr;
}
