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

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Grab normal frame from camera.
        std::future<cv::Mat&> fuTest = TestCamera1->GrabFrame(cvNormalFrame1);
        cvNormalFrame1               = fuTest.get();
        if (!cvNormalFrame1.empty())
        {
            // Print info.
            LOG_INFO(g_qConsoleLogger, "BasicCam Getter FPS: {} | 1% Low: {}", TestCamera1->GetIPS().GetAverageIPS(), TestCamera1->GetIPS().Get1PercentLow());

            // Put FPS on normal frame.
            cv::putText(cvNormalFrame1, std::to_string(TestCamera1->GetIPS().GetExactIPS()), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));

            // Display frame.
            cv::imshow("BasicCamExample", cvNormalFrame1);
        }

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
