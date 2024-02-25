/******************************************************************************
 * @brief Example file that demonstrates opening and using a camera and YOLO model.
 *
 * @file InferenceYOLOModel.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-11-13
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../src/AutonomyGlobals.h"
#include "../../../src/AutonomyLogging.h"
#include "../../../src/util/ExampleChecker.h"
#include "../../../src/util/vision/YOLOModel.hpp"
#include "../../../src/vision/cameras/BasicCam.h"

/******************************************************************************
 * @brief This example demonstrates the proper way to interact with the CameraHandler.
 *      A pointer to a BasicCam is retrieved and then a couple of local cv::Mat are created
 *      for storing frames. Then, the frames are passed to the RequestFrameCopy function of the
 *      camera and a future is IMMEDIATELY returned. The method call doesn't wait for frame to be
 *      retrieved/copied before returning. This allows you to request multiple frames/data from the
 *      camera non-sequentially.
 *
 *      The example also demonstrates how to create a new yolo model using the YOLOModel utility class and
 *      pass image to it for inferencing. Inferencing is the process of forwarding a tensor input through
 *      a deep neural network, getting the output tensor, and interpreting the results. The hardware used in
 *      this example is an EdgeTPU, which is a small external tensor processing unit that can be plugged into
 *      the computer via USB or PCIe.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-11-13
 ******************************************************************************/
void RunExample()
{
    // Initialize and start handlers
    globals::g_pCameraHandler = new CameraHandler();

    // Get reference to camera.
    BasicCam* ExampleBasicCam1 = globals::g_pCameraHandler->GetBasicCam(CameraHandler::eHeadLeftArucoEye);
    // Start basic cam.
    ExampleBasicCam1->Start();

    // Get list of tpu devices.
    std::vector<edgetpu::EdgeTpuManager::DeviceEnumerationRecord> vEdgeTPUDevices = yolomodel::tensorflow::TPUInterpreter::GetHardwareDevices();
    // Loop through each device.
    for (long unsigned int nIter = 0; nIter < vEdgeTPUDevices.size(); ++nIter)
    {
        // Submit logger info.
        LOG_INFO(logging::g_qConsoleLogger, "Device {}: Type={}, Path={}", nIter, int(vEdgeTPUDevices[nIter].type), vEdgeTPUDevices[nIter].path);
    }

    // Initialize a new YOLOModel object.
    yolomodel::tensorflow::TPUInterpreter ExampleEdgeTPUModel =
        yolomodel::tensorflow::TPUInterpreter("../data/models/yolo_models/coco_v5n_x240/best.tflite", yolomodel::tensorflow::TPUInterpreter::eMax);
    // Open and load a new YOLOModel from the given path into an EdgeTPU device.
    ExampleEdgeTPUModel.OpenAndLoad();

    // Declare mats to store images in.
    cv::Mat cvNormalFrame1;
    cv::Mat cvInferenceFrame1;

    // Declare FPS counter.
    IPS FPS = IPS();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Grab normal frame from camera.
        std::future<bool> fuCopyStatus1 = ExampleBasicCam1->RequestFrameCopy(cvNormalFrame1);

        // Show first frame copy.
        if (fuCopyStatus1.get() && !cvNormalFrame1.empty())
        {
            // Convert camera frame from BGR to RGB format.
            cv::cvtColor(cvNormalFrame1, cvInferenceFrame1, cv::COLOR_BGR2RGB);
            // Run inference on YOLO model with current image.
            std::vector<std::vector<yolomodel::Detection>> vOutputTensorObjects = ExampleEdgeTPUModel.Inference(cvInferenceFrame1, 0.40f, 0.60f);
            // Loop through all output detection vectors for each tensor output.
            for (std::vector<yolomodel::Detection> vObjects : vOutputTensorObjects)
            {
                // Draw detected objects on frame.
                yolomodel::DrawDetections(cvNormalFrame1, vObjects);
            }

            // Put FPS on normal frame.
            cv::putText(cvNormalFrame1,
                        std::to_string(ExampleBasicCam1->GetIPS().GetExactIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Display frame.
            cv::imshow("BasicCamExample Frame1", cvNormalFrame1);

            // Print info.
            LOG_INFO(logging::g_qConsoleLogger,
                     "BasicCam Getter FPS: {} | 1% Low: {}",
                     ExampleBasicCam1->GetIPS().GetAverageIPS(),
                     ExampleBasicCam1->GetIPS().Get1PercentLow());
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
    globals::g_pCameraHandler->StopAllCameras();

    // Delete dynamically allocated objects.
    delete globals::g_pCameraHandler;
    // Set dangling pointers to null.
    globals::g_pCameraHandler = nullptr;
}
