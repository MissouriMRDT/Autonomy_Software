/******************************************************************************
 * @brief Example file that demonstrates opening and using a camera and YOLO model.
 *
 * @file InferenceYOLOModel.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-05-03
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../src/AutonomyGlobals.h"
#include "../../../src/AutonomyLogging.h"
#include "../../../src/util/ExampleChecker.h"
#include "../../../src/util/vision/YOLOModel.hpp"
#include "../../../src/vision/cameras/BasicCam.h"

#include <chrono>
#include <iostream>
#include <memory>

std::vector<std::string> LoadNames(const std::string& path)
{
    // load class names
    std::vector<std::string> class_names;
    std::ifstream infile(path);
    if (infile.is_open())
    {
        std::string line;
        while (getline(infile, line))
        {
            class_names.emplace_back(line);
        }
        infile.close();
    }
    else
    {
        std::cerr << "Error loading the class names!\n";
    }

    return class_names;
}

void Demo(cv::Mat& img, const std::vector<std::vector<constants::Detection>>& detections, const std::vector<std::string>& class_names, bool label = true)
{
    if (!detections.empty())
    {
        for (const auto& detection : detections[0])
        {
            const auto& box = detection.cvBoundingBox;
            float score     = detection.fConfidence;
            int class_idx   = detection.nClassID;

            cv::rectangle(img, box, cv::Scalar(0, 0, 255), 2);

            if (label)
            {
                std::stringstream ss;
                ss << std::fixed << std::setprecision(2) << score;
                std::string s   = class_names[class_idx] + " " + ss.str();

                auto font_face  = cv::FONT_HERSHEY_DUPLEX;
                auto font_scale = 1.0;
                int thickness   = 1;
                int baseline    = 0;
                auto s_size     = cv::getTextSize(s, font_face, font_scale, thickness, &baseline);
                cv::rectangle(img, cv::Point(box.tl().x, box.tl().y - s_size.height - 5), cv::Point(box.tl().x + s_size.width, box.tl().y), cv::Scalar(0, 0, 255), -1);
                cv::putText(img, s, cv::Point(box.tl().x, box.tl().y - 5), font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
            }
        }
    }

    cv::namedWindow("Result", cv::WINDOW_AUTOSIZE);
    cv::imshow("Result", img);
    cv::waitKey(1);    // Changed waitKey to 1 for video playback
}

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
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-05-03
 ******************************************************************************/
void RunExample()
{
    // Initialize and start handlers.
    globals::g_pCameraHandler = new CameraHandler();

    // Get reference to camera.
    BasicCam* ExampleBasicCam1 = globals::g_pCameraHandler->GetBasicCam(CameraHandler::eHeadGroundCam);
    // Start basic cam.
    ExampleBasicCam1->Start();

    std::vector<std::string> class_names = LoadNames("/workspaces/Autonomy_Software/data/models/yolo_models/old/coco.names");
    if (class_names.empty())
    {
        LOG_CRITICAL(logging::g_qSharedLogger, "No Model Coco Names");
        exit(-1);
    }

    // // Initialize a new YOLOModel object.
    yolomodel::pytorch::TorchInterpreter ExampleTorchModel =
        yolomodel::pytorch::TorchInterpreter("/workspaces/Autonomy_Software/data/models/yolo_models/old/best.torchscript", torch::kCPU);

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
            std::vector<std::vector<constants::Detection>> vOutputTensorObjects = ExampleTorchModel.Inference(cvNormalFrame1, 0.40f, 0.50f);

            if (vOutputTensorObjects.size() == 0)
            {
                LOG_WARNING(logging::g_qSharedLogger, "No Detections");
            }
            else
            {
                LOG_WARNING(logging::g_qSharedLogger, "{} Detections", vOutputTensorObjects.size());
            }

            // Loop through all output detection vectors for each tensor output.
            for (std::vector<constants::Detection> vObjects : vOutputTensorObjects)
            {
                if (vObjects.size() == 0)
                {
                    LOG_ERROR(logging::g_qSharedLogger, "No Detections");
                }
                else
                {
                    LOG_ERROR(logging::g_qSharedLogger, "{} Detections", vObjects.size());
                }

                for (constants::Detection vObject : vObjects)
                {
                    LOG_CRITICAL(logging::g_qSharedLogger, "nClassID: {}, fConfidence: {}", vObject.nClassID, vObject.fConfidence);
                }

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
