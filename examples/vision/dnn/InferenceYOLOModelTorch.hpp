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
#include "../../../temp/cxxopts.hpp"
#include "../../../temp/detector.h"

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

void Demo(cv::Mat& img, const std::vector<std::vector<Detection>>& detections, const std::vector<std::string>& class_names, bool label = true)
{
    if (!detections.empty())
    {
        for (const auto& detection : detections[0])
        {
            const auto& box = detection.bbox;
            float score     = detection.score;
            int class_idx   = detection.class_idx;

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
    // Get reference to camera.
    BasicCam* ExampleBasicCam1 = new BasicCam("/workspaces/Autonomy_Software/temp/temp.mkv",
                                              constants::BASICCAM_GROUNDCAM_RESOLUTIONX,
                                              constants::BASICCAM_GROUNDCAM_RESOLUTIONY,
                                              constants::BASICCAM_GROUNDCAM_FPS,
                                              constants::BASICCAM_GROUNDCAM_PIXELTYPE,
                                              constants::BASICCAM_GROUNDCAM_HORIZONTAL_FOV,
                                              constants::BASICCAM_GROUNDCAM_VERTICAL_FOV,
                                              constants::BASICCAM_GROUNDCAM_ENABLE_RECORDING,
                                              constants::BASICCAM_GROUNDCAM_FRAME_RETRIEVAL_THREADS);
    // Start basic cam.
    ExampleBasicCam1->Start();

    bool is_gpu = false;
    // set device type - CPU/GPU
    torch::DeviceType device_type;
    if (torch::cuda::is_available() && is_gpu)
    {
        device_type = torch::kCUDA;
    }
    else
    {
        device_type = torch::kCPU;
    }

    std::vector<std::string> class_names = LoadNames("/workspaces/Autonomy_Software/data/models/yolo_models/old/coco.names");
    if (class_names.empty())
    {
        LOG_CRITICAL(logging::g_qSharedLogger, "No Model Coco Names");
        exit(-1);
    }

    // load network
    std::string weights = "/workspaces/Autonomy_Software/data/models/yolo_models/old/best.torchscript";
    auto detector       = Detector(weights, device_type);

    auto temp_img       = cv::Mat::zeros(1920.0, 1080.0, CV_32FC3);
    detector.Run(temp_img, 1.0f, 1.0f);

    float conf_thres = 0.4;
    float iou_thres  = 0.5;

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

            // Put FPS on normal frame.
            cv::putText(cvNormalFrame1,
                        std::to_string(ExampleBasicCam1->GetIPS().GetExactIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Run inference on YOLO model with current image.
            auto result = detector.Run(cvNormalFrame1, conf_thres, iou_thres);
            Demo(cvNormalFrame1, result, class_names, true);

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
    ExampleBasicCam1->RequestStop();
    ExampleBasicCam1->Join();

    // Delete dynamically allocated objects.
    delete ExampleBasicCam1;
    // Set dangling pointers to null.
    ExampleBasicCam1 = nullptr;
}
