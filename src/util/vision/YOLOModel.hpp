/******************************************************************************
 * @brief Defines and implements objects/structs related to opening, interfacing,
 *      and inferencing YOLO models.
 *
 * @file YOLOModel.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-11-11
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef YOLO_MODEL_HPP
#define YOLO_MODEL_HPP

#include "../../AutonomyConstants.h"

/// \cond
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <torch/torch.h>

/// \endcond

/******************************************************************************
 * @brief Namespace containing functions or objects/struct used to aid in easy use
 *      of YOLO models. Namespace can contain other namespace pertaining to the method
 *      of inference or library being used.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-11-11
 ******************************************************************************/
namespace yolomodel
{
    /******************************************************************************
     * @brief This struct is used to
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-11-14
     ******************************************************************************/
    struct Detection
    {
        public:
            /////////////////////////////////////////
            // Define public struct attributes.
            /////////////////////////////////////////

            int nClassID;               // The class index of the object. Dependent on class order when trained.
            std::string szClassName;    // The class name of the object. This is dependent on the class names used when training.
            float fConfidence;          // The detection confidence of the object.
            cv::Rect cvBoundingBox;     // An object used to access the dimensions and other properties of the objects bounding box.
    };

    /******************************************************************************
     * @brief Perform non max suppression for the given predictions. This eliminates/combines
     *      predictions that overlap with each other.
     *
     * @param vObjects - A vector that will be filled with all of the valid/filtered predictions with their data stored in an easy-to-use struct.
     * @param vClassIDs - A reference to a vector that contains class IDs for each prediction.
     * @param vClassConfidences - A reference to a vector that contains the highest class confidence for that prediction.
     * @param vBoundingBoxes - A reference to a vector that contains a cv::Rect bounding box for each prediction.
     * @param fMinObjectConfidence - The minimum confidence for determining which predictions to throw out.
     * @param fNMSThreshold - The threshold value for filtering out weaker bounding boxes or detections.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-11-15
     ******************************************************************************/
    inline void NonMaxSuppression(std::vector<Detection>& vObjects,
                                  std::vector<int>& vClassIDs,
                                  std::vector<float>& vClassConfidences,
                                  std::vector<cv::Rect>& vBoundingBoxes,
                                  float fMinObjectConfidence,
                                  float fNMSThreshold)
    {
        // Create instance variables.
        std::vector<int> vNMSValidIndices;

        // Perform Non-Max Suppression using OpenCV's implementation.
        cv::dnn::NMSBoxes(vBoundingBoxes, vClassConfidences, fMinObjectConfidence, fNMSThreshold, vNMSValidIndices);

        // Loop through each valid index.
        for (int nValidIndex : vNMSValidIndices)
        {
            // Create new Detection struct.
            Detection stNewDetection;
            // Repackage prediction data into easy-to-use struct.
            stNewDetection.nClassID      = vClassIDs[nValidIndex];
            stNewDetection.fConfidence   = vClassConfidences[nValidIndex];
            stNewDetection.cvBoundingBox = vBoundingBoxes[nValidIndex];

            // Append new object detection to objects vector.
            vObjects.emplace_back(stNewDetection);
        }
    }

    /******************************************************************************
     * @brief Given an image and a vector of object structs, draw each object bounding box,
     *      class type, and confidence onto the image.
     *
     * @param cvInputFrame - A reference to the cv::Mat to draw overlays on.
     * @param vObjects - A reference to the vector containing the object detection structs.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-11-15
     ******************************************************************************/
    inline void DrawDetections(cv::Mat& cvInputFrame, std::vector<Detection>& vObjects)
    {
        // Loop through each detection.
        for (Detection stObject : vObjects)
        {
            // Calculate the hue value based on the class ID.
            int nHue = static_cast<int>(stObject.nClassID % 256);
            // Set saturation and value to 1.0 for full intensity colors.
            int nSaturation = 255;
            int nValue      = 255;

            // Convert HSV to RGB
            cv::Mat cvHSV(1, 1, CV_8UC3, cv::Scalar(nHue, nSaturation, nValue));
            cv::cvtColor(cvHSV, cvHSV, cv::COLOR_HSV2BGR);
            // Extract the RGB values
            cv::Vec3b cvConvertedValues = cvHSV.at<cv::Vec3b>(0, 0);
            cv::Scalar cvBoxColor(cvConvertedValues[2], cvConvertedValues[1], cvConvertedValues[0]);

            // Draw bounding box onto image.
            cv::rectangle(cvInputFrame, stObject.cvBoundingBox, cvBoxColor, 2);
            // Draw classID background box onto image.
            cv::rectangle(cvInputFrame,
                          cv::Point(stObject.cvBoundingBox.x, stObject.cvBoundingBox.y - 20),
                          cv::Point(stObject.cvBoundingBox.x + stObject.cvBoundingBox.width, stObject.cvBoundingBox.y),
                          cvBoxColor,
                          cv::FILLED);
            // Draw class text onto image.
            cv::putText(cvInputFrame,
                        std::to_string(stObject.nClassID) + " " + std::to_string(stObject.fConfidence),
                        cv::Point(stObject.cvBoundingBox.x, stObject.cvBoundingBox.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(255, 255, 255));
        }
    }

    /******************************************************************************
     * @brief Namespace containing functions or objects/structs used to run inference on
     *      a YOLO model with the PyTorch library. This namespace contains static functions
     *      for getting available hardware devices, and classes for running a .pt model on each device.
     *      This namespace was built to work with YOLO models only!
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-06
     ******************************************************************************/
    namespace pytorch
    {
        /******************************************************************************
         * @brief This class is designed to enable quick, easy, and robust inferencing of .pt
         *      yolo model.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-06
         ******************************************************************************/
        class PyTorchInterpreter
        {
            public:
                /////////////////////////////////////////
                // Declare public enums that are specific to and used within this class.
                /////////////////////////////////////////
                enum class HardwareDevices
                {
                    eCPU,    // The CPU device.
                    eCUDA    // The CUDA device.
                };

                /////////////////////////////////////////
                // Declare public methods and member variables.
                /////////////////////////////////////////

                /******************************************************************************
                 * @brief Construct a new PyTorchInterpreter object.
                 *
                 * @param szModelPath - The path to the model to open and inference.
                 * @param trDevice - The device to run the model on. Default is CUDA. Other options are CPU and MKLDNN.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-06
                 ******************************************************************************/
                PyTorchInterpreter(std::string szModelPath, HardwareDevices eHardwareDevice = HardwareDevices::eCUDA)
                {
                    // Initialize member variables.
                    m_szModelPath      = szModelPath;
                    m_bReady           = false;
                    m_cvModelInputSize = cv::Size(640, 640);
                    m_szModelTask      = "Unknown";
                    m_vClassLabels     = std::vector<std::string>();

                    // Translate the hardware device enum to a torch device.
                    switch (eHardwareDevice)
                    {
                        case HardwareDevices::eCPU: m_trDevice = torch::kCPU; break;
                        case HardwareDevices::eCUDA: m_trDevice = torch::kCUDA; break;
                        default: m_trDevice = torch::kCPU; break;
                    }

                    // Submit logger message.
                    LOG_INFO(logging::g_qSharedLogger, "Attempting to load model {} onto device {}", szModelPath, m_trDevice.str());

                    // Check if the model path is valid.
                    if (!std::filesystem::exists(szModelPath))
                    {
                        // Submit logger message.
                        LOG_ERROR(logging::g_qSharedLogger, "Model path {} does not exist!", szModelPath);
                        return;
                    }
                    // Check if the device is available.
                    if (!torch::cuda::is_available() && m_trDevice == torch::kCUDA)
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "CUDA device is not available, falling back to CPU.");
                        m_trDevice = torch::kCPU;
                    }
                    else
                    {
                        // Submit logger message.
                        LOG_INFO(logging::g_qSharedLogger, "Using device: {}", m_trDevice.str());
                    }

                    // Finally, attempt to load the model.
                    try
                    {
                        // Load the model and set it to eval mode for inference.
                        torch::jit::ExtraFilesMap trExtraConfigFiles{{"config.txt", ""}};
                        m_trModel = torch::jit::load(szModelPath, m_trDevice, trExtraConfigFiles);
                        m_trModel.eval();

                        // Use nlohmann json to parse the config file.
                        nlohmann::json jConfig = nlohmann::json::parse(trExtraConfigFiles.at("config.txt"));
                        // Get the input image size for the model.
                        m_cvModelInputSize = cv::Size(jConfig["imgsz"][0], jConfig["imgsz"][1]);
                        m_szModelTask      = jConfig["task"];
                        for (const auto& item : jConfig["names"].items())
                        {
                            m_vClassLabels.push_back(item.value());
                        }
                        // Submit the config json as a debug message.
                        LOG_DEBUG(logging::g_qSharedLogger, "Model config: {}", jConfig.dump(4));

                        // If device is CUDA, verify that CUDA execution actually works (e.g. no missing kernel image for device architecture)
                        if (m_trDevice == torch::kCUDA)
                        {
                            try
                            {
                                torch::Tensor trTest = torch::zeros({1, 3, m_cvModelInputSize.height, m_cvModelInputSize.width}, torch::kCUDA);
                                m_trModel.forward({trTest});
                            }
                            catch (const std::exception& e)
                            {
                                LOG_WARNING(logging::g_qSharedLogger, "CUDA execution unavailable on device architecture; falling back to CPU for YOLO model.");
                                m_trDevice = torch::kCPU;
                                m_trModel  = torch::jit::load(szModelPath, m_trDevice, trExtraConfigFiles);
                                m_trModel.eval();
                            }
                        }

                        // Check if the model is empty.
                        if (m_trModel.get_methods().empty())
                        {
                            LOG_ERROR(logging::g_qSharedLogger, "Model is empty! Check if the correct model file was provided.");
                            return;
                        }
                        // Check if the model did not move to the expected device.
                        if (m_trModel.buffers().size() > 0)
                        {
                            // Get the device of the model.
                            torch::Device model_device = m_trModel.buffers().begin().operator->().device();
                            if (model_device.type() != m_trDevice.type())
                            {
                                LOG_ERROR(logging::g_qSharedLogger, "Model did not move to the expected device! Model is on: {}", model_device.str());
                                return;
                            }
                        }

                        // Model is ready for inference.
                        LOG_INFO(logging::g_qSharedLogger,
                                 "Model successfully loaded and set to eval mode. The model is a {} model, and has {} classes.",
                                 m_szModelTask,
                                 m_vClassLabels.size());

                        // Set flag saying we are ready for inference.
                        m_bReady = true;
                    }
                    catch (const std::exception& trError)
                    {
                        LOG_ERROR(logging::g_qSharedLogger, "Error loading model: {}", trError.what());
                    }
                }

                /******************************************************************************
                 * @brief Destroy the PyTorchInterpreter object.
                 *
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-06
                 ******************************************************************************/
                ~PyTorchInterpreter()
                {
                    // Nothing to destroy.
                }

                /******************************************************************************
                 * @brief Given an input image forward the image through the YOLO model to run inference
                 *      on the PyTorch model, then parse and repackage the output tensor data into a vector
                 *      of easy-to-use Detection structs.
                 *
                 * @param cvInputFrame - The RGB camera frame to run detection on.
                 * @param fMinObjectConfidence - Minimum confidence required for an object to be considered a valid detection
                 * @param fNMSThreshold - Threshold for Non-Maximum Suppression, controlling overlap between bounding box predictions.
                 * @return std::vector<Detection> - A vector of structs containing information about the valid object detections in the given image.
                 *
                 * @note The input image MUST BE RGB format, otherwise you will likely experience prediction accuracy problems.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-06
                 ******************************************************************************/
                std::vector<Detection> Inference(const cv::Mat& cvInputFrame, const float fMinObjectConfidence = 0.85, const float fNMSThreshold = 0.6)
                {
                    // Force single-threaded execution (if acceptable for your workload)
                    torch::set_num_threads(1);
                    // Disable gradient calculation during inference.
                    torch::NoGradGuard trNoGrad;
                    // Synchronize inference across threads for this model instance.
                    std::lock_guard<std::mutex> lgLock(m_muInferenceMutex);

                    // Create instance variables.
                    std::vector<Detection> vObjects;

                    // Preprocess the given image and pack int into an image.
                    torch::Tensor trTensorImage = PreprocessImage(cvInputFrame, m_trDevice);

                    // Perform inference.
                    std::vector<torch::jit::IValue> vInputs;
                    vInputs.push_back(trTensorImage);
                    torch::Tensor trOutputTensor;
                    try
                    {
                        trOutputTensor = m_trModel.forward(vInputs).toTensor();
                    }
                    catch (const std::exception& trError)
                    {
                        LOG_ERROR(logging::g_qSharedLogger, "Error running inference: {}", trError.what());
                        return vObjects;
                    }
                    catch (...)
                    {
                        LOG_ERROR(logging::g_qSharedLogger, "Unknown error running inference.");
                        return vObjects;
                    }

                    // Calculate the general stride sizes for YOLO based on input tensor shape.
                    int nImgSize  = m_cvModelInputSize.height;
                    int nP3Stride = std::pow((nImgSize / 8), 2);
                    int nP4Stride = std::pow((nImgSize / 16), 2);
                    int nP5Stride = std::pow((nImgSize / 32), 2);
                    // Calculate the proper prediction length for different YOLO versions.
                    int nYOLOv5AnchorsPerGridPoint = 3;
                    int nYOLOv8AnchorsPerGridPoint = 1;
                    int nYOLOv5TotalPredictionLength =
                        (nP3Stride * nYOLOv5AnchorsPerGridPoint) + (nP4Stride * nYOLOv5AnchorsPerGridPoint) + (nP5Stride * nYOLOv5AnchorsPerGridPoint);
                    int nYOLOv8TotalPredictionLength =
                        (nP3Stride * nYOLOv8AnchorsPerGridPoint) + (nP4Stride * nYOLOv8AnchorsPerGridPoint) + (nP5Stride * nYOLOv8AnchorsPerGridPoint);

                    // Parse the output tensor.
                    std::vector<int> vClassIDs;
                    std::vector<std::string> vClassLabels;
                    std::vector<float> vClassConfidences;
                    std::vector<cv::Rect> vBoundingBoxes;

                    // Get the largest dimension of our output tensor.
                    int nLargestDimension = *std::max_element(trOutputTensor.sizes().begin(), trOutputTensor.sizes().end());
                    // Check if the output tensor is YOLOv5 format.
                    if (nLargestDimension == nYOLOv5TotalPredictionLength)
                    {
                        // Parse inferenced output from tensor.
                        this->ParseTensorOutputYOLOv5(trOutputTensor, vClassIDs, vClassConfidences, vBoundingBoxes, cvInputFrame.size(), fMinObjectConfidence);
                    }
                    // Check if the output tensor is YOLOv8 format.
                    else if (nLargestDimension == nYOLOv8TotalPredictionLength)
                    {
                        // Parse inferenced output from tensor.
                        this->ParseTensorOutputYOLOv8(trOutputTensor, vClassIDs, vClassConfidences, vBoundingBoxes, cvInputFrame.size(), fMinObjectConfidence);
                    }

                    // Perform NMS to filter out bad/duplicate detections.
                    NonMaxSuppression(vObjects, vClassIDs, vClassConfidences, vBoundingBoxes, fMinObjectConfidence, fNMSThreshold);

                    // Loop through the final detections and set the class names for each detection based on the class ID.
                    for (size_t nIter = 0; nIter < vObjects.size(); ++nIter)
                    {
                        // Check if the class ID is valid.
                        if (vClassIDs[nIter] >= 0 && vClassIDs[nIter] < static_cast<int>(m_vClassLabels.size()))
                        {
                            vObjects[nIter].szClassName = m_vClassLabels[vClassIDs[nIter]];
                        }
                        else
                        {
                            vObjects[nIter].szClassName = "UnknownClass";
                        }
                    }

                    return vObjects;
                }

                /******************************************************************************
                 * @brief Check if the model is ready for inference.
                 *
                 * @return true - Model is ready for inference.
                 * @return false - Model is not ready for inference.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-02-13
                 ******************************************************************************/
                bool IsReadyForInference() const { return m_bReady; }

            private:
                /////////////////////////////////////////
                // Declare private methods.
                /////////////////////////////////////////

                /******************************************************************************
                 * @brief Given an input image, preprocess the image to match the input tensor shape
                 *      of the model, then return the preprocessed image as a tensor.
                 *
                 * @param cvInputFrame - The input image to preprocess.
                 * @param trDevice - The device to run the model on.
                 * @return torch::Tensor - The preprocessed image as a tensor.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-03-08
                 ******************************************************************************/
                torch::Tensor PreprocessImage(const cv::Mat& cvInputFrame, const torch::Device& trDevice)
                {
                    // Resize the input image to match model and normalize it to 0-1.
                    cv::Mat cvResizedImage;
                    cv::resize(cvInputFrame, cvResizedImage, cv::Size(m_cvModelInputSize.width, m_cvModelInputSize.height), cv::INTER_LINEAR);
                    cvResizedImage.convertTo(cvResizedImage, CV_32FC3, 1.0 / 255.0);

                    // Convert OpenCV mat to a tensor.
                    torch::Tensor trTensorImage = torch::from_blob(cvResizedImage.data, {1, cvResizedImage.rows, cvResizedImage.cols, 3}, torch::kFloat);
                    trTensorImage               = trTensorImage.permute({0, 3, 1, 2}).clone();    // Convert to CxHxW format and clone into owned contiguous memory.
                    if (trDevice != torch::kCPU)
                    {
                        trTensorImage = trTensorImage.to(trDevice);             // Move tensor to the specified hardware device.
                    }

                    return trTensorImage;
                }

                /******************************************************************************
                 * @brief Given a tensor output from a YOLOv5 model, parse it's output into something more usable.
                 *
                 * @param trOutput - A reference to the output tensor from the model. The tensor should be of shape [1, 25200, 85] for YOLOv5.
                 * @param vClassIDs - A reference to a vector that will be filled with class IDs for each prediction. The class ID of a prediction will be chosen
                 * @param vClassConfidences - A reference to a vector that will be filled with the highest class confidence for
                 * @param vBoundingBoxes - A reference to a vector that will be filled with cv::Rect bounding box for each prediction.
                 * @param cvInputFrameSize - The size of the original input frame. This is used to scale the bounding boxes back to the original image size.
                 * @param fMinObjectConfidence - The minimum confidence for determining which predictions to keep. Predictions with a confidence below this value will be
                 *discarded.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-03-13
                 ******************************************************************************/
                void ParseTensorOutputYOLOv5(const torch::Tensor& trOutput,
                                             std::vector<int>& vClassIDs,
                                             std::vector<float>& vClassConfidences,
                                             std::vector<cv::Rect>& vBoundingBoxes,
                                             const cv::Size& cvInputFrameSize,
                                             const float fMinObjectConfidence)
                {
                    /*
                     * For YOLOv5, you divide your image size, i.e. 640 by the P3, P4, P5 output strides of 8, 16, 32 to arrive at grid sizes
                     * of 80x80, 40x40, 20x20. Each grid point has 3 anchors by default (anchor box values: small, medium, large), and each anchor contains a vector 5 +
                     * nc long, where nc is the number of classes the model has. So for a 640 image, the output tensor will be [1, 25200, 85]
                     */
                    // Squeeze the batch dimension from the output tensor.
                    torch::Tensor trSqueezedOutput = trOutput.squeeze(0);

                    // Move the tensor to CPU if necessary. If we're using GPU and we don't move the tensor to CPU, we will get an error and it will be slow.
                    if (trSqueezedOutput.device().is_cuda())
                    {
                        trSqueezedOutput = trSqueezedOutput.to(torch::kCPU);
                    }
                    // Convert tensor to float if necessary.
                    if (trSqueezedOutput.scalar_type() != torch::kFloat32)
                    {
                        trSqueezedOutput = trSqueezedOutput.to(torch::kFloat32);
                    }
                    // Ensure tensor is contiguous in memory.
                    if (!trSqueezedOutput.is_contiguous())
                    {
                        trSqueezedOutput = trSqueezedOutput.contiguous();
                    }

                    // Create an accessor for fast element-wise access.
                    at::TensorAccessor trAccessor = trSqueezedOutput.accessor<float, 2>();
                    const int nNumDetections      = trSqueezedOutput.size(0);
                    const int nTotalValues        = trSqueezedOutput.size(1);    // equals 5 + number_of_classes

                    // Loop through each detection.
                    for (int i = 0; i < nNumDetections; i++)
                    {
                        // Get the objectness confidence. This is the 5th value for each grid/anchor prediction. (4th index)
                        float fObjectnessConfidence = trAccessor[i][4];

                        // Check if the object confidence is greater than or equal to the threshold.
                        if (fObjectnessConfidence < fMinObjectConfidence)
                        {
                            continue;
                        }

                        // Retrieve bounding box data.
                        float fCenterX = trAccessor[i][0];
                        float fCenterY = trAccessor[i][1];
                        float fWidth   = trAccessor[i][2];
                        float fHeight  = trAccessor[i][3];

                        // Scale bounding box to original image size.
                        int nLeft           = static_cast<int>((fCenterX - (0.5 * fWidth)) * cvInputFrameSize.width);
                        int nTop            = static_cast<int>((fCenterY - (0.5 * fHeight)) * cvInputFrameSize.height);
                        int nBoundingWidth  = static_cast<int>(fWidth * cvInputFrameSize.width);
                        int nBoundingHeight = static_cast<int>(fHeight * cvInputFrameSize.height);

                        // Repackaged bounding box data to be more readable.
                        cv::Rect cvBoundingBox(nLeft, nTop, nBoundingWidth, nBoundingHeight);

                        // Loop over class confidence values and find the class ID with the highest confidence.
                        float fClassConfidence = -1.0f;
                        int nClassID           = -1;
                        for (int j = 5; j < nTotalValues; j++)
                        {
                            float fConfidence = trAccessor[i][j];
                            if (fConfidence > fClassConfidence)
                            {
                                fClassConfidence = fConfidence;
                                nClassID         = j - 5;
                            }
                        }

                        // Only process detections that meet the minimum confidence.
                        if (fClassConfidence < fMinObjectConfidence)
                        {
                            continue;
                        }

                        // Add data to vectors.
                        vClassIDs.emplace_back(nClassID);
                        vClassConfidences.emplace_back(fClassConfidence);
                        vBoundingBoxes.emplace_back(cvBoundingBox);
                    }
                }

                /******************************************************************************
                 * @brief Given a tensor output from a YOLOv5 model, parse it's output into something more usable.
                 *
                 * @param trOutput - A reference to the output tensor from the model.
                 * @param vClassIDs - A reference to a vector that will be filled with class IDs for each prediction. The class ID of a prediction will be choosen
                 *             by the highest class confidence for that prediction.
                 * @param vClassConfidences - A reference to a vector that will be filled with the highest class confidence for that prediction.
                 * @param vBoundingBoxes - A reference to a vector that will be filled with cv::Rect bounding box for each prediction.
                 * @param cvInputFrameSize - The size of the original input frame before resizing. This is used to scale the bounding box back to the original size.
                 * @param fMinObjectConfidence - The minimum confidence required for an object to be considered a valid detection.
                 *
                 * @note For YOLOv8, you divide your image size, i.e. 640 by the P3, P4, P5 output strides of 8, 16, 32 to arrive at grid sizes
                 *       of 80x80, 40x40, 20x20. Each grid point has 1 anchor, and each anchor contains a vector 4 + nc long, where nc is the number
                 *       of classes the model has. So for a 640 image, the output tensor will be [1, 84, 8400] (80 classes). Notice how the larger dimensions is swapped
                 *       when compared to YOLOv8.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-03-08
                 ******************************************************************************/
                void ParseTensorOutputYOLOv8(const torch::Tensor& trOutput,
                                             std::vector<int>& vClassIDs,
                                             std::vector<float>& vClassConfidences,
                                             std::vector<cv::Rect>& vBoundingBoxes,
                                             const cv::Size& cvInputFrameSize,
                                             const float fMinObjectConfidence)
                {
                    /*
                     * Permute the output tensor shape to match the expected format of the model. If the model is YOLOv8, the output
                     * shape for a 640x640 image will be [1, 4 + nc, 8400] (nc = number of classes). Notice how the larger dimensions is swapped
                     * when compared to YOLOv5. We will permute the tensor to [1, 8400, 4 + nc] to make it easier to parse. Then squeeze the
                     * tensor to remove the batch dimension so the final shape will be [8400, 4 + nc]. Thanks pytorch for being cool with the
                     * permute function.
                     */
                    // Permute the tensor shape from [1, 4 + nc, 8400] to [1, 8400, 4 + nc]
                    // and then squeeze to remove the batch dimension, resulting in [8400, 4 + nc]
                    torch::Tensor trPermuteOutput = trOutput.permute({0, 2, 1}).squeeze(0);

                    // Move tensor to CPU if necessary. If we're using GPU and we don't move the tensor to CPU, we will get an error and it will be slow.
                    if (trPermuteOutput.device().is_cuda())
                    {
                        trPermuteOutput = trPermuteOutput.to(torch::kCPU);
                    }
                    // Convert tensor to float if necessary.
                    if (trPermuteOutput.scalar_type() != torch::kFloat32)
                    {
                        trPermuteOutput = trPermuteOutput.to(torch::kFloat32);
                    }
                    // Ensure tensor is contiguous in memory.
                    if (!trPermuteOutput.is_contiguous())
                    {
                        trPermuteOutput = trPermuteOutput.contiguous();
                    }

                    // Create an accessor for fast element-wise access.
                    at::TensorAccessor trAccessor = trPermuteOutput.accessor<float, 2>();
                    const int nNumDetections      = trPermuteOutput.size(0);
                    const int nTotalValues        = trPermuteOutput.size(1);    // equals 4 + number_of_classes

                    // Loop through each detection.
                    for (int i = 0; i < nNumDetections; i++)
                    {
                        float fClassConfidence = -1.0f;
                        int nClassID           = -1;

                        // Loop over class confidence values.
                        for (int j = 4; j < nTotalValues; j++)
                        {
                            float fConfidence = trAccessor[i][j];
                            if (fConfidence > fClassConfidence)
                            {
                                fClassConfidence = fConfidence;
                                nClassID         = j - 4;
                            }
                        }

                        // Only process detections that meet the minimum confidence.
                        if (fClassConfidence < fMinObjectConfidence)
                        {
                            continue;
                        }

                        // Retrieve bounding box data.
                        float fCenterX = trAccessor[i][0];
                        float fCenterY = trAccessor[i][1];
                        float fWidth   = trAccessor[i][2];
                        float fHeight  = trAccessor[i][3];

                        // Scale bounding box to original image size.
                        int nLeft      = static_cast<int>(fCenterX * cvInputFrameSize.width / 640.0f - (0.5f * fWidth * cvInputFrameSize.width / 640.0f));
                        int nTop       = static_cast<int>(fCenterY * cvInputFrameSize.height / 640.0f - (0.5f * fHeight * cvInputFrameSize.height / 640.0f));
                        int nBoxWidth  = static_cast<int>(fWidth * cvInputFrameSize.width / 640.0f);
                        int nBoxHeight = static_cast<int>(fHeight * cvInputFrameSize.height / 640.0f);
                        cv::Rect cvBoundingBox(nLeft, nTop, nBoxWidth, nBoxHeight);

                        // Append results.
                        vClassIDs.push_back(nClassID);
                        vClassConfidences.push_back(fClassConfidence);
                        vBoundingBoxes.push_back(cvBoundingBox);
                    }
                }

                /////////////////////////////////////////
                // Declare private member variables.
                /////////////////////////////////////////
                torch::jit::script::Module m_trModel;
                torch::Device m_trDevice = torch::kCPU;
                std::string m_szModelPath;
                bool m_bReady;
                std::string m_szModelTask;
                cv::Size m_cvModelInputSize;
                std::vector<std::string> m_vClassLabels;
                std::mutex m_muInferenceMutex;
        };
    }    // namespace pytorch
}    // namespace yolomodel

#endif
