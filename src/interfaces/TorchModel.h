/******************************************************************************
 * @brief
 *
 * @file TorchModel.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-04-10
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef TORCH_MODEL_H
#define TORCH_MODEL_H

#include "../AutonomyConstants.h"
#include "../AutonomyLogging.h"

/// \cond
#include <memory>

#include <torch/script.h>
#include <torch/torch.h>

#include <ATen/cuda/CUDAEvent.h>
#include <c10/cuda/CUDAStream.h>

#include <opencv2/core.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

/// \endcond

class TorchModel
{
    public:
        TorchModel(const std::string& szModelPath, const torch::DeviceType& eDeviceType);

        ~TorchModel() {}

        torch::jit::script::Module GetModel();

        torch::DeviceType GetDevice();

        std::string GetPath();

        bool IsHalf();

        std::vector<float> LetterboxImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size);
        void ScaleCoordinates(std::vector<constants::Detection>& data, float pad_w, float pad_h, float scale, const cv::Size& img_shape);

    protected:
        virtual std::vector<std::vector<constants::Detection>> PostProcessing(const torch::Tensor& detections,
                                                                              float pad_w,
                                                                              float pad_h,
                                                                              float scale,
                                                                              const cv::Size& img_shape,
                                                                              float conf_thres,
                                                                              float iou_thres)                                                                     = 0;

        virtual std::vector<std::vector<constants::Detection>> Inference(const cv::Mat& cvInputFrame, const float fMinObjectConfidence, const float fNMSThreshold) = 0;

        torch::jit::script::Module m_ltModel;
        torch::DeviceType m_eDevice;
        std::string m_szModelPath;
        bool m_bHalfModel;
};

#endif    // TORCH_MODEL_H
