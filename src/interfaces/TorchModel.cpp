/******************************************************************************
 * @brief
 *
 * @file TorchModel.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-04-10
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "TorchModel.h"
#include <string>

TorchModel::TorchModel(const std::string& szModelPath, const torch::DeviceType& eDeviceType)
{
    m_szModelPath = szModelPath;
    m_eDevice     = eDeviceType;

    try
    {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        m_ltModel = torch::jit::load(m_szModelPath);
    }
    catch (const c10::Error& e)
    {
        std::cerr << "Error loading the model!\n\n" << e.msg();
        std::exit(EXIT_FAILURE);
    }

    m_bHalfModel = (m_eDevice != torch::kCPU);
    m_ltModel.to(m_eDevice);

    if (m_bHalfModel)
    {
        m_ltModel.to(torch::kHalf);
    }

    m_ltModel.eval();
}

torch::jit::script::Module TorchModel::GetModel()
{
    return m_ltModel;
}

torch::DeviceType TorchModel::GetDevice()
{
    return m_eDevice;
}

std::string TorchModel::GetPath()
{
    return m_szModelPath;
}

bool TorchModel::IsHalf()
{
    return m_bHalfModel;
}

std::vector<float> TorchModel::LetterboxImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size)
{
    auto in_h   = static_cast<float>(src.rows);
    auto in_w   = static_cast<float>(src.cols);
    float out_h = out_size.height;
    float out_w = out_size.width;

    float scale = std::min(out_w / in_w, out_h / in_h);

    int mid_h   = static_cast<int>(in_h * scale);
    int mid_w   = static_cast<int>(in_w * scale);

    cv::resize(src, dst, cv::Size(mid_w, mid_h));

    int top   = (static_cast<int>(out_h) - mid_h) / 2;
    int down  = (static_cast<int>(out_h) - mid_h + 1) / 2;
    int left  = (static_cast<int>(out_w) - mid_w) / 2;
    int right = (static_cast<int>(out_w) - mid_w + 1) / 2;

    cv::copyMakeBorder(dst, dst, top, down, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    std::vector<float> pad_info{static_cast<float>(left), static_cast<float>(top), scale};
    return pad_info;
}

void TorchModel::ScaleCoordinates(std::vector<constants::Detection>& data, float pad_w, float pad_h, float scale, const cv::Size& img_shape)
{
    auto clip = [](float n, float lower, float upper)
    {
        return std::max(lower, std::min(n, upper));
    };

    std::vector<constants::Detection> detections;
    for (auto& i : data)
    {
        float x1        = (i.cvBoundingBox.tl().x - pad_w) / scale;    // x padding
        float y1        = (i.cvBoundingBox.tl().y - pad_h) / scale;    // y padding
        float x2        = (i.cvBoundingBox.br().x - pad_w) / scale;    // x padding
        float y2        = (i.cvBoundingBox.br().y - pad_h) / scale;    // y padding

        x1              = clip(x1, 0, img_shape.width);
        y1              = clip(y1, 0, img_shape.height);
        x2              = clip(x2, 0, img_shape.width);
        y2              = clip(y2, 0, img_shape.height);

        i.cvBoundingBox = cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));
    }
}
