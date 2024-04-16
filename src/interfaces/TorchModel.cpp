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

template<typename T, typename P>
TorchModel<T, P>::TorchModel(const std::string& szModelPath)
{
    m_szModelPath = szModelPath;
}

template<typename T, typename P>
void TorchModel<T, P>::LoadModel(TorchModel<T, P>::DeviceType eDeviceType)
{
    m_eDevice = (eDeviceType == eGPU && torch::cuda::is_available()) ? torch::kCUDA : torch::kCPU;
    m_ltModel = torch::jit::load(m_szModelPath, m_eDevice);
    m_ltModel->to(m_eDevice);
}

template<typename T, typename P>
std::shared_ptr<torch::jit::script::Module> TorchModel<T, P>::GetModel()
{
    return m_ltModel;
}

template<typename T, typename P>
torch::Device TorchModel<T, P>::GetDevice()
{
    return m_eDevice;
}

template<typename T, typename P>
std::string TorchModel<T, P>::GetPath()
{
    return m_szModelPath;
}
