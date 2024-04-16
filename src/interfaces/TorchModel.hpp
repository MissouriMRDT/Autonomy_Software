/******************************************************************************
 * @brief
 *
 * @file TorchModel.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-04-10
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef TORCH_MODEL_HPP
#define TORCH_MODEL_HPP

#include "../AutonomyLogging.h"

/// \cond
#include <iostream>
#include <memory>
#include <torch/script.h>
#include <torch/torch.h>

/// \endcond

template<typename T, typename P>
class TorchModel
{
    public:
        enum DeviceType
        {
            eCPU,
            eGPU
        };

        TorchModel(const std::string& szModelPath) { m_szModelPath = szModelPath; }

        ~TorchModel() {}

        void LoadModel(DeviceType eDeviceType = eGPU)
        {
            m_eDevice = (eDeviceType == eGPU && torch::cuda::is_available()) ? torch::kCUDA : torch::kCPU;
            m_ltModel = torch::jit::load(m_szModelPath, m_eDevice);
            m_ltModel->to(m_eDevice);
        }

        std::shared_ptr<torch::jit::script::Module> GetModel() { return m_ltModel; }

        torch::Device GetDevice() { return m_eDevice; }

        std::string GetPath() { return m_szModelPath; }

    protected:
        virtual T Inference(const P& tInput, const float fMinObjectConfidence, const float fNMSThreshold) = 0;

    private:
        std::shared_ptr<torch::jit::script::Module> m_ltModel;
        torch::Device m_eDevice;
        std::string m_szModelPath;
};

#endif    // TORCH_MODEL_HPP
