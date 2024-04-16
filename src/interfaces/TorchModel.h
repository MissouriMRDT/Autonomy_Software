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

        TorchModel(const std::string& szModelPath);

        ~TorchModel();

        void LoadModel(DeviceType eDeviceType = eGPU);

        std::shared_ptr<torch::jit::script::Module> GetModel();

        torch::Device GetDevice();

        std::string GetPath();

    protected:
        virtual T Inference(const P& tInput, const float fMinObjectConfidence, const float fNMSThreshold) = 0;

    private:
        std::shared_ptr<torch::jit::script::Module> m_ltModel;
        torch::Device m_eDevice;
        std::string m_szModelPath;
};

#endif    // TORCH_MODEL_H
