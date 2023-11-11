/******************************************************************************
 * @brief Defines and implements objects/structs related to opening, interfacing,
 *      and inferencing tensorflow models. All contained within the TensorflowModel
 *      namespace.
 *
 * @file TensorflowModel.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-11-11
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef TENSORFLOW_MODEL_HPP
#define TENSORFLOW_MODEL_HPP

#include "../../AutonomyLogging.h"

#include <edgetpu.h>
#include <tensorflow/lite/builtin_ops.h>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <vector>

/******************************************************************************
 * @brief Namespace containing functions or objects/struct used to aid in easy use
 *      of tensorflow models. This namespace contains static functions for getting available
 *      hardware devices, and classes for running a .tflite model on each device.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-11-11
 ******************************************************************************/
namespace tensorflowmodel
{
    /******************************************************************************
     * @brief This class is designed to enable quick, easy, and robust handling of .tflite
     *      models for deployment and inference on the Coral EdgeTPU Accelerator.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-24
     ******************************************************************************/
    class EdgeTPU
    {
        public:
            /////////////////////////////////////////
            // Declare public enums that are specific to and used withing this class.
            /////////////////////////////////////////

            // Enumerator for selecting which EdgeTPU device type the model should try to run on.
            enum DeviceType
            {
                eAuto,    // Any open device will be picked. Prioritizes PCIe device if not already in use.
                ePCIe,    // Attempt to use a PCIe device for this model.
                eUSB      // Attempt to use a USB device for this model.
            };

            // Enumerator for selecting performance mode of EdgeTPU devices.
            enum PerformanceModes
            {
                eLow,       // Power saver mode. Low power draw and little heat output, but not great performance.
                eMedium,    // Balanced. Medium power draw, medium performance.
                eHigh,      // Performance mode. High power draw and increased heat output, great performance.
                eMax        // Maximum clock speed. Max performance, but greatest power draw and heat output. Could damage device in hot environments.
            };

            /////////////////////////////////////////
            // Declare public methods and member variables.
            /////////////////////////////////////////

            /******************************************************************************
             * @brief Construct a new Edge T P U object.
             *
             * @param szModelPath - The path to the model to open and inference on the EdgeTPU.
             * @param ePowerMode - The desired power mode of the device.
             * @param unMaxBulkInQueueLength - Input queue length for device. Larger queue may improve USB
             *                   performance going from device to host.
             * @param bUSBAlwaysDFU - Whether or not to always reload firmware into the device after when this object is created.
             *
             * @note The given model must be a tflite model custom compiled to map operations to the EdgeTPU refer to
             *          https://coral.ai/docs/edgetpu/models-intro/#compiling and
             *          https://coral.ai/docs/edgetpu/compiler/#system-requirements
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-11-11
             ******************************************************************************/
            EdgeTPU(std::string szModelPath, PerformanceModes ePowerMode = eHigh, unsigned int unMaxBulkInQueueLength = 32, bool bUSBAlwaysDFU = false)
            {
                // Initialize member variables.
                m_szModelPath                                  = szModelPath;
                m_tpuDeviceOptions["Usb.MaxBulkInQueueLength"] = std::to_string(unMaxBulkInQueueLength);
                m_bDeviceOpened                                = false;

                // Determine which power mode should be set.
                switch (ePowerMode)
                {
                    case eLow: m_tpuDeviceOptions["Performance"] = "Low"; break;
                    case eMedium: m_tpuDeviceOptions["Performance"] = "Medium"; break;
                    case eHigh: m_tpuDeviceOptions["Performance"] = "High"; break;
                    case eMax: m_tpuDeviceOptions["Performance"] = "Max"; break;
                    default: m_tpuDeviceOptions["Performance"] = "High"; break;
                }

                // Determine if firmware should be loaded ever time code is started.
                if (bUSBAlwaysDFU)
                {
                    // Always load firmware.
                    m_tpuDeviceOptions["Usb.AlwaysDfu"] = "True";
                }
                else
                {
                    // Only load firmware on first init of device.
                    m_tpuDeviceOptions["Usb.AlwaysDfu"] = "False";
                }
            }

            /******************************************************************************
             * @brief Destroy the EdgeTPU object.
             *
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-10-24
             ******************************************************************************/
            ~EdgeTPU()
            {
                // Nothing to destroy.
            }

            /******************************************************************************
             * @brief Attempt to open the model at the given path and load it onto the EdgeTPU device.
             *
             * @param eDeviceType - An enumerator specifying which device this model should run on. (PCIe, USB, or autoselect)
             * @return TfLiteStatus - The Tensorflow Lite status of the model interpreter. Status will be TfLiteOk if
             *      model was successfully opened and loaded onto the device.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-11-11
             ******************************************************************************/
            TfLiteStatus OpenAndLoad(DeviceType eDeviceType = eAuto)
            {
                // Create instance variables.
                TfLiteStatus tfReturnStatus = TfLiteStatus::kTfLiteCancelled;
                std::vector<edgetpu::EdgeTpuManager::DeviceEnumerationRecord> vValidDevices;

                // Determine which device is going to be used for this model.
                switch (eDeviceType)
                {
                    case eAuto: m_tpuDevice.type = edgetpu::DeviceType(-1); break;
                    case ePCIe: m_tpuDevice.type = edgetpu::DeviceType::kApexPci; break;
                    case eUSB: m_tpuDevice.type = edgetpu::DeviceType::kApexUsb; break;
                    default: m_tpuDevice.type = edgetpu::DeviceType(-1); break;
                }

                // Load compiled Edge TPU model as a flatbuffer model.
                m_pTFLiteModel = tflite::FlatBufferModel::VerifyAndBuildFromFile(m_szModelPath.c_str());
                // Check if model was successfully opened.
                if (m_pTFLiteModel != nullptr)
                {
                    // Get a list of available devices and already opened devices.
                    std::vector<edgetpu::EdgeTpuManager::DeviceEnumerationRecord> vDevices      = this->GetHardwareDevices();
                    std::vector<std::shared_ptr<edgetpu::EdgeTpuContext>> vAlreadyOpenedDevices = this->GetOpenedHardwareDevices();

                    // Get list of valid, unopened devices.
                    // Loop through available devices.
                    for (unsigned int nIter = 0; nIter < vDevices.size(); ++nIter)
                    {
                        // Create instance variables.
                        bool bValidDevice = true;

                        // Loop through all opened devices.
                        for (unsigned int nJter = 0; nJter < vAlreadyOpenedDevices.size(); ++nJter)
                        {
                            // Check if current available device has already been opened.
                            if (vAlreadyOpenedDevices[nJter]->GetDeviceEnumRecord().path == vDevices[nIter].path)
                            {
                                // Set device as not valid.
                                bValidDevice = false;
                            }
                            // Determine if we should check device type.
                            else if (eDeviceType != eAuto)
                            {
                                // Check if device type matches.
                                if (vDevices[nIter].type != m_tpuDevice.type)
                                {
                                    // Set device as not valid.
                                    bValidDevice = false;
                                }
                            }
                        }

                        // Check if still valid.
                        if (bValidDevice)
                        {
                            // Append to valid devices vector.
                            vValidDevices.emplace_back(vDevices[nIter]);
                        }
                    }

                    // Check if any valid devices were found.
                    if (vValidDevices.size() > 0)
                    {
                        // Loop through each device until one successfully opens.
                        for (unsigned int nIter = 0; nIter < vValidDevices.size() && !m_bDeviceOpened; ++nIter)
                        {
                            // Submit logger message.
                            LOG_INFO(logging::g_qSharedLogger,
                                     "Attempting to load {} onto {} device at {} ({})...",
                                     m_szModelPath,
                                     this->DeviceTypeToString(vValidDevices[nIter].type),
                                     vValidDevices[nIter].path,
                                     this->DeviceTypeToString(vValidDevices[nIter].type));

                            // Attempt to open device.
                            m_pEdgeTPUContext = this->GetEdgeManager()->OpenDevice(vValidDevices[nIter].type, vValidDevices[nIter].path, m_tpuDeviceOptions);

                            // Only proceed if device opened.
                            if (m_pEdgeTPUContext->IsReady())
                            {
                                // Create custom tflite operations for edge tpu.
                                tflite::ops::builtin::BuiltinOpResolverWithXNNPACK tfResolver;
                                tfResolver.AddCustom(edgetpu::kCustomOp, edgetpu::RegisterCustomOp());
                                // Create tflite interpreter with model and operations resolver.
                                if (tflite::InterpreterBuilder(*m_pTFLiteModel, tfResolver)(&m_pInterpreter) != kTfLiteOk)
                                {
                                    // Submit logger message.
                                    LOG_WARNING(logging::g_qSharedLogger,
                                                "Unable to build interpreter for model {} with device {} ({})",
                                                m_szModelPath,
                                                vValidDevices[nIter].path,
                                                this->DeviceTypeToString(vValidDevices[nIter].type));

                                    // Release interpreter and context.
                                    m_pInterpreter.reset();
                                    m_pEdgeTPUContext.reset();

                                    // Update return status.
                                    tfReturnStatus = TfLiteStatus::kTfLiteUnresolvedOps;
                                }
                                else
                                {
                                    // Bind the given context device with interpreter.
                                    m_pInterpreter->SetExternalContext(kTfLiteEdgeTpuContext, m_pEdgeTPUContext.get());
                                    // Attempt to allocate necessary tensors for model onto device.
                                    if (m_pInterpreter->AllocateTensors() != kTfLiteOk)
                                    {
                                        // Submit logger message.
                                        LOG_WARNING(
                                            logging::g_qSharedLogger,
                                            "Even though device was opened and interpreter was built, allocation of tensors failed for model {} with device {} ({})",
                                            m_szModelPath,
                                            vValidDevices[nIter].path,
                                            this->DeviceTypeToString(vValidDevices[nIter].type));

                                        // Release interpreter and context.
                                        m_pInterpreter.reset();
                                        m_pEdgeTPUContext.reset();

                                        // Update return status.
                                        tfReturnStatus = TfLiteStatus::kTfLiteDelegateDataWriteError;
                                    }
                                    else
                                    {
                                        // Submit logger message.
                                        LOG_WARNING(logging::g_qSharedLogger,
                                                    "Successfully opened and loaded model {} with device {} ({})",
                                                    m_szModelPath,
                                                    vValidDevices[nIter].path,
                                                    this->DeviceTypeToString(vValidDevices[nIter].type));

                                        // Set toggle that model is opened with device.
                                        m_bDeviceOpened = true;
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        // Submit logger message.
                        LOG_ERROR(logging::g_qSharedLogger,
                                  "No valid devices were found for model {}. Device type is {}",
                                  m_szModelPath,
                                  this->DeviceTypeToString(m_tpuDevice.type));
                    }
                }
                else
                {
                    // Submit logger message.
                    LOG_ERROR(logging::g_qSharedLogger, "Unable to load model {}. Is this actually compiled for the EdgeTPU?", m_szModelPath);
                }

                // Return status.
                return tfReturnStatus;
            }

            /////////////////////////////////////////
            // Setters
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Getters
            /////////////////////////////////////////

            /******************************************************************************
             * @brief Accessor for the Device Is Opened private member.
             *
             * @return true - Model has been successfully opened and loaded onto device.
             * @return false - Model has not yet been opened or loaded onto a device.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-11-11
             ******************************************************************************/
            bool GetDeviceIsOpened() const { return m_bDeviceOpened; }

            /******************************************************************************
             * @brief Retrieve a list of EdgeTPU devices from the edge API.
             *
             * @return std::vector<edgetpu::EdgeTpuManager::DeviceEnumerationRecord> - A vector containing
             *      device records for currently connected devices. Each device record contains a type (usb/pcie) and path.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-11-11
             ******************************************************************************/
            static std::vector<edgetpu::EdgeTpuManager::DeviceEnumerationRecord> GetHardwareDevices()
            {
                // Create instance variables.
                edgetpu::EdgeTpuManager* tpuEdgeManagerInstance = edgetpu::EdgeTpuManager::GetSingleton();

                // Check if edgetpu singleton objects are supported.
                if (tpuEdgeManagerInstance != nullptr)
                {
                    // Get a list of devices from the edgetpu api.
                    return tpuEdgeManagerInstance->EnumerateEdgeTpu();
                }
                else
                {
                    // Return empty vector.
                    return std::vector<edgetpu::EdgeTpuManager::DeviceEnumerationRecord>();
                }
            }

            /******************************************************************************
             * @brief Retrieve a list of already opened EdgeTPU devices from the edge API.
             *
             * @return std::vector<edgetpu::EdgeTpuManager::DeviceEnumerationRecord> - A vector containing
             *      device records for currently opened devices. Each device record contains a type (usb/pcie) and path.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-11-11
             ******************************************************************************/
            static std::vector<std::shared_ptr<edgetpu::EdgeTpuContext>> GetOpenedHardwareDevices()
            {
                // Create instance variables.
                edgetpu::EdgeTpuManager* tpuEdgeManagerInstance = edgetpu::EdgeTpuManager::GetSingleton();

                // Check if edgetpu singleton objects are supported.
                if (tpuEdgeManagerInstance != nullptr)
                {
                    // Get a list of devices from the edgetpu api.
                    return tpuEdgeManagerInstance->GetOpenedDevices();
                }
                else
                {
                    // Return empty vector.
                    return std::vector<std::shared_ptr<edgetpu::EdgeTpuContext>>();
                }
            }

        private:
            /////////////////////////////////////////
            // Declare private methods.
            /////////////////////////////////////////

            /******************************************************************************
             * @brief Retrieves a pointer to an EdgeTPUManager instance from the libedgetpu library.
             *
             * @return edgetpu::EdgeTpuManager* - A pointer to the manager. Will be nullptr if not supported on
             *          this operating system.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-11-11
             ******************************************************************************/
            edgetpu::EdgeTpuManager* GetEdgeManager()
            {
                // Create instance variables.
                edgetpu::EdgeTpuManager* tpuEdgeManagerInstance = edgetpu::EdgeTpuManager::GetSingleton();

                // Check if edgetpu singleton objects are supported.
                if (tpuEdgeManagerInstance == nullptr)
                {
                    // Submit logger message.
                    LOG_CRITICAL(logging::g_qSharedLogger, "Unable to get EdgeTPU manager! This operating system does not support singletons.");
                }

                // Get a list of devices from the edgetpu api.
                return tpuEdgeManagerInstance;
            }

            /******************************************************************************
             * @brief to_string method for converting a device type to a readable string.
             *
             * @param eDeviceType - The edgetpu device type. (kApexUsb or kApexPci)
             * @return std::string - The equivalent string. (USB or PCIe)
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-11-11
             ******************************************************************************/
            std::string DeviceTypeToString(edgetpu::DeviceType eDeviceType)
            {
                // Determine which device type string should be returned.
                switch (eDeviceType)
                {
                    case edgetpu::DeviceType::kApexUsb: return "USB";
                    case edgetpu::DeviceType::kApexPci: return "PCIe";
                    default: return "Not Found";
                }
            }

            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////

            std::string m_szModelPath;
            edgetpu::EdgeTpuManager::DeviceEnumerationRecord m_tpuDevice;
            edgetpu::EdgeTpuManager::DeviceOptions m_tpuDeviceOptions;
            std::unique_ptr<tflite::FlatBufferModel> m_pTFLiteModel;
            std::shared_ptr<edgetpu::EdgeTpuContext> m_pEdgeTPUContext;
            std::unique_ptr<tflite::Interpreter> m_pInterpreter;
            bool m_bDeviceOpened;
    };
}    // namespace tensorflowmodel

#endif
