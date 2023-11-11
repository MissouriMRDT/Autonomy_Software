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
#include <tensorflow/lite/interpreter.h>
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
             * @param eDeviceType - An enumerator specifying which device this model should run on. (PCIe, USB, or autoselect)
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-11-11
             ******************************************************************************/
            EdgeTPU(std::string szModelPath,
                    PerformanceModes ePowerMode         = eHigh,
                    unsigned int unMaxBulkInQueueLength = 32,
                    bool bUSBAlwaysDFU                  = false,
                    DeviceType eDeviceType              = eAuto)
            {
                // Initialize member variables.
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

            /////////////////////////////////////////
            // Setters
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Getters
            /////////////////////////////////////////

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
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "Unable to enumerate EdgeTPU devices! This operating system does not support singletons.");

                    // Return empty vector.
                    return std::vector<edgetpu::EdgeTpuManager::DeviceEnumerationRecord>();
                }
            }

        private:
            /////////////////////////////////////////
            // Declare private methods.
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////
            edgetpu::EdgeTpuManager::DeviceEnumerationRecord m_tpuDevice;
            edgetpu::EdgeTpuManager::DeviceOptions m_tpuDeviceOptions;
            std::unique_ptr<tflite::FlatBufferModel> m_pTFLiteModel;
            std::shared_ptr<edgetpu::EdgeTpuContext> m_pEdgeTPUContext;
            std::unique_ptr<tflite::Interpreter> m_tfModelInterpreter;
    };
}    // namespace tensorflowmodel

#endif
