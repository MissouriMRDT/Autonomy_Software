/******************************************************************************
 * @brief Defines and implements objects/structs related to opening, interfacing,
 *      and inferencing YOLO models. All contained within the TensorflowModel
 *      namespace.
 *
 * @file YOLOModel.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-11-11
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef YOLO_MODEL_HPP
#define YOLO_MODEL_HPP

#include "../../interfaces/TensorflowTPU.hpp"

#include <opencv2/opencv.hpp>

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
    struct Detection
    {};

    /******************************************************************************
     * @brief Namespace containing functions or objects/structs used to run inference on
     *      a YOLO model with the Tensorflow library. This namespace contains static functions
     *      for getting available hardware devices, and classes for running a .tflite model on each device.
     *      This namespace was built to work with YOLO models only!
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-11-13
     ******************************************************************************/
    namespace tensorflow
    {

        /******************************************************************************
         * @brief This struct is used to store the dimensions of a tensor.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-11-12
         ******************************************************************************/
        struct TensorDimensions
        {
            public:
                // Define public struct attributes.
                int nHeight;
                int nWidth;
                int nChannels;
        };

        /******************************************************************************
         * @brief This class is designed to enable quick, easy, and robust inferencing of .tflite
         *      yolo model.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-10-24
         ******************************************************************************/
        class TPUInterpreter : public TensorflowTPU<std::vector<Detection>, cv::Mat>
        {
            public:
                /////////////////////////////////////////
                // Declare public enums that are specific to and used withing this class.
                /////////////////////////////////////////

                /////////////////////////////////////////
                // Declare public methods and member variables.
                /////////////////////////////////////////

                /******************************************************************************
                 * @brief Construct a new TPUInterpreter object.
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
                TPUInterpreter(std::string szModelPath, PerformanceModes ePowerMode = eHigh, unsigned int unMaxBulkInQueueLength = 32, bool bUSBAlwaysDFU = false) :
                    TensorflowTPU<std::vector<Detection>, cv::Mat>(szModelPath, ePowerMode, unMaxBulkInQueueLength, bUSBAlwaysDFU)

                {}

                /******************************************************************************
                 * @brief Destroy the TPUInterpreter object.
                 *
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2023-10-24
                 ******************************************************************************/
                ~TPUInterpreter()
                {
                    // Nothing to destroy.
                }

                /******************************************************************************
                 * @brief Given an input image forward the image through the YOLO model to run inference
                 *      on the EdgeTPU, then parse and repackage the output tensor data into a vector
                 *      of easy-to-use Detection structs.
                 *
                 *      YOLOv5 predicts 25200 grid_cells when fed with a (3, 640, 640) image
                 *     (Three detection layers for small, medium, and large objects same size as input with same bit depth).
                 *      Each grid_cell is a vector composed by (5 + num_classes) values where the 5 values are [object_score, Xc, Yc, W, H].
                 *      Output would be [1, 25200, 13] for a model with eight classes and 640x640 input size.
                 *
                 *      Check out https://pub.towardsai.net/yolov5-m-implementation-from-scratch-with-pytorch-c8f84a66c98b
                 *      for some great info.
                 *
                 * @param cvInputFrame - The RGB camera frame to run detection on.
                 * @return std::vector<Detection> -
                 *
                 * @note The input image MUST BE RGB format, otherwise you will likely experience prediction accuracy problems.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2023-11-13
                 ******************************************************************************/
                std::vector<Detection> Inference(cv::Mat& cvInputFrame) override {}

            private:
                /////////////////////////////////////////
                // Declare private methods.
                /////////////////////////////////////////

                /******************************************************************************
                 * @brief Get the input shape of the tensor at the given index. Requires the device
                 *      to have been successfully opened.
                 *
                 * @param nInputIndex - The index of the input tensor to use. YOLO models that
                 *          have been converted to a edgetpu quantized .tflite file will only have one
                 *          input at index 0.
                 * @return TensorDimensions - A struct containing the height, width, and channels of the input tensor.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2023-11-12
                 ******************************************************************************/
                TensorDimensions GetInputShape(const int nInputIndex = 0)
                {
                    // Create instance variables.
                    TensorDimensions stInputDimensions = {0, 0, 0};

                    // Check if interpreter has been built.
                    if (m_bDeviceOpened)
                    {
                        // Get the desired input tensor shape of the model.
                        int nTensorIndex             = m_pInterpreter->inputs()[nInputIndex];
                        TfLiteIntArray* tfDimensions = m_pInterpreter->tensor(nTensorIndex)->dims;

                        // Package dimensions into struct.
                        stInputDimensions.nHeight   = tfDimensions->data[1];
                        stInputDimensions.nWidth    = tfDimensions->data[2];
                        stInputDimensions.nChannels = tfDimensions->data[3];
                    }

                    return stInputDimensions;
                }

                /******************************************************************************
                 * @brief Get the output shape of the tensor at the given index. Requires the device
                 *      to have been successfully opened.
                 *
                 * @param nOutputIndex - The index of the input tensor to use. YOLO models that
                 *          have been converted to a edgetpu quantized .tflite file will only have one
                 *          output at index 0.
                 * @return TensorDimensions - A struct containing the height, width, and channels of the output tensor.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2023-11-12
                 ******************************************************************************/
                TensorDimensions GetOutputShape(const int nOutputIndex = 0)
                {
                    // Create instance variables.
                    TensorDimensions stOutputDimensions = {0, 0, 0};

                    // Check if interpreter has been built.
                    if (m_bDeviceOpened)
                    {
                        // Get the desired output tensor shape of the model.
                        int nTensorIndex             = m_pInterpreter->inputs()[nOutputIndex];
                        TfLiteIntArray* tfDimensions = m_pInterpreter->tensor(nTensorIndex)->dims;

                        // Package dimensions into struct.
                        stOutputDimensions.nHeight   = tfDimensions->data[1];
                        stOutputDimensions.nWidth    = tfDimensions->data[2];
                        stOutputDimensions.nChannels = tfDimensions->data[3];
                    }

                    return stOutputDimensions;
                }

                /////////////////////////////////////////
                // Declare private member variables.
                /////////////////////////////////////////
        };
    }    // namespace tensorflow
}    // namespace yolomodel

#endif
