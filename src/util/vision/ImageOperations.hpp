/******************************************************************************
 * @brief Defines and implements functions related to operations on images. All
 *      functions are defined within the imgops namespace.
 *
 * @file ImageOperations.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-31
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef IMAGE_OPERATIONS_HPP
#define IMAGE_OPERATIONS_HPP

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

/******************************************************************************
 * @brief Namespace containing functions related to operations on images or other
 *      large binary operations.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-31
 ******************************************************************************/
namespace imgops
{
    /******************************************************************************
     * @brief Provides an easy method of mapping sl::Mat types to cv::Mat types.
     *
     * @param slType - The data type/layout of the sl mat object.
     * @return int - The integer representing the cv mat type that maps to the given sl mat type.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-08-31
     ******************************************************************************/
    int GetSLToOpenCVMatType(const sl::MAT_TYPE slType)
    {
        // Create instance variables.
        int nOpenCVType = -1;

        // Map sl to cv mat type.
        switch (slType)
        {
            case sl::MAT_TYPE::F32_C1: nOpenCVType = CV_32FC1; break;
            case sl::MAT_TYPE::F32_C2: nOpenCVType = CV_32FC2; break;
            case sl::MAT_TYPE::F32_C3: nOpenCVType = CV_32FC3; break;
            case sl::MAT_TYPE::F32_C4: nOpenCVType = CV_32FC4; break;
            case sl::MAT_TYPE::U8_C1: nOpenCVType = CV_8UC1; break;
            case sl::MAT_TYPE::U8_C2: nOpenCVType = CV_8UC2; break;
            case sl::MAT_TYPE::U8_C3: nOpenCVType = CV_8UC3; break;
            case sl::MAT_TYPE::U8_C4: nOpenCVType = CV_8UC4; break;
            default: break;
        }

        // Return new type.
        return nOpenCVType;
    }

    /******************************************************************************
     * @brief Provides an easy method of mapping cv::Mat types to sl::Mat types.
     *
     * @param slType - The data type/layout of the cv mat object.
     * @return int - The integer representing the sl mat type that maps to the given cv mat type.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-08-31
     ******************************************************************************/
    sl::MAT_TYPE GetCVToOpenSLMatType(const int cvType)
    {
        // Create instance variables.
        sl::MAT_TYPE slMatType = sl::MAT_TYPE::U8_C1;

        // Map sl to cv mat type.
        switch (cvType)
        {
            case CV_32FC1: slMatType = sl::MAT_TYPE::F32_C1; break;
            case CV_32FC2: slMatType = sl::MAT_TYPE::F32_C2; break;
            case CV_32FC3: slMatType = sl::MAT_TYPE::F32_C3; break;
            case CV_32FC4: slMatType = sl::MAT_TYPE::F32_C4; break;
            case CV_8UC1: slMatType = sl::MAT_TYPE::U8_C1; break;
            case CV_8UC2: slMatType = sl::MAT_TYPE::U8_C2; break;
            case CV_8UC3: slMatType = sl::MAT_TYPE::U8_C3; break;
            case CV_8UC4: slMatType = sl::MAT_TYPE::U8_C4; break;
        }

        // Return new type.
        return slMatType;
    }

    /******************************************************************************
     * @brief Convert a ZEDSDK sl::Mat object into an OpenCV cv::Mat object. This copies
     *      the mat from GPU memory to CPU memory.
     *
     * @param input - The ZEDSDK Mat to be converted.
     * @return cv::Mat - The resultant OpenCV mat object.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-08-31
     ******************************************************************************/
    cv::Mat ConvertSLMatToCVMat(sl::Mat& slInputMat)
    {
        // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
        // cv::Mat and sl::Mat will share a single memory structure.
        return cv::Mat(slInputMat.getHeight(),
                       slInputMat.getWidth(),
                       GetSLToOpenCVMatType(slInputMat.getDataType()),
                       slInputMat.getPtr<sl::uchar1>(sl::MEM::CPU),
                       slInputMat.getStepBytes(sl::MEM::CPU));
    }

    /******************************************************************************
     * @brief Convert a ZEDSDK sl::Mat object into an OpenCV cv::cuda::GpuMat object.
     *      Keeps all Mat memory in GPU VRAM for faster processing.
     *
     * @param slInputMat -
     * @return cv::cuda::GpuMat -
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-08-31
     ******************************************************************************/
    cv::cuda::GpuMat ConvertSLMatToGPUMat(sl::Mat& slInputMat)
    {
        // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
        // cv::Mat and sl::Mat will share a single memory structure
        return cv::cuda::GpuMat(slInputMat.getHeight(),
                                slInputMat.getWidth(),
                                GetSLToOpenCVMatType(slInputMat.getDataType()),
                                slInputMat.getPtr<sl::uchar1>(sl::MEM::GPU),
                                slInputMat.getStepBytes(sl::MEM::GPU));
    }
}    // namespace imgops
#endif
