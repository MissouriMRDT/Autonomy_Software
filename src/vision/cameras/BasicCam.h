/******************************************************************************
 * @brief Defines the BasicCam class.
 *
 * @file BasicCam.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef BASICCAM_H
#define BASICCAM_H

#include <opencv2/opencv.hpp>

#include "../../interfaces/Camera.hpp"

class BasicCam : public Camera<cv::Mat>
{
    private:
        // Declare private methods and functions variables.

        bool m_bCameraIsConnectedOnVideoIndex;
        int m_nCameraIndex;
        std::string m_szCameraPath;
        cv::VideoCapture m_cvCamera;
        cv::Mat m_cvFrame;

    public:
        // Declare public methods and member variables.

        BasicCam(const std::string szCameraPath,
                 const int nPropResolutionX,
                 const int nPropResolutionY,
                 const int nPropFramesPerSecond,
                 const PIXEL_FORMATS ePropPixelFormat,
                 const double dPropHorizontalFOV,
                 const double dPropVerticalFOV);
        BasicCam(const int nCameraIndex,
                 const int nPropResolutionX,
                 const int nPropResolutionY,
                 const int nPropFramesPerSecond,
                 const PIXEL_FORMATS ePropPixelFormat,
                 const double dPropHorizontalFOV,
                 const double dPropVerticalFOV);
        ~BasicCam();
        cv::Mat GrabFrame(const bool bGrabRaw = false) override;

        // Getters.
        template<typename T>
        T GetCameraLocation() const;
        bool GetCameraIsOpen() override;
};
#endif
