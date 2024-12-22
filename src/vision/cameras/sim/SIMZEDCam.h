/******************************************************************************
 * @brief Defines the SIMZEDCam class.
 *
 * @file SIMZEDCam.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef SIMZEDCAM_H
#define SIMZEDCAM_H

#include "../../../interfaces/AutonomyThread.hpp"
#include "../../../interfaces/Camera.hpp"
#include "WebRTC.h"

/// \cond
#include <opencv2/opencv.hpp>

/// \endcond

/******************************************************************************
 * @brief This class implements and interfaces with the SIM cameras and data.
 *  It is designed in such a way that multiple other classes/threads
 *  can safely call any method of an object of this class withing resource corruption
 *  or slowdown of the camera.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
class SIMZEDCam : public Camera<cv::Mat>
{
    public:
        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////
        SIMZEDCam(const std::string szCameraPath,
                  const int nPropResolutionX,
                  const int nPropResolutionY,
                  const int nPropFramesPerSecond,
                  const PIXEL_FORMATS ePropPixelFormat,
                  const double dPropHorizontalFOV,
                  const double dPropVerticalFOV,
                  const bool bEnableRecordingFlag,
                  const int nNumFrameRetrievalThreads = 10);
        SIMZEDCam(const int nCameraIndex,
                  const int nPropResolutionX,
                  const int nPropResolutionY,
                  const int nPropFramesPerSecond,
                  const PIXEL_FORMATS ePropPixelFormat,
                  const double dPropHorizontalFOV,
                  const double dPropVerticalFOV,
                  const bool bEnableRecordingFlag,
                  const int nNumFrameRetrievalThreads = 10);
        ~SIMZEDCam();
        std::future<bool> RequestFrameCopy(cv::Mat& cvFrame) override;
        std::future<bool> RequestDepthCopy(cv::Mat& cvDepth, const bool bRetrieveMeasure = true);
        std::future<bool> RequestPointCloudCopy(cv::Mat& cvPointCloud);

        /////////////////////////////////////////
        // Getters.
        /////////////////////////////////////////

        std::string GetCameraLocation() const;
        bool GetCameraIsOpen() override;

    private:
        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////
        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;

        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        // Basic Camera specific.
        std::string m_szCameraPath;
        int m_nNumFrameRetrievalThreads;

        // Mats for storing frames.
        cv::Mat m_cvFrame;
        cv::Mat m_cvDepthImage;
        cv::Mat m_cvDepthBuffer;
        cv::Mat m_cvDepthMeasure;
        cv::Mat m_cvPointCloud;

        // Mutexes for copying frames from the WebRTC connection to the OpenCV Mats.
        std::shared_mutex m_muWebRTCRGBImageCopyMutex;
        std::shared_mutex m_muWebRTCDepthImageCopyMutex;
        std::shared_mutex m_muWebRTCDepthMeasureCopyMutex;
        std::shared_mutex m_muWebRTCPointCloudCopyMutex;

        // WebRTC connections for each camera stream from the RoveSoSimulator.
        std::unique_ptr<WebRTC> m_pRGBStream;
        std::unique_ptr<WebRTC> m_pDepthImageStream;
        std::unique_ptr<WebRTC> m_pDepthMeasureStream;
        std::unique_ptr<WebRTC> m_pPointCloudStream;
};
#endif
