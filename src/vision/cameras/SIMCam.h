/******************************************************************************
 * @brief Defines the SIMCam class.
 *
 * @file SIMCam.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef SIMCAM_H
#define SIMCAM_H

#include <opencv2/opencv.hpp>

#include "../../interfaces/AutonomyThread.hpp"
#include "../../interfaces/Camera.hpp"

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
class SIMCam : public Camera<cv::Mat>, public AutonomyThread<void>
{
    public:
        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////
        SIMCam(const std::string szCameraPath,
               const int nPropResolutionX,
               const int nPropResolutionY,
               const int nPropFramesPerSecond,
               const PIXEL_FORMATS ePropPixelFormat,
               const double dPropHorizontalFOV,
               const double dPropVerticalFOV,
               const bool bEnableRecordingFlag,
               const int nNumFrameRetrievalThreads = 10);
        SIMCam(const int nCameraIndex,
               const int nPropResolutionX,
               const int nPropResolutionY,
               const int nPropFramesPerSecond,
               const PIXEL_FORMATS ePropPixelFormat,
               const double dPropHorizontalFOV,
               const double dPropVerticalFOV,
               const bool bEnableRecordingFlag,
               const int nNumFrameRetrievalThreads = 10);
        ~SIMCam();
        std::future<bool> RequestFrameCopy(cv::Mat& cvFrame) override;
        std::future<bool> RequestDepthCopy(cv::Mat& cvDepth, const bool bRetrieveMeasure = true);
        std::future<bool> RequestPointCloudCopy(cv::Mat& cvPointCloud);

        /////////////////////////////////////////
        // Getters.
        /////////////////////////////////////////
        template<typename T>
        T GetCameraLocation() const;
        bool GetCameraIsOpen() override;

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        // Basic Camera specific.

        cv::VideoCapture m_cvCamera;
        std::string m_szCameraPath;
        bool m_bCameraIsConnectedOnVideoIndex;
        int m_nCameraIndex;
        int m_nNumFrameRetrievalThreads;

        // Mats for storing frames.

        cv::Mat m_cvFrame;
        cv::Mat m_cvDepthImage;
        cv::Mat m_cvDepthMeasure;
        cv::Mat m_cvPointCloud;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////
        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
};
#endif
