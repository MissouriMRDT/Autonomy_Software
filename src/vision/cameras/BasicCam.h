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
#include <shared_mutex>

#include "../../interfaces/AutonomyThread.hpp"
#include "../../interfaces/Camera.hpp"
#include "../../util/vision/FetchContainers.hpp"

class BasicCam : public Camera<cv::Mat>, public AutonomyThread<void>
{
    public:
        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////
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
        bool GrabFrame(cv::Mat& cvFrame) override;

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

        // Mats for storing frames.
        cv::Mat m_cvFrame;

        // Queues and mutexes for scheduling and copying camera frames and data to other threads.
        std::queue<std::reference_wrapper<containers::FrameFetchContainer<cv::Mat&>>> m_qFrameCopySchedule;
        std::shared_mutex m_muPoolScheduleMutex;
        std::mutex m_muFrameCopyMutex;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////
        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
};
#endif
