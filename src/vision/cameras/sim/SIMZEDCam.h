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

/// \cond
#include <opencv2/opencv.hpp>
#include <rtc/rtc.hpp>

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
        // Declare private member variables.
        /////////////////////////////////////////
        // Basic Camera specific.

        std::string m_szCameraPath;
        int m_nNumFrameRetrievalThreads;

        // Mats for storing frames.
        cv::Mat m_cvFrame;
        cv::Mat m_cvDepthImage;
        cv::Mat m_cvDepthMeasure;
        cv::Mat m_cvPointCloud;

        // WebRTC connection to RoveSoSimulator Pixel Streamer.
        std::shared_ptr<rtc::WebSocket> m_pWebSocket;
        std::shared_ptr<rtc::PeerConnection> m_pPeerConnection;
        std::shared_ptr<rtc::DataChannel> m_pDataChannel;
        std::shared_ptr<rtc::Track> rtcVideoTrack1;
        std::shared_ptr<rtc::H264RtpDepacketizer> rtcH264DepacketizationHandler;
        std::shared_ptr<rtc::RtcpReceivingSession> rtcRTCPReceivingSession;
        std::shared_ptr<rtc::RtpDepacketizer> rtcRTPDepacketizer;
        std::shared_ptr<rtc::RtcpSrReporter> rtcRTCPSrReporter;
        std::shared_ptr<rtc::RtcpNackResponder> rtcRTCPNackResponder;
        std::shared_ptr<rtc::Track> rtcVideoTrack2;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////
        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
        bool ConnectToSignallingServer(const std::string& szSignallingServerURL);
};
#endif
