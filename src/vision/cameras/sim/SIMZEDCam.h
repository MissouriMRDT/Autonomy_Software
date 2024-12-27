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
#include "../../../interfaces/ZEDCamera.hpp"
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
class SIMZEDCam : public ZEDCamera
{
    public:
        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////

        SIMZEDCam(const std::string szCameraPath,
                  const int nPropResolutionX,
                  const int nPropResolutionY,
                  const int nPropFramesPerSecond,
                  const double dPropHorizontalFOV,
                  const double dPropVerticalFOV,
                  const bool bEnableRecordingFlag,
                  const int nNumFrameRetrievalThreads     = 10,
                  const unsigned int unCameraSerialNumber = 0);
        ~SIMZEDCam();
        std::future<bool> RequestFrameCopy(cv::Mat& cvFrame) override;
        std::future<bool> RequestDepthCopy(cv::Mat& cvDepth, const bool bRetrieveMeasure = true);
        std::future<bool> RequestPointCloudCopy(cv::Mat& cvPointCloud);
        sl::ERROR_CODE ResetPositionalTracking() override;
        sl::ERROR_CODE RebootCamera() override;
        sl::FUSION_ERROR_CODE SubscribeFusionToCameraUUID(sl::CameraIdentifier& slCameraUUID) override;
        sl::CameraIdentifier PublishCameraToFusion() override;

        /////////////////////////////////////////
        // Setters for class member variables.
        /////////////////////////////////////////

        sl::ERROR_CODE EnablePositionalTracking(const float fExpectedCameraHeightFromFloorTolerance = constants::ZED_DEFAULT_FLOOR_PLANE_ERROR) override;
        void DisablePositionalTracking() override;
        void SetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO) override;

        /////////////////////////////////////////
        // Getters.
        /////////////////////////////////////////

        bool GetCameraIsOpen() override;
        bool GetUsingGPUMem() const override;
        std::string GetCameraModel() override;
        std::future<bool> RequestPositionalPoseCopy(Pose& stPose) override;
        std::future<bool> RequestFusionGeoPoseCopy(sl::GeoPose& slGeoPose) override;
        bool GetPositionalTrackingEnabled() override;

    private:
        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;

        void SetCallbacks();

        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        // ZED Camera specific.

        std::string m_szCameraPath;
        std::atomic<bool> m_bCameraPositionalTrackingEnabled;

        // WebRTC connections for each camera stream from the RoveSoSimulator.

        std::unique_ptr<WebRTC> m_pRGBStream;
        std::unique_ptr<WebRTC> m_pDepthImageStream;
        std::unique_ptr<WebRTC> m_pDepthMeasureStream;
        std::unique_ptr<WebRTC> m_pPointCloudStream;

        // Pose tracking offsets. (ZEDSDK is broken and can't handle large translations internally as it uses float32.)

        double m_dPoseOffsetX;
        double m_dPoseOffsetY;
        double m_dPoseOffsetZ;
        double m_dPoseOffsetXO;
        double m_dPoseOffsetYO;
        double m_dPoseOffsetZO;

        // Data from NavBoard.

        geoops::RoverPose m_stCurrentRoverPose;
        std::shared_mutex m_muCurrentRoverPoseMutex;

        // Mats for storing frames.

        cv::Mat m_cvFrame;
        cv::Mat m_cvDepthImage;
        cv::Mat m_cvDepthBuffer;
        cv::Mat m_cvDepthMeasure;
        cv::Mat m_cvPointCloud;

        std::queue<containers::DataFetchContainer<Pose>> m_qPoseCopySchedule;
        std::queue<containers::DataFetchContainer<sl::GeoPose>> m_qGeoPoseCopySchedule;

        // Mutexes for copying frames from the WebRTC connection to the OpenCV Mats.

        std::shared_mutex m_muWebRTCRGBImageCopyMutex;
        std::shared_mutex m_muWebRTCDepthImageCopyMutex;
        std::shared_mutex m_muWebRTCDepthMeasureCopyMutex;
        std::shared_mutex m_muWebRTCPointCloudCopyMutex;

        // Mutexes for copying frames from the ZEDSDK to the OpenCV Mats in PoolLinearCode.
        std::shared_mutex m_muPoseCopyMutex;
        std::shared_mutex m_muGeoPoseCopyMutex;

        // Atomic flags for checking if data is queued.

        std::atomic<bool> m_bPosesQueued;
        std::atomic<bool> m_bGeoPosesQueued;
};
#endif
