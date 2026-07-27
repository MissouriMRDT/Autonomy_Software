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
#include "../../../util/threading/RetryTimer.hpp"
#include "WebRTC.h"

/// \cond
#include <RoveComm/RoveComm.h>
#include <RoveComm/RoveCommManifest.h>
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
        sl::ERROR_CODE ResetPositionalTracking() override;
        sl::ERROR_CODE RebootCamera() override;

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
        bool GetPositionalTrackingEnabled() override;

    private:
        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
        void SetCallbacks();
        void EstimateDepthMeasure(const cv::Mat& cvDepthImage, cv::Mat& cvDepthMeasure);
        void CalculatePointCloud(const cv::Mat& cvDepthMeasure, cv::Mat& cvPointCloud);
        void PublishStatus();               // Build and publish the CameraStatus snapshot.
        bool GetStreamsAreConnected() const;    // Single source of truth for "is this camera open".
        void ImplReconnectStreams();            // Rebuild the WebRTC stream objects. Safe on the producer thread.

        // Reconnect pacing and edge-triggered connected/disconnected logging. The producer thread
        // never stops itself when the simulator is unreachable; it idles, retries on this
        // monotonic timer, and logs only when the connection state actually changes.
        threadutils::RetryTimer m_tmReconnectTimer{constants::SIM_STREAM_RECONNECT_RETRY_INTERVAL};
        bool m_bLastKnownOpenState = true;

        // Owning-thread implementations of the foreign-thread control methods, run when the
        // producer drains the command queue.
        sl::ERROR_CODE ImplResetPositionalTracking();
        void ImplSetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO);

        /******************************************************************************
         * @brief Callback function to process incoming IMU data from RoveComm for the SIM ZED Camera.
         *      Normally, this data would come from the physical ZED camera's IMU over USB,
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-11-19
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<double>&, const sockaddr_in&)> ProcessIMUData =
            [this](const rovecomm::RoveCommPacket<double>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stdAddr;

            // Acquire a write lock on the IMU data handoff mutex (this callback runs on a foreign
            // RoveComm thread; the producer thread reads m_stIMUData under the same lock).
            std::unique_lock<std::shared_mutex> lkSensorsProcessLock(m_muIMUDataMutex);
            // Update IMU data.
            m_stIMUData.imu.linear_acceleration.x = static_cast<float>(stPacket.vData[0]);
            m_stIMUData.imu.linear_acceleration.y = static_cast<float>(stPacket.vData[1]);
            m_stIMUData.imu.linear_acceleration.z = static_cast<float>(stPacket.vData[2]);
            m_stIMUData.imu.angular_velocity.x    = static_cast<float>(stPacket.vData[3]);
            m_stIMUData.imu.angular_velocity.y    = static_cast<float>(stPacket.vData[4]);
            m_stIMUData.imu.angular_velocity.z    = static_cast<float>(stPacket.vData[5]);

            // Manually calculate the Gyro pose using the Tait-Bryan angles (ZYX convention) and the quaternion representation.
            // This is because the SIM does not provide orientation data from the IMU, only angular velocity.
            double dQx    = stPacket.vData[6];
            double dQy    = stPacket.vData[7];
            double dQz    = stPacket.vData[8];
            double dQw    = stPacket.vData[9];
            double dRoll  = std::atan2(2.0 * (dQw * dQx + dQy * dQz), 1.0 - 2.0 * (dQx * dQx + dQy * dQy));
            double dPitch = std::asin(2.0 * (dQw * dQy - dQz * dQx));
            double dYaw   = std::atan2(2.0 * (dQw * dQz + dQx * dQy), 1.0 - 2.0 * (dQy * dQy + dQz * dQz));
            // Pack the gyro values into a sl::Transform.
            sl::float3 slEulerAngles(static_cast<float>(dRoll), static_cast<float>(dPitch), static_cast<float>(dYaw));
            sl::Transform slIMUTransform;
            slIMUTransform.setEulerAngles(slEulerAngles);
            m_stIMUData.imu.pose = slIMUTransform;

            // Unlock mutex.
            lkSensorsProcessLock.unlock();

            // Submit logger message.
            LOG_DEBUG(logging::g_qSharedLogger,
                      "Incoming IMU data processed from RoveComm for SIM ZED Camera: (AccelX {}, AccelY {}, AccelZ {}, GyroX {}, GyroY {}, GyroZ {})",
                      stPacket.vData[0],
                      stPacket.vData[1],
                      stPacket.vData[2],
                      stPacket.vData[3],
                      stPacket.vData[4],
                      stPacket.vData[5]);
        };

        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        // ZED Camera specific.

        std::string m_szCameraPath;
        std::atomic<bool> m_bCameraPositionalTrackingEnabled;
        std::string m_szFullStreamName;

        // Simulated IMU Data from the SIM.
        sl::SensorsData m_stIMUData;

        // WebRTC connections for each camera stream from the RoveSoSimulator.

        std::unique_ptr<WebRTC> m_pRGBStream;
        std::unique_ptr<WebRTC> m_pDepthImageStream;

        // Pose tracking offsets. (ZEDSDK is broken and can't handle large translations internally as it uses float32.)

        double m_dPoseOffsetX;
        double m_dPoseOffsetY;
        double m_dPoseOffsetZ;
        double m_dPoseOffsetXO;
        double m_dPoseOffsetYO;
        double m_dPoseOffsetZO;

        // Data from NavBoard. Polled and read only on the owning producer thread (and by the
        // posted SetPositionalPose command, which also runs there), so no mutex is needed.

        geoops::RoverPose m_stCurrentRoverPose;

        // Buffers the WebRTC callbacks write into (on foreign decoder threads) and the producer
        // reads from. The two WebRTC mutexes guard that async, library-driven handoff and must stay.

        cv::Mat m_cvFrame;
        cv::Mat m_cvDepthImageBuffer;
        cv::Mat m_cvDepthImage;
        cv::Mat m_cvDepthMeasure;    // Producer-computed from m_cvDepthImage.
        cv::Mat m_cvPointCloud;      // Producer-computed from m_cvDepthMeasure.

        std::shared_mutex m_muWebRTCRGBImageCopyMutex;      // Guards m_cvFrame (RGB callback <-> producer).
        std::shared_mutex m_muWebRTCDepthImageCopyMutex;    // Guards m_cvDepthImage (depth callback <-> producer).
        std::shared_mutex m_muIMUDataMutex;                 // Guards m_stIMUData (RoveComm IMU callback <-> producer).
};
#endif
