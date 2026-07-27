/******************************************************************************
 * @brief Defines the SIMBasicCam class.
 *
 * @file SIMBasicCam.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef SIMBASICCAM_H
#define SIMBASICCAM_H

#include "../../../interfaces/AutonomyThread.hpp"
#include "../../../interfaces/BasicCamera.hpp"
#include "../../../util/threading/RetryTimer.hpp"

/// \cond
#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <tracy/Tracy.hpp>

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
class SIMBasicCam : public BasicCamera
{
    public:
        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////
        SIMBasicCam(const std::string szCameraPath,
                    const int nPropResolutionX,
                    const int nPropResolutionY,
                    const int nPropFramesPerSecond,
                    const PIXEL_FORMATS ePropPixelFormat,
                    const double dPropHorizontalFOV,
                    const double dPropVerticalFOV,
                    const bool bEnableRecordingFlag,
                    const int nNumFrameRetrievalThreads = 10);
        ~SIMBasicCam();

        /////////////////////////////////////////
        // Getters.
        /////////////////////////////////////////

        bool GetCameraIsOpen() override;
        std::string GetCameraLocation() const override;

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        // Basic Camera specific.

        cv::VideoCapture m_cvCamera;

        // Producer-thread-local scratch frame. Only ThreadedContinuousCode() touches it;
        // the deep copy into the pooled snapshot is what other threads see.
        cv::Mat m_cvFrame;

        // Lock-free published camera-open status. Written only on the owning thread, read by
        // GetCameraIsOpen() from any thread, so no foreign thread touches the VideoCapture.
        std::atomic<bool> m_abCameraOpen{false};

        // Reconnect pacing and edge-triggered open/closed logging. The producer thread never
        // stops itself for a missing camera; it idles, retries on this monotonic timer, and logs
        // only when the open state actually changes so an idling thread cannot flood the log.
        threadutils::RetryTimer m_tmReconnectTimer{constants::CAMERA_RECONNECT_RETRY_INTERVAL};
        bool m_bLastKnownOpenState = true;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////
        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
};
#endif
