/******************************************************************************
 * @brief Defines and implements the BasicCamera interface class.
 *
 * @file BasicCamera.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-22
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef BASICCAMERA_HPP
#define BASICCAMERA_HPP

// AutonomyConstants.h is needed for the publisher pool sizing constants used below. It is safe
// to include here (it includes Camera.hpp, not this header, so there is no cycle) and without it
// this header only compiles when some other header happens to have pulled constants in first.
#include "../AutonomyConstants.h"
#include "../util/threading/Publisher.hpp"
#include "Camera.hpp"

/// \cond

/// \endcond

/******************************************************************************
 * @brief This class serves as a middle inheritor between the Camera interface
 *      and the BasicCam class. BasicCam and SIMBasicCam will inherit from this.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-22
 ******************************************************************************/
class BasicCamera : public Camera<cv::Mat>
{
    public:
        /******************************************************************************
         * @brief Construct a new Basic Camera object.
         *
         * @param szCameraPath - The file path to the camera hardware.
         * @param nPropResolutionX - X res of camera.
         * @param nPropResolutionY - Y res of camera.
         * @param nPropFramesPerSecond - FPS camera is running at.
         * @param ePropPixelFormat - The pixel layout/format of the image.
         * @param dPropHorizontalFOV - The horizontal field of view.
         * @param dPropVerticalFOV - The vertical field of view.
         * @param bEnableRecordingFlag - Whether or not this camera should be recorded.
         * @param nNumFrameRetrievalThreads - The number of threads to use for frame queueing and copying.
         ******************************************************************************/
        BasicCamera(const std::string szCameraPath,
                    const int nPropResolutionX,
                    const int nPropResolutionY,
                    const int nPropFramesPerSecond,
                    const PIXEL_FORMATS ePropPixelFormat,
                    const double dPropHorizontalFOV,
                    const double dPropVerticalFOV,
                    const bool bEnableRecordingFlag,
                    const int nNumFrameRetrievalThreads) :
            Camera(nPropResolutionX,
                   nPropResolutionY,
                   nPropFramesPerSecond,
                   ePropPixelFormat,
                   dPropHorizontalFOV,
                   dPropVerticalFOV,
                   bEnableRecordingFlag,
                   nNumFrameRetrievalThreads)
        {
            // Initialize member variables.
            m_nCameraIndex                   = -1;
            m_szCameraPath                   = szCameraPath;
            m_bCameraIsConnectedOnVideoIndex = false;
        }

        /******************************************************************************
         * @brief Construct a new Basic Camera object. Overloaded for dev/video indexes.
         *
         * @param nCameraIndex - The video index that the camera is connected on.
         * @param nPropResolutionX - X res of camera.
         * @param nPropResolutionY - Y res of camera.
         * @param nPropFramesPerSecond - FPS camera is running at.
         * @param ePropPixelFormat - The pixel layout/format of the image.
         * @param dPropHorizontalFOV - The horizontal field of view.
         * @param dPropVerticalFOV - The vertical field of view.
         * @param bEnableRecordingFlag - Whether or not this camera should be recorded.
         * @param nNumFrameRetrievalThreads - The number of threads to use for frame queueing and copying.
         ******************************************************************************/
        BasicCamera(const int nCameraIndex,
                    const int nPropResolutionX,
                    const int nPropResolutionY,
                    const int nPropFramesPerSecond,
                    const PIXEL_FORMATS ePropPixelFormat,
                    const double dPropHorizontalFOV,
                    const double dPropVerticalFOV,
                    const bool bEnableRecordingFlag,
                    const int nNumFrameRetrievalThreads) :
            Camera(nPropResolutionX,
                   nPropResolutionY,
                   nPropFramesPerSecond,
                   ePropPixelFormat,
                   dPropHorizontalFOV,
                   dPropVerticalFOV,
                   bEnableRecordingFlag,
                   nNumFrameRetrievalThreads)
        {
            // Initialize member variables.
            m_nCameraIndex                   = nCameraIndex;
            m_szCameraPath                   = "";
            m_bCameraIsConnectedOnVideoIndex = true;
        }

        /******************************************************************************
         * @brief Destroy the Basic Camera object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        virtual ~BasicCamera() {}

        /******************************************************************************
         * @brief The code inside this private method runs in a separate thread, but still
         *      has access to this*. This method continuously gets new frames from the OpenCV
         *      VideoCapture object and publishes a deep-copied snapshot of each one, but only
         *      while at least one Reader is alive. Consumers read the newest snapshot on
         *      their own schedule, so this loop never waits on them.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        virtual void ThreadedContinuousCode() {}

        /******************************************************************************
         * @brief Not used. Frame distribution is handled by the publish-latest mechanism
         *      in ThreadedContinuousCode(); no per-consumer fan-out work remains.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        virtual void PooledLinearCode() {}

        /******************************************************************************
         * @brief Accessor for the cameras path or video index.
         *
         * @return std::string - The path or index of the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        virtual std::string GetCameraLocation() const { return ""; }

        /******************************************************************************
         * @brief Accessor for this camera's publish-latest frame channel.
         *
         *      Returns a Reader: holding it expresses demand (the producer only reads
         *      and publishes frames while at least one Reader is alive) and Get()s the
         *      newest immutable frame snapshot with a non-blocking read. Because the
         *      Reader is the only thing handed out, a consumer can neither publish into
         *      this channel nor read it without registering demand.
         *      Both the real BasicCam and the simulated SIMBasicCam publish through
         *      this same channel, so consumers stay drop-in interchangeable.
         *
         * @return pubsub::Reader<cv::Mat> - A demand-carrying read handle for the frame channel.
         *
         * @note Load a snapshot once into a local and work from that local; calling
         *      Get() repeatedly returns whatever is newest each time. To modify a
         *      snapshot, clone it on your own thread first - published snapshots are
         *      immutable and shared.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        pubsub::Reader<cv::Mat> GetFrameReader() { return m_pubFrame.CreateReader(); }

    protected:
        // Declare protected methods and member variables.
        int m_nCameraIndex;
        std::string m_szCameraPath;
        bool m_bCameraIsConnectedOnVideoIndex;

        // Publish-latest channel for the camera's BGRA frame. Producers copy each new
        // frame into a pooled snapshot and publish it; consumers read the newest
        // snapshot without blocking the producer. Replaces the old frame request queue.
        // The explicit preallocation and growth ceiling keep steady state allocation free
        // and surface a snapshot-leaking consumer as a logged error rather than an OOM.
        pubsub::Publisher<cv::Mat> m_pubFrame{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};

    private:
        // Declare private methods and member variables.
};

#endif    // BASIC_CAMERA_HPP
