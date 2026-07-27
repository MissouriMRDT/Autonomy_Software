/******************************************************************************
 * @brief Implements the BasicCam class.
 *
 * @file BasicCam.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-19
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "BasicCam.h"
#include "../../AutonomyConstants.h"
#include "../../AutonomyLogging.h"

/******************************************************************************
 * @brief Construct a new Basic Cam:: Basic Cam object.
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
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
BasicCam::BasicCam(const std::string szCameraPath,
                   const int nPropResolutionX,
                   const int nPropResolutionY,
                   const int nPropFramesPerSecond,
                   const PIXEL_FORMATS ePropPixelFormat,
                   const double dPropHorizontalFOV,
                   const double dPropVerticalFOV,
                   const bool bEnableRecordingFlag,
                   const int nNumFrameRetrievalThreads) :
    BasicCamera(szCameraPath,
                nPropResolutionX,
                nPropResolutionY,
                nPropFramesPerSecond,
                ePropPixelFormat,
                dPropHorizontalFOV,
                dPropVerticalFOV,
                bEnableRecordingFlag,
                nNumFrameRetrievalThreads)
{
    // Initialize the OpenCV mat to a black/empty image the size of the camera resolution.
    m_cvFrame = cv::Mat::zeros(nPropResolutionY, nPropResolutionX, CV_8UC4);

    // Set video cap properties.
    m_cvCamera.set(cv::CAP_PROP_FRAME_WIDTH, nPropResolutionX);
    m_cvCamera.set(cv::CAP_PROP_FRAME_HEIGHT, nPropResolutionY);
    m_cvCamera.set(cv::CAP_PROP_FPS, nPropFramesPerSecond);

    // Initialize other member variables.

    // Attempt to open camera with OpenCV's VideoCapture and print if successfully opened or not.
    if (m_cvCamera.open(szCameraPath))
    {
        // Submit logger message.
        LOG_INFO(logging::g_qSharedLogger, "Camera {} at path/URL {} has been successfully opened.", m_cvCamera.getBackendName(), szCameraPath);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "Unable to open camera at path/URL {}", szCameraPath);
    }

    // Publish the initial open status. This is the only place the VideoCapture is queried
    // off the owning thread, and it happens before that thread starts, so it is safe.
    m_abCameraOpen.store(m_cvCamera.isOpened(), std::memory_order_release);

    // Set max FPS of the ThreadedContinuousCode method.
    this->SetMainThreadIPSLimit(nPropFramesPerSecond);
}

/******************************************************************************
 * @brief Construct a new Basic Cam:: Basic Cam object. Overloaded for dev/video
 *      indexes.
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
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
BasicCam::BasicCam(const int nCameraIndex,
                   const int nPropResolutionX,
                   const int nPropResolutionY,
                   const int nPropFramesPerSecond,
                   const PIXEL_FORMATS ePropPixelFormat,
                   const double dPropHorizontalFOV,
                   const double dPropVerticalFOV,
                   const bool bEnableRecordingFlag,
                   const int nNumFrameRetrievalThreads) :
    BasicCamera(nCameraIndex,
                nPropResolutionX,
                nPropResolutionY,
                nPropFramesPerSecond,
                ePropPixelFormat,
                dPropHorizontalFOV,
                dPropVerticalFOV,
                bEnableRecordingFlag,
                nNumFrameRetrievalThreads)
{
    // Initialize the OpenCV mat to a black/empty image the size of the camera resolution.
    m_cvFrame = cv::Mat::zeros(nPropResolutionY, nPropResolutionX, CV_8UC4);

    // Set video cap properties.
    m_cvCamera.set(cv::CAP_PROP_FRAME_WIDTH, nPropResolutionX);
    m_cvCamera.set(cv::CAP_PROP_FRAME_HEIGHT, nPropResolutionY);
    m_cvCamera.set(cv::CAP_PROP_FPS, nPropFramesPerSecond);

    // Initialize other member variables.

    // Attempt to open camera with OpenCV's VideoCapture.
    m_cvCamera.open(m_nCameraIndex);
    // Check if the camera was successfully opened.
    if (m_cvCamera.isOpened())
    {
        // Submit logger message.
        LOG_INFO(logging::g_qSharedLogger, "Camera {} at video index {} has been successfully opened.", m_cvCamera.getBackendName(), m_nCameraIndex);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "Unable to open camera at video index {}", m_nCameraIndex);
    }

    // Publish the initial open status. This is the only place the VideoCapture is queried
    // off the owning thread, and it happens before that thread starts, so it is safe.
    m_abCameraOpen.store(m_cvCamera.isOpened(), std::memory_order_release);

    // Set max FPS of the ThreadedContinuousCode method.
    this->SetMainThreadIPSLimit(nPropFramesPerSecond);
}

/******************************************************************************
 * @brief Destroy the Basic Cam:: Basic Cam object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
BasicCam::~BasicCam()
{
    // Stop threaded code.
    this->RequestStop();
    this->Join();

    // Release camera capture object.
    m_cvCamera.release();

    // Check if camera was connected on a video index.
    if (m_bCameraIsConnectedOnVideoIndex)
    {
        // Submit logger message.
        LOG_INFO(logging::g_qSharedLogger, "Basic camera at video index {} has been successfully closed.", m_nCameraIndex);
    }
    else
    {
        // Submit logger message.
        LOG_INFO(logging::g_qSharedLogger, "Basic camera at path/URL {} has been successfully closed.", m_szCameraPath);
    }
}

/******************************************************************************
 * @brief The code inside this private method runs in a separate thread, but still
 *      has access to this*. This method continuously gets new frames from the OpenCV
 *      VideoCapture object and publishes a deep-copied snapshot of each one, but only while
 *      at least one consumer is subscribed. Consumers read the newest snapshot on their own
 *      schedule, so this loop never waits on them.
 *
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-16
 ******************************************************************************/
void BasicCam::ThreadedContinuousCode()
{
    // Check if camera is NOT open. isOpened() only ever runs on this owning thread.
    if (!m_cvCamera.isOpened())
    {
        // Publish that the camera is not currently open so foreign readers see it lock free.
        m_abCameraOpen.store(false, std::memory_order_release);

        // Log the open -> closed transition exactly once instead of every iteration.
        if (m_bLastKnownOpenState)
        {
            // Remember the new state so we do not log again until it changes back.
            m_bLastKnownOpenState = false;
            // Submit logger message.
            LOG_CRITICAL(logging::g_qSharedLogger,
                         "BasicCam at {}/{} is not open. Retrying every {} ms until it connects; this thread will keep running.",
                         m_nCameraIndex,
                         m_szCameraPath,
                         constants::CAMERA_RECONNECT_RETRY_INTERVAL.count());
        }

        // Rate limit reopen attempts on a monotonic deadline. This thread NEVER stops itself for a
        // missing camera: it idles and keeps retrying, so a camera that is absent at startup or
        // unplugged at runtime is recovered automatically and startup ordering never matters.
        if (m_tmReconnectTimer.Ready())
        {
            // Whether this attempt managed to reopen the capture device.
            bool bCameraReopened = false;

            // Check if camera was opened with an index or path.
            if (m_nCameraIndex == -1)
            {
                // Attempt to reopen camera.
                bCameraReopened = m_cvCamera.open(m_szCameraPath);
            }
            else
            {
                // Attempt to reopen camera.
                bCameraReopened = m_cvCamera.open(m_nCameraIndex);
            }

            // Check if camera was reopened.
            if (bCameraReopened)
            {
                // Publish the reopened status.
                m_abCameraOpen.store(true, std::memory_order_release);
                // Record the closed -> open transition so the recovery is logged once.
                m_bLastKnownOpenState = true;
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "Camera {}/{} has been reconnected and reopened!", m_nCameraIndex, m_szCameraPath);
            }
            else
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Attempt to reopen Camera {}/{} has failed! Trying again in {} ms...",
                            m_nCameraIndex,
                            m_szCameraPath,
                            constants::CAMERA_RECONNECT_RETRY_INTERVAL.count());
            }
        }
    }
    else
    {
        // Publish that the camera is open.
        m_abCameraOpen.store(true, std::memory_order_release);

        // Only grab and publish a frame when a consumer actually wants one. With no live
        // subscribers there is nothing to produce, so we skip the read entirely.
        if (m_pubFrame.HasSubscribers())
        {
            // Check if new frame was read successfully into the producer-local scratch buffer.
            if (m_cvCamera.read(m_cvFrame))
            {
                // Resize the frame to the configured resolution.
                cv::resize(m_cvFrame, m_cvFrame, cv::Size(m_nPropResolutionX, m_nPropResolutionY), 0.0, 0.0, constants::BASICCAM_RESIZE_INTERPOLATION_METHOD);

                // Acquire a pooled snapshot slot and DEEP COPY the frame into it. We must never
                // publish a Mat that aliases the VideoCapture buffer; copyTo reuses the slot's
                // existing allocation when the geometry matches, so steady state does not allocate.
                std::shared_ptr<pubsub::Snapshot<cv::Mat>> pSlot = m_pubFrame.Acquire();
                m_cvFrame.copyTo(pSlot->tData);
                // Publish the immutable snapshot as the newest frame.
                m_pubFrame.Publish(std::move(pSlot));
            }
            else
            {
                // Submit logger message.
                LOG_ERROR(logging::g_qSharedLogger, "Unable to read new frame for camera {}, {}! Closing camera...", m_nCameraIndex, m_szCameraPath);
                // Release camera capture.
                m_cvCamera.release();
                // Publish the closed status. Publish NO frame; the last good snapshot stays valid
                // and consumers can detect it is stale via its sequence number and publish time.
                m_abCameraOpen.store(false, std::memory_order_release);
            }
        }
    }
}

/******************************************************************************
 * @brief Not used. Frame distribution is handled by the publish-latest mechanism
 *      in ThreadedContinuousCode(); no per-consumer fan-out work remains, so this
 *      override is an intentional no-op required only because AutonomyThread
 *      declares it pure virtual.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void BasicCam::PooledLinearCode() {}

/******************************************************************************
 * @brief Accessor for the camera open status.
 *
 * @return true - The camera has been successfully opened.
 * @return false - The camera has not been successfully opened.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
bool BasicCam::GetCameraIsOpen()
{
    // Read the lock-free published open status. The VideoCapture itself is only ever touched
    // on the owning thread, so foreign callers never race it here.
    return this->GetThreadState() == AutonomyThreadState::eRunning && m_abCameraOpen.load(std::memory_order_acquire);
}

/******************************************************************************
 * @brief Accessor for the cameras path or video index.
 *
 * @return std::string - The path or index of the camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
std::string BasicCam::GetCameraLocation() const
{
    // Check if camera location is a hardware path or video index.
    if (m_bCameraIsConnectedOnVideoIndex)
    {
        // If video index, return index integer.
        return std::to_string(m_nCameraIndex);
    }
    else
    {
        // If video path, return path string.
        return m_szCameraPath;
    }
}
