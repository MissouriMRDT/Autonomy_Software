/******************************************************************************
 * @brief Implements the SIMBasicCam class.
 *
 * @file SIMBasicCam.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "SIMBasicCam.h"

#include "../../../AutonomyConstants.h"
#include "../../../AutonomyLogging.h"

/******************************************************************************
 * @brief Construct a new SIM Cam:: SIM Cam object.
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
 * @date 2023-09-30
 ******************************************************************************/
SIMBasicCam::SIMBasicCam(const std::string szCameraPath,
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
    // Initialize OpenCV mats to a black/empty image the size of the camera resolution.
    m_cvFrame = cv::Mat::zeros(nPropResolutionY, nPropResolutionX, CV_8UC4);

    // Attempt to open camera with OpenCV's VideoCapture and print if successfully opened or not.
    if (m_cvCamera.open(szCameraPath))
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "SIMCamera {} at path/URL {} has been successfully opened.", m_cvCamera.getBackendName(), szCameraPath);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "Unable to open SIMCamera at path/URL {}", szCameraPath);
    }

    // Publish the initial open status (queried on the constructing thread, before the producer starts).
    m_abCameraOpen.store(m_cvCamera.isOpened(), std::memory_order_release);

    // Set max FPS of the ThreadedContinuousCode method.
    this->SetMainThreadIPSLimit(nPropFramesPerSecond);
}

/******************************************************************************
 * @brief Destroy the SIM Cam:: SIM Cam object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
SIMBasicCam::~SIMBasicCam()
{
    // Stop threaded code.
    this->RequestStop();
    this->Join();

    // Release camera capture object.
    m_cvCamera.release();
}

/******************************************************************************
 * @brief The code inside this private method runs in a separate thread, but still
 *      has access to this*. This method continuously gets new frames from the simulator
 *      stream and publishes a deep-copied snapshot of each one, but only while at least one
 *      consumer is subscribed. Consumers read the newest snapshot on their own schedule, so
 *      this loop never waits on them.
 *
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
void SIMBasicCam::ThreadedContinuousCode()
{
    ZoneScopedC(tracy::Color::Pink1);
    // Check if camera is NOT open. isOpened() only ever runs on this owning thread.
    if (!m_cvCamera.isOpened())
    {
        // Publish that the camera is not open so foreign readers see it lock free.
        m_abCameraOpen.store(false, std::memory_order_release);

        // Log the open -> closed transition exactly once instead of every iteration.
        if (m_bLastKnownOpenState)
        {
            // Remember the new state so we do not log again until it changes back.
            m_bLastKnownOpenState = false;
            // Submit logger message.
            LOG_CRITICAL(logging::g_qSharedLogger,
                         "SIMBasicCam at {}/{} is not open. Retrying every {} ms until it connects; this thread will keep running.",
                         m_nCameraIndex,
                         m_szCameraPath,
                         constants::CAMERA_RECONNECT_RETRY_INTERVAL.count());
        }

        // Rate limit reopen attempts on a monotonic deadline. This thread NEVER stops itself for a
        // missing camera: it idles and keeps retrying, so startup ordering never matters and a
        // camera that appears later is picked up automatically.
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
                LOG_INFO(logging::g_qSharedLogger, "SIMBasicCam {}/{} has been reconnected and reopened!", m_nCameraIndex, m_szCameraPath);
            }
            else
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Attempt to reopen SIMBasicCam {}/{} has failed! Trying again in {} ms...",
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

        // Only produce a frame when a consumer actually wants one.
        if (m_pubFrame.HasSubscribers())
        {
            // TODO: PUT CODE HERE FOR GETTING FRAMES AND DATA FROM SIMULATOR into m_cvFrame.

            // Acquire a pooled snapshot slot and DEEP COPY the frame into it. We must never publish a
            // Mat that aliases a source buffer; copyTo reuses the slot's allocation when sizes match.
            std::shared_ptr<pubsub::Snapshot<cv::Mat>> pSlot = m_pubFrame.Acquire();
            m_cvFrame.copyTo(pSlot->tData);
            // Publish the immutable snapshot as the newest frame.
            m_pubFrame.Publish(std::move(pSlot));
        }
    }
}

/******************************************************************************
 * @brief Not used. Frame distribution is handled by the publish-latest mechanism
 *      in ThreadedContinuousCode(); no per-consumer fan-out work remains.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void SIMBasicCam::PooledLinearCode() {}

/******************************************************************************
 * @brief Accessor for the camera open status.
 *
 * @return true - The camera has been successfully opened.
 * @return false - The camera has not been successfully opened.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
bool SIMBasicCam::GetCameraIsOpen()
{
    // Read the lock-free published open status. The VideoCapture is only ever touched on the owning thread.
    return m_abCameraOpen.load(std::memory_order_acquire) && this->GetThreadState() == AutonomyThreadState::eRunning;
}

/******************************************************************************
 * @brief Accessor for the cameras path or video index.
 *
 * @return std::string - The path or index of the camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
std::string SIMBasicCam::GetCameraLocation() const
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
