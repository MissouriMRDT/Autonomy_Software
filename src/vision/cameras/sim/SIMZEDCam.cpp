/******************************************************************************
 * @brief Implements the SIMZEDCam class.
 *
 * @file SIMZEDCam.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "SIMZEDCam.h"

#include "../../../AutonomyConstants.h"
#include "../../../AutonomyGlobals.h"
#include "../../../AutonomyLogging.h"
#include "../../../AutonomyNetworking.h"
#include "../../../util/NumberOperations.hpp"
#include "SIMZEDCamCUDA.h"

/// \cond
#include <cmath>

// #include <omp.h>

/// \endcond

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
 * @param unCameraSerialNumber - The serial number of the camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
SIMZEDCam::SIMZEDCam(const std::string szCameraPath,
                     const int nPropResolutionX,
                     const int nPropResolutionY,
                     const int nPropFramesPerSecond,
                     const double dPropHorizontalFOV,
                     const double dPropVerticalFOV,
                     const bool bEnableRecordingFlag,
                     const int nNumFrameRetrievalThreads,
                     const unsigned int unCameraSerialNumber) :
    ZEDCamera(nPropResolutionX,
              nPropResolutionY,
              nPropFramesPerSecond,
              dPropHorizontalFOV,
              dPropVerticalFOV,
              bEnableRecordingFlag,
              false,
              false,
              nNumFrameRetrievalThreads,
              unCameraSerialNumber)
{
    // Create instance variables.
    std::string szWebsocketAddress = "";
    std::string szFullStreamName   = "";

    // Split the websocket URL from the stream name.
    size_t siPos = szCameraPath.rfind('/');

    // Ensure we found a slash and it's not just the protocol 'ws://'
    if (siPos != std::string::npos && siPos > 6)
    {
        // Assign directly to your instance variables (this copies the data safely)
        szWebsocketAddress = szCameraPath.substr(0, siPos);
        szFullStreamName   = szCameraPath.substr(siPos + 1);

        LOG_NOTICE(logging::g_qSharedLogger, "Address: {}, Identifier: {}", szWebsocketAddress, szFullStreamName);
    }
    else
    {
        LOG_ERROR(logging::g_qSharedLogger, "Invalid Camera Path: {}", szCameraPath);
    }

    // Assign member variables.
    m_szCameraPath                     = szWebsocketAddress;
    m_szFullStreamName                 = szFullStreamName;
    m_nNumFrameRetrievalThreads        = nNumFrameRetrievalThreads;
    m_bCameraPositionalTrackingEnabled = false;
    // Initialize pose offsets to zero (previously left uninitialized).
    m_dPoseOffsetX  = 0.0;
    m_dPoseOffsetY  = 0.0;
    m_dPoseOffsetZ  = 0.0;
    m_dPoseOffsetXO = 0.0;
    m_dPoseOffsetYO = 0.0;
    m_dPoseOffsetZO = 0.0;

    // Initialize OpenCV mats to a black/empty image the size of the camera resolution.
    m_cvFrame        = cv::Mat::zeros(nPropResolutionY, nPropResolutionX, CV_8UC4);
    m_cvDepthImage   = cv::Mat::zeros(nPropResolutionY, nPropResolutionX, CV_8UC1);
    m_cvDepthMeasure = cv::Mat::zeros(nPropResolutionY, nPropResolutionX, CV_32FC1);
    m_cvPointCloud   = cv::Mat::zeros(nPropResolutionY, nPropResolutionX, CV_32FC4);

    // Construct camera stream objects. Append proper camera path arguments to each URL camera path.
    m_pRGBStream        = std::make_unique<WebRTC>(szWebsocketAddress, m_szFullStreamName + "RGB");
    m_pDepthImageStream = std::make_unique<WebRTC>(szWebsocketAddress, m_szFullStreamName + "DepthImage");

    // Set callbacks for the WebRTC connections.
    this->SetCallbacks();

    // Set RoveComm callbacks for the data from the sim.
    if (network::g_pRoveCommUDPNode)
    {
        // Determine the IP address to send the subscribe packet to.
        const manifest::AddressEntry& stIPAddress = constants::MODE_SIM ? constants::SIM_IP_ADDRESS : manifest::RoveSoSimulator::IP_ADDRESS;

        // Send subscribe packet to RoveSoSimulator.
        network::g_pRoveCommUDPNode->Subscribe(stIPAddress, constants::ROVECOMM_OUTGOING_UDP_PORT);

        // Set RoveComm callbacks.
        network::g_pRoveCommUDPNode->On<manifest::RoveSoSimulator::Telemetry::IMU>([this](const auto& stPacket) { ProcessIMUData(stPacket); });
    }

    // Set max FPS of the ThreadedContinuousCode method.
    this->SetMainThreadIPSLimit(nPropFramesPerSecond);

    // Publish an initial status snapshot. The producer thread is not running yet, so this is the
    // only thread touching the streams and the call is safe here. Without it every status accessor
    // (notably GetCameraModel(), which other components read while being constructed) would see a
    // null snapshot and report defaults until the producer's first iteration.
    this->PublishStatus();
}

/******************************************************************************
 * @brief Destroy the SIM Cam:: SIM Cam object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
SIMZEDCam::~SIMZEDCam()
{
    LOG_NOTICE(logging::g_qSharedLogger, "Destroying SIMZEDCam object.");
    // Stop threaded code FIRST. The producer thread owns the stream objects: its reconnect path
    // (ImplReconnectStreams) closes, destroys and rebuilds them. Touching those pointers from this
    // thread before the producer is joined races that rebuild and can hang inside CloseConnection().
    // Once Join() returns, this thread is the only one left that can reach the streams.
    this->RequestStop();
    this->Join();

    // Do NOT close the streams explicitly here. ~WebRTC already closes its own connections, and
    // calling CloseConnection() first makes every stream pay the close-wait twice. The unique_ptr
    // members are destroyed after this body runs, which is still safely after Join().

    // Shut down the command queue so any command posted after the producer thread stopped is
    // cancelled (its future resolves with an error) rather than left stranded.
    m_cmdQueue.Shutdown();
}

/******************************************************************************
 * @brief This method sets the callbacks for the WebRTC connections.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
void SIMZEDCam::SetCallbacks()
{
    // Set the frame callbacks.
    m_pRGBStream->SetOnFrameReceivedCallback(
        [this](cv::Mat& cvFrame)
        {
            // Check if the frame is empty.
            if (!cvFrame.empty())
            {
                // Acquire a lock on the webRTC copy mutex.
                std::unique_lock lkWebRTC(m_muWebRTCRGBImageCopyMutex);
                // Deep copy the frame.
                m_cvFrame = cvFrame.clone();
            }
        });
    m_pDepthImageStream->SetOnFrameReceivedCallback(
        [this](cv::Mat& cvFrame)
        {
            // Check if the frame is empty.
            if (!cvFrame.empty())
            {
                // Acquire a lock on the webRTC copy mutex.
                std::unique_lock lkWebRTC(m_muWebRTCDepthImageCopyMutex);
                // Deep copy the frame to the depth image buffer.
                m_cvDepthImageBuffer = cvFrame.clone();
                // Convert the depth image buffer to grayscale.
                cv::cvtColor(m_cvDepthImageBuffer, m_cvDepthImage, cv::COLOR_BGR2GRAY);
            }
        });
}

/******************************************************************************
 * @brief This method estimates the depth measure from the depth image.
 *
 * @param cvDepthImage - The depth image to estimate the depth measure from.
 * @param cvDepthMeasure - The estimated depth measure that will be written to.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-29
 ******************************************************************************/
void SIMZEDCam::EstimateDepthMeasure(const cv::Mat& cvDepthImage, cv::Mat& cvDepthMeasure)
{
    ZoneScopedC(tracy::Color::Orange2);
    // Declare instance variables.
    const float fMaxDepth = 2001.0f;    // Maximum depth in cm.

    // Check if the depth image is empty.
    if (cvDepthImage.empty())
    {
        // If the depth image is empty, fill the depth measure with zeros.
        cvDepthMeasure = cv::Mat::zeros(cvDepthMeasure.size(), CV_32FC1);
        return;
    }

    if (constants::SIM_DEPTH_STREAM_USE_GPU)
    {
        // Estimate the depth measure using CUDA.
        EstimateDepthMeasureCUDA(cvDepthImage, cvDepthMeasure, fMaxDepth);
    }
    else
    {
        // #pragma omp parallel for collapse(2)

        // Iterate over each pixel in the cvDepthImage image.
        for (int nY = 0; nY < cvDepthImage.rows; ++nY)
        {
            for (int nX = 0; nX < cvDepthImage.cols; ++nX)
            {
                // For this, we are just using the depth image to estimate the depth measure. We will treat 255 as 0 cm and 0 as fMaxDepth - 1 cm.
                // Get the depth value from the depth image.
                uchar ucDepthValue = cvDepthImage.at<uchar>(nY, nX);

                // Calculate the depth in cm.
                float fDepth = (1.0f - (ucDepthValue / 255.0f)) * fMaxDepth;
                // Check if nY and nX are within the bounds of the depth measure image.
                if (nY < cvDepthMeasure.rows && nX < cvDepthMeasure.cols)
                {
                    // Store the estimated depth in the new cv::Mat. Convert cm to m.
                    cvDepthMeasure.at<float>(nY, nX) = fDepth / 100.0f;    // Convert cm to m.
                }
            }
        }
    }
}

/******************************************************************************
 * @brief This method calculates a point cloud from the decoded depth measure
 *      use some simple trig and the camera FOV.
 *
 * @param cvDepthMeasure - The decoded depth measure.
 * @param cvPointCloud - The point cloud that will be written to.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-04
 ******************************************************************************/
void SIMZEDCam::CalculatePointCloud(const cv::Mat& cvDepthMeasure, cv::Mat& cvPointCloud)
{
    ZoneScopedC(tracy::Color::Orange2);
    // Calculate focal lengths from FOV.
    const double dRadPerDeg = M_PI / 180.0;
    const double dFx        = (cvDepthMeasure.cols / 2.0) / tan(m_dPropHorizontalFOV * dRadPerDeg / 2.0);
    const double dFy        = (cvDepthMeasure.rows / 2.0) / tan(m_dPropVerticalFOV * dRadPerDeg / 2.0);
    // Image center.
    const double dCx = cvDepthMeasure.cols / 2.0;
    const double dCy = cvDepthMeasure.rows / 2.0;

    if (constants::SIM_DEPTH_STREAM_USE_GPU)
    {
        // Calculate the point cloud using CUDA.
        CalculatePointCloudCUDA(cvDepthMeasure, cvPointCloud, dFx, dFy, dCx, dCy);
    }
    else
    {
        // This is a parallel for loop that calculates the point cloud from the decoded depth measure.
        // #pragma omp parallel for collapse(2)

        // Iterate over each pixel in the cvDepthMeasure image.
        for (int nY = 0; nY < cvDepthMeasure.rows; ++nY)
        {
            for (int nX = 0; nX < cvDepthMeasure.cols; ++nX)
            {
                // Get depth value.
                float fDepth = cvDepthMeasure.at<float>(nY, nX);

                // Skip invalid depth values.
                if (fDepth <= 0)
                {
                    cvPointCloud.at<cv::Vec4f>(nY, nX) = cv::Vec4f(0, 0, 0, 0);
                    continue;
                }

                // Convert from pixel coordinates to 3D coordinates.
                float fX = static_cast<float>((nX - dCx) * fDepth / dFx);
                float fY = static_cast<float>((dCy - nY) * fDepth / dFy);
                float fZ = fDepth;

                // Store point. (XYZ + intensity, using Y channel for intensity)
                cvPointCloud.at<cv::Vec4f>(nY, nX) = cv::Vec4f(fX, fY, fZ, 255);
            }
        }
    }
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
void SIMZEDCam::ThreadedContinuousCode()
{
    // 1. Control channel in. SetPositionalPose/ResetPositionalTracking run here, on this thread.
    m_cmdQueue.DrainAll();

    // 2. Handle disconnected streams. This thread NEVER stops itself when the simulator is
    //    unreachable: it idles, retries the connection on a monotonic timer, and publishes NO
    //    imagery. Publishing while disconnected would emit the constructor's all-black scratch
    //    Mats at full rate with an advancing sequence number, which consumers cannot distinguish
    //    from real frames and which defeats their "skip if unchanged" short circuit.
    if (!this->GetStreamsAreConnected())
    {
        // Log the connected -> disconnected transition exactly once instead of every iteration.
        if (m_bLastKnownOpenState)
        {
            // Remember the new state so we do not log again until it changes back.
            m_bLastKnownOpenState = false;
            // Submit logger message.
            LOG_CRITICAL(logging::g_qSharedLogger,
                         "SIM camera {} streams are not connected (is the simulator running at {}?). Retrying every {} ms; this thread will keep running.",
                         m_szFullStreamName,
                         m_szCameraPath,
                         constants::SIM_STREAM_RECONNECT_RETRY_INTERVAL.count());
        }

        // Rate limit reconnect attempts on a monotonic deadline. WebRTC does not retry on its
        // own (ConnectToSignallingServer is called once at construction and onClosed only logs),
        // so rebuilding the stream objects here is what recovers a simulator that starts late.
        if (m_tmReconnectTimer.Ready())
        {
            // Rebuild both stream objects and reattach their frame callbacks.
            this->ImplReconnectStreams();
        }

        // Publish the updated (disconnected) status and produce no data this iteration.
        this->PublishStatus();
        return;
    }

    // Log the disconnected -> connected transition exactly once.
    if (!m_bLastKnownOpenState)
    {
        // Remember the new state so the recovery is reported a single time.
        m_bLastKnownOpenState = true;
        // Submit logger message.
        LOG_INFO(logging::g_qSharedLogger, "SIM camera {} streams have connected and are producing frames.", m_szFullStreamName);
    }

    // 3. Poll the NavBoard for the current rover pose. m_stCurrentRoverPose is owning-thread-local.
    if (globals::g_pNavigationBoard != nullptr)
    {
        // Get the current rover pose from the NavBoard.
        m_stCurrentRoverPose = geoops::RoverPose(globals::g_pNavigationBoard->GetGPSData(), globals::g_pNavigationBoard->GetHeading());
    }

    // 4. RGB frame out. The RGB callback writes m_cvFrame on a foreign thread, so read under the lock.
    if (m_pubFrameCPU.HasSubscribers())
    {
        // Acquire a read lock so the WebRTC callback does not write m_cvFrame mid-copy.
        std::shared_lock<std::shared_mutex> lkRGB(m_muWebRTCRGBImageCopyMutex);
        if (!m_cvFrame.empty())
        {
            // Deep copy the frame into a pooled snapshot and publish.
            std::shared_ptr<pubsub::Snapshot<cv::Mat>> pSlot = m_pubFrameCPU.Acquire();
            m_cvFrame.copyTo(pSlot->tData);
            lkRGB.unlock();
            m_pubFrameCPU.Publish(std::move(pSlot));
        }
    }

    // 5. Depth image and the products derived from it (measure, point cloud).
    const bool bDepthImageWanted   = m_pubDepthImageCPU.HasSubscribers();
    const bool bDepthMeasureWanted = m_pubDepthMeasureCPU.HasSubscribers();
    const bool bPointCloudWanted   = m_pubPointCloudCPU.HasSubscribers();
    if (bDepthImageWanted || bDepthMeasureWanted || bPointCloudWanted)
    {
        // Under the depth lock, publish the depth image and compute the depth measure (both need m_cvDepthImage).
        bool bHaveDepth = false;
        {
            // Acquire a read lock so the WebRTC callback does not write m_cvDepthImage mid-read.
            std::shared_lock<std::shared_mutex> lkDepth(m_muWebRTCDepthImageCopyMutex);
            if (!m_cvDepthImage.empty())
            {
                // Mark that we have a valid depth image this iteration.
                bHaveDepth = true;
                // Publish the depth image.
                if (bDepthImageWanted)
                {
                    // Deep copy the depth image into a pooled snapshot and publish.
                    std::shared_ptr<pubsub::Snapshot<cv::Mat>> pSlot = m_pubDepthImageCPU.Acquire();
                    m_cvDepthImage.copyTo(pSlot->tData);
                    m_pubDepthImageCPU.Publish(std::move(pSlot));
                }
                // Compute the depth measure (needed by both the measure and point-cloud publishers).
                if (bDepthMeasureWanted || bPointCloudWanted)
                {
                    // Estimate the depth measure from the depth image into the producer-local Mat.
                    this->EstimateDepthMeasure(m_cvDepthImage, m_cvDepthMeasure);
                }
            }
        }

        // Depth measure and point cloud are producer-local, so publish them outside the depth lock.
        if (bHaveDepth)
        {
            // Publish the depth measure.
            if (bDepthMeasureWanted)
            {
                // Deep copy the depth measure into a pooled snapshot and publish.
                std::shared_ptr<pubsub::Snapshot<cv::Mat>> pSlot = m_pubDepthMeasureCPU.Acquire();
                m_cvDepthMeasure.copyTo(pSlot->tData);
                m_pubDepthMeasureCPU.Publish(std::move(pSlot));
            }
            // Compute and publish the point cloud.
            if (bPointCloudWanted)
            {
                // Calculate the point cloud from the estimated depth measure.
                this->CalculatePointCloud(m_cvDepthMeasure, m_cvPointCloud);
                // Deep copy the point cloud into a pooled snapshot and publish.
                std::shared_ptr<pubsub::Snapshot<cv::Mat>> pSlot = m_pubPointCloudCPU.Acquire();
                m_cvPointCloud.copyTo(pSlot->tData);
                m_pubPointCloudCPU.Publish(std::move(pSlot));
            }
        }
    }

    // 6. Pose out (only while positional tracking is enabled).
    if (m_bCameraPositionalTrackingEnabled.load(std::memory_order_acquire) && m_pubPose.HasSubscribers())
    {
        // Get angle realignments.
        double dNewYO = numops::InputAngleModulus<double>(m_stCurrentRoverPose.GetCompassHeading() + m_dPoseOffsetYO, 0.0, 360.0);
        // Repack values into pose.
        Pose stPose(m_stCurrentRoverPose.GetUTMCoordinate().dEasting + m_dPoseOffsetX,
                    m_stCurrentRoverPose.GetUTMCoordinate().dAltitude + m_dPoseOffsetY,
                    m_stCurrentRoverPose.GetUTMCoordinate().dNorthing + m_dPoseOffsetZ,
                    m_dPoseOffsetXO,
                    dNewYO,
                    m_dPoseOffsetZO);
        // Acquire a pooled slot, store the pose, and publish.
        std::shared_ptr<pubsub::Snapshot<Pose>> pSlot = m_pubPose.Acquire();
        pSlot->tData                                  = stPose;
        m_pubPose.Publish(std::move(pSlot));
    }

    // 7. Sensors (IMU) out. The IMU callback writes m_stIMUData on a foreign thread; read under the lock.
    if (m_pubSensors.HasSubscribers())
    {
        // Acquire a read lock so the RoveComm IMU callback does not write m_stIMUData mid-copy.
        std::shared_lock<std::shared_mutex> lkIMU(m_muIMUDataMutex);
        // Deep copy the sensor data into a pooled snapshot and publish.
        std::shared_ptr<pubsub::Snapshot<sl::SensorsData>> pSlot = m_pubSensors.Acquire();
        pSlot->tData                                             = m_stIMUData;
        lkIMU.unlock();
        m_pubSensors.Publish(std::move(pSlot));
    }

    // 8. Status out.
    this->PublishStatus();
}

/******************************************************************************
 * @brief Check whether both simulator video streams are currently connected. This is
 *      the single source of truth for "is this camera open", used by both the producer
 *      loop's gate and the published CameraStatus.
 *
 * @return true - Both streams are constructed and connected.
 * @return false - At least one stream is missing or disconnected.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-26
 ******************************************************************************/
bool SIMZEDCam::GetStreamsAreConnected() const
{
    // Both streams must be constructed and their signalling websockets open.
    return (m_pRGBStream != nullptr && m_pRGBStream->GetIsConnected()) && (m_pDepthImageStream != nullptr && m_pDepthImageStream->GetIsConnected());
}

/******************************************************************************
 * @brief Build and publish the SIM camera status snapshot so the status
 *      accessors are lock-free reads.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void SIMZEDCam::PublishStatus()
{
    // Build the status from the current connection state (queried on the owning thread).
    CameraStatus stStatus;
    stStatus.bCameraIsOpen              = this->GetStreamsAreConnected();
    stStatus.bPositionalTrackingEnabled = m_bCameraPositionalTrackingEnabled.load(std::memory_order_acquire) && stStatus.bCameraIsOpen;
    stStatus.szCameraModel              = "SIMZED2i";

    // Publish the status snapshot.
    std::shared_ptr<pubsub::Snapshot<CameraStatus>> pSlot = m_pubStatus.Acquire();
    pSlot->tData                                          = stStatus;
    m_pubStatus.Publish(std::move(pSlot));
}

/******************************************************************************
 * @brief Not used. Frame distribution is handled by the publish-latest mechanism
 *      in ThreadedContinuousCode(); no per-consumer fan-out work remains.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
void SIMZEDCam::PooledLinearCode() {}

/******************************************************************************
 * @brief This method is used to reset the positional tracking of the camera.
 *      Because this is a simulation camera, and then ZEDSDK is not available,
 *      this method will just reset the offsets to zero.
 *
 * @return sl::ERROR_CODE - The error code returned by the ZED SDK. In this case, it will always be SUCCESS.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
sl::ERROR_CODE SIMZEDCam::ResetPositionalTracking()
{
    // Post to the owning thread and block for the result; the offsets are read by the producer.
    return this->RunOnOwningThread<sl::ERROR_CODE>([this]() { return this->ImplResetPositionalTracking(); }, sl::ERROR_CODE::FAILURE, "ResetPositionalTracking");
}

/******************************************************************************
 * @brief Owning-thread implementation of ResetPositionalTracking().
 *
 * @return sl::ERROR_CODE - Always SUCCESS for the simulated camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
sl::ERROR_CODE SIMZEDCam::ImplResetPositionalTracking()
{
    // Reset offsets back to zero.
    m_dPoseOffsetX  = 0.0;
    m_dPoseOffsetY  = 0.0;
    m_dPoseOffsetZ  = 0.0;
    m_dPoseOffsetXO = 0.0;
    m_dPoseOffsetYO = 0.0;
    m_dPoseOffsetZO = 0.0;

    return sl::ERROR_CODE::SUCCESS;
}

/******************************************************************************
 * @brief This method is used to reboot the camera. This method will stop the camera thread,
 *      join the camera thread, destroy the camera stream objects, reconstruct the camera stream
 *      objects, set the frame callbacks, and restart the camera thread. This simulates a camera
 *      reboot since the ZED SDK is not available.
 *
 * @return sl::ERROR_CODE - The error code returned by the ZED SDK. In this case, it will always be SUCCESS.
 *              Even if the streams are not successfully reconnected, the camera will still be considered open.
 *
 * @author clayjay3 (claytonraycowen@gmail.com), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
sl::ERROR_CODE SIMZEDCam::RebootCamera()
{
    // Stop the camera thread.
    this->RequestStop();
    // Join the camera thread.
    this->Join();

    // Tear down and rebuild the streams now that no producer thread is touching them.
    this->ImplReconnectStreams();

    // Restart the camera thread.
    this->Start();

    return sl::ERROR_CODE::SUCCESS;
}

/******************************************************************************
 * @brief Tear down and rebuild the WebRTC stream objects. Runs on whichever thread owns
 *      the streams at the time: the producer thread (from the reconnect path in
 *      ThreadedContinuousCode) or a foreign thread that has already stopped and joined
 *      the producer (RebootCamera).
 *
 * @note Do NOT call RebootCamera() from the producer thread. It calls RequestStop() and
 *      Join() on itself, and Join() waits on the very thread pool the caller is running
 *      in, which self-deadlocks (or throws, since the pool is built with
 *      BS_THREAD_POOL_ENABLE_WAIT_DEADLOCK_CHECK). The producer loop calls this helper
 *      instead, which never touches thread lifecycle.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-26
 ******************************************************************************/
void SIMZEDCam::ImplReconnectStreams()
{
    // Close the existing connections so their callbacks stop firing before we destroy them.
    if (m_pRGBStream != nullptr)
    {
        // Shut down the RGB connection.
        m_pRGBStream->CloseConnection();
    }
    if (m_pDepthImageStream != nullptr)
    {
        // Shut down the depth connection.
        m_pDepthImageStream->CloseConnection();
    }

    // Destroy the camera stream objects.
    m_pRGBStream.reset();
    m_pDepthImageStream.reset();
    // Reconstruct camera stream objects. Append proper camera path arguments to each URL camera path.
    m_pRGBStream        = std::make_unique<WebRTC>(m_szCameraPath, m_szFullStreamName + "RGB");
    m_pDepthImageStream = std::make_unique<WebRTC>(m_szCameraPath, m_szFullStreamName + "DepthImage");

    // Set the frame callbacks on the new stream objects.
    this->SetCallbacks();
}

/******************************************************************************
 * @brief This method is used to enable positional tracking on the camera.
 *      Since this is a simulation camera, this method will just set the member variable
 *      and then use the NavBoard to simulate the camera's position.
 *
 * @param fExpectedCameraHeightFromFloorTolerance - The expected camera height from the floor tolerance.
 * @return sl::ERROR_CODE - The error code returned by the ZED SDK. In this case, it will always be SUCCESS.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
sl::ERROR_CODE SIMZEDCam::EnablePositionalTracking(const float fExpectedCameraHeightFromFloorTolerance)
{
    // Unused parameter.
    (void) fExpectedCameraHeightFromFloorTolerance;
    // Update member variables.
    m_bCameraPositionalTrackingEnabled = true;

    return sl::ERROR_CODE::SUCCESS;
}

/******************************************************************************
 * @brief This method is used to disable positional tracking on the camera.
 *      Since this is a simulation camera, this method will just set the member variable.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
void SIMZEDCam::DisablePositionalTracking()
{
    // Update member variables.
    m_bCameraPositionalTrackingEnabled = false;
}

/******************************************************************************
 * @brief This method is used to set the positional pose of the camera.
 *      Since this is a simulation camera, this method will just set the offset member variables.
 *
 * @param dX - The new X position of the camera in ZED_MEASURE_UNITS.
 * @param dY - The new Y position of the camera in ZED_MEASURE_UNITS.
 * @param dZ - The new Z position of the camera in ZED_MEASURE_UNITS.
 * @param dXO - The new tilt of the camera around the X axis in degrees.
 * @param dYO - The new tilt of the camera around the Y axis in degrees.
 * @param dZO - The new tilt of the camera around the Z axis in degrees.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
void SIMZEDCam::SetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO)
{
    // Post to the owning thread and block until done. This reads m_stCurrentRoverPose, which the
    // producer thread polls, so it must run on the owning thread to avoid racing it.
    this->RunOnOwningThreadVoid([this, dX, dY, dZ, dXO, dYO, dZO]() { this->ImplSetPositionalPose(dX, dY, dZ, dXO, dYO, dZO); }, "SetPositionalPose");
}

/******************************************************************************
 * @brief Owning-thread implementation of SetPositionalPose().
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void SIMZEDCam::ImplSetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO)
{
    // Update offset member variables.
    m_dPoseOffsetX  = dX - m_stCurrentRoverPose.GetUTMCoordinate().dEasting;
    m_dPoseOffsetY  = dY - m_stCurrentRoverPose.GetUTMCoordinate().dAltitude;
    m_dPoseOffsetZ  = dZ - m_stCurrentRoverPose.GetUTMCoordinate().dNorthing;
    m_dPoseOffsetXO = dXO;
    m_dPoseOffsetYO = dYO - m_stCurrentRoverPose.GetCompassHeading();
    m_dPoseOffsetZO = dZO;
}

/******************************************************************************
 * @brief Accessor for the camera open status.
 *
 * @return true - The camera has been successfully opened.
 * @return false - The camera has not been successfully opened.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
bool SIMZEDCam::GetCameraIsOpen()
{
    // Lock-free read of the newest published status snapshot.
    pubsub::Publisher<CameraStatus>::SharedSnapshot pStatus = m_pubStatus.Get();
    return pStatus != nullptr && pStatus->tData.bCameraIsOpen && this->GetThreadState() == AutonomyThreadState::eRunning;
}

/******************************************************************************
 * @brief Returns if the camera is using GPU memory. This is a simulation camera,
 *       so this method will always return false.
 *
 * @return true - We are using GPU memory.
 * @return false - We are not using GPU memory.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
bool SIMZEDCam::GetUsingGPUMem() const
{
    return false;
}

/******************************************************************************
 * @brief Accessor for the name of this model of camera.
 *
 * @return std::string - The model of the camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
std::string SIMZEDCam::GetCameraModel()
{
    return "SIMZED2i";
}

/******************************************************************************
 * @brief Accessor for the if the camera's positional tracking is enabled.
 *      Since this is a simulation camera, this method will just return the member variable.
 *
 * @return true - Positional tracking is enabled.
 * @return false - Positional tracking is disabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
bool SIMZEDCam::GetPositionalTrackingEnabled()
{
    // Lock-free read of the newest published status snapshot.
    pubsub::Publisher<CameraStatus>::SharedSnapshot pStatus = m_pubStatus.Get();
    return pStatus != nullptr && pStatus->tData.bPositionalTrackingEnabled && this->GetThreadState() == AutonomyThreadState::eRunning;
}

/******************************************************************************
 * @brief Callback function to process incoming IMU data from RoveComm for the SIM ZED Camera.
 *      Normally, this data would come from the physical ZED camera's IMU over USB,
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-11-19
 ******************************************************************************/
void SIMZEDCam::ProcessIMUData(const rovecomm::RoveCommPacket<double>& stPacket)
{
    // Acquire a write lock on the IMU data handoff mutex (this callback runs on a foreign
    // RoveComm thread; the producer thread reads m_stIMUData under the same lock).
    std::unique_lock lkSensorsProcessLock(m_muIMUDataMutex);
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
}
