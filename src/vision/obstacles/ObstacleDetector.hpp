/******************************************************************************
 * @brief Implements ObstacleDetector class.
 *
 * @file ObstacleDetector.hpp
 * @author Donovan Bale (donovan@balehaus.org)
 * @date 2025-05-02
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../util/vision/ImageOperations.hpp"
#include "ObstacleDetector.h"

ObstacleDetector::ObstacleDetector(std::shared_ptr<BasicCamera> pBasicCam,
                                   const int nDetectorMaxFPS                      = 30,
                                   const bool bEnable_tracking                    = false,
                                   const bool bEnableRecordingFlag                = false,
                                   const int nNumDetectedObstacleRetrievalThreads = 5,
                                   const bool bUsingGpuMats                       = false)
{
    // Initialize member variables
    m_pCamera                              = pBasicCam;
    m_bTorchInitialized                    = false;
    m_bTorchEnabled                        = true;
    m_bEnableTracking                      = bEnable_tracking;
    m_bUsingZedCamera                      = false;    // Toggle ZED functions off.
    m_bUsingGpuMats                        = bUsingGpuMats;
    m_bCameraIsOpened                      = false;
    m_nNumDetectedObstacleRetrievalThreads = nNumDetectedObstacleRetrievalThreads;
    m_szCameraName                         = std::dynamic_pointer_cast<BasicCamera>(pBasicCam)->GetCameraLocation();
    m_bEnableRecordingFlag                 = bEnableRecordingFlag;
    m_IPS                                  = IPS();

    // Create a multi-tracker for tracking multiple obstacles from the torch detectors.
    // @todo Replace with Obstacle Detection specific constants
    m_pMultiTracker = std::make_shared<tracking::MultiTracker>(constants::ARUCO_BBOX_TRACKER_LOST_TIMEOUT,
                                                               constants::ARUCO_BBOX_TRACKER_MAX_TRACK_TIME,
                                                               constants::ARUCO_BBOX_TRACKER_IOU_MATCH_THRESHOLD);

    // Set max IPS of main thread.
    this->SetMainThreadIPSLimit(nDetectorMaxFPS);

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "ObstacleDetector created for camera at path/index: {}", m_szCameraName);
}

ObstacleDetector::ObstacleDetector(std::shared_ptr<ZEDCamera> pZEDCam,
                                   const int nDetectorMaxFPS                      = 30,
                                   const bool bEnable_tracking                    = false,
                                   const bool bEnableRecordingFlag                = false,
                                   const int nNumDetectedObstacleRetrievalThreads = 5,
                                   const bool bUsingGpuMats                       = false)
{
    // Initialize member variables.
    m_pCamera                              = pZEDCam;
    m_bTorchInitialized                    = false;
    m_bTorchEnabled                        = true;
    m_bEnableTracking                      = bEnable_tracking;
    m_bUsingZedCamera                      = true;    // Toggle ZED functions on.
    m_bUsingGpuMats                        = bUsingGpuMats;
    m_bCameraIsOpened                      = false;
    m_nNumDetectedObstacleRetrievalThreads = nNumDetectedObstacleRetrievalThreads;
    m_szCameraName                         = std::dynamic_pointer_cast<BasicCamera>(pZEDCam)->GetCameraLocation();
    m_bEnableRecordingFlag                 = bEnableRecordingFlag;
    m_IPS                                  = IPS();

    // Create a multi-tracker for tracking multiple tags from the torch detectors.
    // @todo Replace with Obstacle Detection specific constants
    m_pMultiTracker = std::make_shared<tracking::MultiTracker>(constants::ARUCO_BBOX_TRACKER_LOST_TIMEOUT, constants::ARUCO_BBOX_TRACKER_IOU_MATCH_THRESHOLD);

    // Set max IPS of main thread.
    this->SetMainThreadIPSLimit(nDetectorMaxFPS);

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "TagDetector created for camera: {}", m_szCameraName);
}

ObstacleDetector::~ObstacleDetector()
{
    // Stop threaded code.
    this->RequestStop();
    this->Join();

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "ObstacleDetector for camera {} had been successfully destroyed.", this->GetCameraName());
}

void ObstacleDetector::ThreadedContinuousCode()
{
    // Check if using ZEDCam or BasicCam.
    if (m_bUsingZedCamera)
    {
        // Check if camera is NOT open.
        if (!std::dynamic_pointer_cast<ZEDCamera>(m_pCamera)->GetCameraIsOpen())
        {
            // Set camera opened toggle.
            m_bCameraIsOpened = false;

            // If camera's not open on first iteration of thread, it's probably not present, so stop.
            if (this->GetThreadState() == AutonomyThreadState::eStarting)
            {
                // Shutdown threads for this ZEDCam.
                this->RequestStop();

                // Submit logger message.
                LOG_CRITICAL(logging::g_qSharedLogger,
                             "TagDetector start was attempted for ZED camera with serial number {}, but camera never properly opened or it has been closed/rebooted! "
                             "This tag detector will now stop.",
                             std::dynamic_pointer_cast<ZEDCamera>(m_pCamera)->GetCameraSerial());
            }
        }
        else
        {
            // Set camera opened toggle.
            m_bCameraIsOpened = true;
        }
    }
    else
    {
        // Check if camera is NOT open.
        if (!std::dynamic_pointer_cast<BasicCamera>(m_pCamera)->GetCameraIsOpen())
        {
            // Set camera opened toggle.
            m_bCameraIsOpened = false;

            // If camera's not open on first iteration of thread, it's probably not present, so stop.
            if (this->GetThreadState() == AutonomyThreadState::eStarting)
            {
                // Shutdown threads for this BasicCam.
                this->RequestStop();

                // Submit logger message.
                LOG_CRITICAL(logging::g_qSharedLogger,
                             "TagDetector start was attempted for BasicCam at {}, but camera never properly opened or it has become disconnected!",
                             std::dynamic_pointer_cast<BasicCamera>(m_pCamera)->GetCameraLocation());
            }
        }
        else
        {
            // Set camera opened toggle.
            m_bCameraIsOpened = true;
        }
    }
    // Check if camera is opened.
    if (m_bCameraIsOpened)
    {
        // Create future for indicating when the frame has been copied.
        std::future<bool> fuPointCloudCopyStatus;
        std::future<bool> fuRegularFrameCopyStatus;

        // Check if using ZED or regular camera.
        if (m_bUsingZedCamera)
        {
            // Check if the ZED camera is returning cv::cuda::GpuMat or cv:Mat.
            if (m_bUsingGpuMats)
            {
                // Grabs point cloud from ZEDCam. Dynamic casts Camera to ZEDCamera* so we can use ZEDCam methods.
                fuPointCloudCopyStatus = std::dynamic_pointer_cast<ZEDCamera>(m_pCamera)->RequestPointCloudCopy(m_cvGPUPointCloud);
                // Get the regular RGB image from the camera.
                fuRegularFrameCopyStatus = std::dynamic_pointer_cast<ZEDCamera>(m_pCamera)->RequestFrameCopy(m_cvGPUFrame);

                // Wait for point cloud to be retrieved.
                if (fuPointCloudCopyStatus.get() && fuRegularFrameCopyStatus.get())
                {
                    // Download mat from GPU memory.
                    m_cvGPUPointCloud.download(m_cvPointCloud);
                    m_cvGPUFrame.download(m_cvFrame);
                    // Drop the Alpha channel from the image copy to preproc frame.
                    cv::cvtColor(m_cvFrame, m_cvFrame, cv::COLOR_BGRA2RGB);
                }
                else
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "TagDetector unable to get point cloud from ZEDCam!");
                }
            }
            else
            {
                // Grabs point cloud from ZEDCam.
                fuPointCloudCopyStatus   = std::dynamic_pointer_cast<ZEDCamera>(m_pCamera)->RequestPointCloudCopy(m_cvPointCloud);
                fuRegularFrameCopyStatus = std::dynamic_pointer_cast<ZEDCamera>(m_pCamera)->RequestFrameCopy(m_cvFrame);

                // Wait for point cloud to be retrieved.
                if (!fuPointCloudCopyStatus.get())
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "TagDetector unable to get point cloud from ZEDCam!");
                }
                if (!fuRegularFrameCopyStatus.get())
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "TagDetector unable to get regular frame from ZEDCam!");
                }
                else if (!m_cvFrame.empty() && m_cvFrame.channels() > 3)
                {
                    // Drop the Alpha channel from the image. This is necessary for the Aruco detection.
                    cv::cvtColor(m_cvFrame, m_cvFrame, cv::COLOR_BGRA2RGB);
                }
            }
        }
        else
        {
            // Grab frames from camera.
            fuPointCloudCopyStatus = std::dynamic_pointer_cast<BasicCamera>(m_pCamera)->RequestFrameCopy(m_cvFrame);

            // Wait for point cloud to be retrieved.
            if (!fuPointCloudCopyStatus.get())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "TagDetector unable to get point cloud from BasicCam!");
            }
            else
            {
                // Check if the camera image is a >3 channel image.
                if (m_cvFrame.channels() > 3)
                {
                    // Drop the Alpha channel from the image copy to preproc frame.
                    cv::cvtColor(m_cvFrame, m_cvFrame, cv::COLOR_BGRA2RGB);
                }

                // Clear list of newly detected obstacles;
                m_vNewlyDetectedObstacles.clear();
            }
        }

        /////////////////////////////////////////
        // Actual detection logic goes here.
        /////////////////////////////////////////
        // Check if the frame is empty.
        if (m_cvFrame.empty())
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "Frame from camera is empty!");
            return;
        }

        m_vDetectedObstacles.clear();

        // torch detection
        // Drop the Alpha channel from the image copy to preproc frame.
        cv::cvtColor(m_cvFrame, m_cvTorchProcFrame, cv::COLOR_BGRA2RGB);
        // Detect tags in the image.
        // @todo Replace with actual detection function.
        inline std::vector<O> detect_template(const cv::Mat& cvFrame,
                                              yolomodel::pytorch::PyTorchInterpreter& tfPyTorchDetector,
                                              const float fMinObjectConfidence = 0.40f,
                                              const float fNMSThreshold        = 0.60f);
        std::vector<O> vNewTorchObstacles = detect_template(m_cvTorchProcFrame, *m_pTorchDetector, m_fTorchMinObjectConfidence, m_fTorchIOUThreshold);
        // Add Torch tags to the list of newly detected tags.
        m_vNewlyDetectedObstacles.insert(m_vNewlyDetectedObstacles.end(), vNewTorchObstacles.begin(), vNewTorchObstacles.end());

        // Set the FOV of the camera in the tag structs for this detector's camera.
        for (O& stTag : m_vNewlyDetectedObstacles)
        {
            // Set tag FOV parameter to this tag detectors camera's FOV.
            stTag.dHorizontalFOV = m_pCamera->GetPropHorizontalFOV();
        }

        // Merge the newly detected tags with the pre-existing detected tags
        this->UpdateDetectedObstacles(m_vNewlyDetectedObstacles);

        //@todo Draw obstacle overlays onto normal image.

        //@todo Name the window the name of the camera.
    }
    // Acquire a shared_lock on the detected obstacle copy queue.
    std::shared_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Check if the detected obstacle copy queue is empty.
    if (!m_qDetectedObstacleDrawnOverlayFramesCopySchedule.empty() || !m_qDetectedObstacleCopySchedule.empty())
    {
        size_t siQueueLength = m_qDetectedObstacleDrawnOverlayFramesCopySchedule.size() + m_qDetectedObstacleCopySchedule.size();
        // Start the thread pool to store multiple copies of the detected tags to the requesting threads
        this->RunDetachedPool(siQueueLength, m_nNumDetectedObstacleRetrievalThreads);
        // Wait for thread pool to finish.
        this->JoinPool();
        // Relaease lock on frame copy queue.
        lkSchedulers.unlock();
    }
}
