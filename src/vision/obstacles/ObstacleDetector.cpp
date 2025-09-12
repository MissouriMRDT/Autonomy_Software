/******************************************************************************
 * @brief Implements ObstacleDetector class.
 *
 * @file ObstacleDetector.hpp
 * @author Donovan Bale (donovan@balehaus.org)
 * @date 2025-05-02
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "ObstacleDetector.h"
#include "../../AutonomyGlobals.h"
#include "../../util/vision/Geolocate.hpp"
#include "../../util/vision/ImageOperations.hpp"

ObstacleDetector::ObstacleDetector(std::shared_ptr<BasicCamera> pBasicCam,
                                   const int nDetectorMaxFPS,
                                   const bool bEnableRecordingFlag,
                                   const int nNumDetectedObstacleRetrievalThreads,
                                   const bool bUsingGpuMats)
{
    // Initialize member variables
    m_pCamera                              = pBasicCam;
    m_bTorchEnabled                        = true;
    m_bUsingZedCamera                      = false;    // Toggle ZED functions off.
    m_bUsingGpuMats                        = bUsingGpuMats;
    m_bCameraIsOpened                      = false;
    m_nNumDetectedObstacleRetrievalThreads = nNumDetectedObstacleRetrievalThreads;
    m_szCameraName                         = std::dynamic_pointer_cast<BasicCamera>(pBasicCam)->GetCameraLocation();
    m_bEnableRecordingFlag                 = bEnableRecordingFlag;
    m_IPS                                  = IPS();

    // Create a multi-tracker for tracking multiple obstacles from the torch detectors.
    // @todo Replace with Obstacle Detection specific constants
    m_pMultiTracker = std::make_shared<tracking::MultiTracker>(constants::BBOX_TRACKER_LOST_TIMEOUT,
                                                               constants::BBOX_TRACKER_MAX_TRACK_TIME,
                                                               constants::BBOX_TRACKER_IOU_MATCH_THRESHOLD);

    // Set max IPS of main thread.
    this->SetMainThreadIPSLimit(nDetectorMaxFPS);

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "ObstacleDetector created for camera at path/index: {}", m_szCameraName);
}

ObstacleDetector::ObstacleDetector(std::shared_ptr<ZEDCamera> pZEDCam,
                                   const int nDetectorMaxFPS,
                                   const bool bEnableRecordingFlag,
                                   const int nNumDetectedObstacleRetrievalThreads,
                                   const bool bUsingGpuMats)

{
    // Initialize member variables.

    m_pCamera                              = pZEDCam;
    m_bTorchInitialized                    = false;
    m_bTorchEnabled                        = true;
    m_bUsingZedCamera                      = true;    // Toggle ZED functions on.
    m_bUsingGpuMats                        = bUsingGpuMats;
    m_bCameraIsOpened                      = false;
    m_nNumDetectedObstacleRetrievalThreads = nNumDetectedObstacleRetrievalThreads;
    m_szCameraName                         = std::dynamic_pointer_cast<BasicCamera>(pZEDCam)->GetCameraLocation();
    m_bEnableRecordingFlag                 = bEnableRecordingFlag;
    m_IPS                                  = IPS();

    // Create a multi-tracker for tracking multiple tags from the torch detectors.
    m_pMultiTracker = std::make_shared<tracking::MultiTracker>(constants::BBOX_TRACKER_LOST_TIMEOUT, constants::BBOX_TRACKER_IOU_MATCH_THRESHOLD);

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

/******************************************************************************
 * @brief Attempt to open the next available Torch hardware and load model at the given
 *      path onto the device.
 *
 * @param szModelPath - The absolute path to the model to open.
 * @param eDevice - The hardware device to launch the Torch model on.
 * @return true - Model was opened and loaded successfully onto the Torch device.
 * @return false - Something went wrong, model/device not opened.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-03-06
 ******************************************************************************/
bool ObstacleDetector::InitTorchDetection(const std::string& szModelPath, yolomodel::pytorch::PyTorchInterpreter::HardwareDevices eDevice)
{
    // Initialize a new YOLOModel object.
    m_pTorchDetector = std::make_shared<yolomodel::pytorch::PyTorchInterpreter>(szModelPath, eDevice);

    // Check if device/model was opened without issue.
    if (m_pTorchDetector->IsReadyForInference())
    {
        // Update member variable.
        m_bTorchInitialized = true;
        // Return status.
        return true;
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "Unable to initialize Torch detection for TagDetector.");
        // Update member variable.
        m_bTorchInitialized = false;
        // Return status.
        return false;
    }
}

/******************************************************************************
 * @brief Turn on torch detection with given parameters.
 *
 * @param fMinObjectConfidence - The lower limit of detection confidence.
 * @param fNMSThreshold - The overlap thresh for NMS algorithm.
 *
 * @author UhOhDonovan (donovan@balehaus.org)
 * @date 2025-09-03
 ******************************************************************************/
void ObstacleDetector::EnableTorchDetection(const float fMinObjectConfidence, const float fNMSThreshold)
{
    // Update member variables.
    m_fTorchMinObjectConfidence = fMinObjectConfidence;
    m_fTorchNMSThreshold        = fNMSThreshold;

    // Check if torch model has been initialized.
    if (!m_bTorchInitialized)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Tried to enable torch detection for TagDetector but it has not been initialized yet!");
        // Update member variable.
        m_bTorchEnabled = false;
    }
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
        std::vector<obstacledetectutils::Obstacle> vDetectedObstacles;

        if (m_pTorchDetector->IsReadyForInference())
        {
            std::vector<yolomodel::Detection> vOutputObstacles = m_pTorchDetector->Inference(m_cvTorchProcFrame, 0.85, 0.6, true);
            for (const yolomodel::Detection& stObstacleDetection : vOutputObstacles)
            {
                torch::Tensor trContiguous = stObstacleDetection.trSegment.contiguous();

                cv::Mat cvMask(trContiguous.size(0), trContiguous.size(1), CV_32FC1, trContiguous.data_ptr<float>());

                obstacledetectutils::Obstacle stDetectedObstacle;
                stDetectedObstacle.dConfidence       = stObstacleDetection.fConfidence;
                stDetectedObstacle.pBoundingBox      = std::make_shared<cv::Rect2d>(stObstacleDetection.cvBoundingBox);
                stDetectedObstacle.pSegmentMask      = std::make_shared<cv::Mat>(cvMask);
                stDetectedObstacle.nID               = stObstacleDetection.nClassID;
                stDetectedObstacle.cvImageResolution = m_cvFrame.size();

                vDetectedObstacles.emplace_back(stDetectedObstacle);
            }
        }

        // Add Torch tags to the list of newly detected tags.
        m_vNewlyDetectedObstacles.insert(m_vNewlyDetectedObstacles.end(), vDetectedObstacles.begin(), vDetectedObstacles.end());

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

void ObstacleDetector::PooledLinearCode()
{
    /////////////////////////////
    //  Detection Overlay Frame queue.
    /////////////////////////////
    // Acquire sole writing access to the detectedTagCopySchedule.
    std::unique_lock<std::shared_mutex> lkObstacleOverlayFrameQueue(m_muFrameCopyMutex);
    // Check if there are unfulfilled requests.
    if (!m_qDetectedObstacleDrawnOverlayFramesCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::FrameFetchContainer<cv::Mat> stContainer = m_qDetectedObstacleDrawnOverlayFramesCopySchedule.front();
        // Pop out of queue.
        m_qDetectedObstacleDrawnOverlayFramesCopySchedule.pop();
        // Release lock.
        lkObstacleOverlayFrameQueue.unlock();

        // Check which frame we should copy.
        switch (stContainer.eFrameType)
        {
            case PIXEL_FORMATS::eObstacleDetection: *stContainer.pFrame = m_cvTorchProcFrame.clone(); break;
            default: *stContainer.pFrame = m_cvTorchProcFrame.clone(); break;
        }

        stContainer.pCopiedFrameStatus->set_value(true);
    }

    /////////////////////////////
    //  Obstacle queue.
    /////////////////////////////
    // Acquire sole writing access to the detectedObstacleCopySchedule.
    std::unique_lock<std::shared_mutex> lkObstacleQueue(m_muObstacleDataCopyMutex);
    // Check if there are unfulfilled requests.
    if (!m_qDetectedObstacleCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::DataFetchContainer<std::vector<obstacledetectutils::Obstacle>> stContainer = m_qDetectedObstacleCopySchedule.front();
        m_qDetectedObstacleCopySchedule.pop();
        // Release lock.
        lkObstacleQueue.unlock();

        // Copy the detected tags to the target location
        *stContainer.pData = m_vDetectedObstacles;

        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedDataStatus->set_value(true);
    }
}

/******************************************************************************
 * @brief Accessor for the camera name or path that this ObstacleDetector is tied to.
 *
 * @return std::string - The name/path/index of the camera used by this ObstacleDetector.
 *
 * @author UhOhDonovan (donovan@balehaus.org)
 * @date 2025-09-12
 ******************************************************************************/
std::string ObstacleDetector::GetCameraName()
{
    return m_szCameraName;
}

void ObstacleDetector::UpdateDetectedObstacles(std::vector<obstacledetectutils::Obstacle>& vNewlyDetectedObstacles)
{
    if (vNewlyDetectedObstacles.empty())
    {
        // I DON'T KNOW IF I NEED THIS OR NOT
        m_pMultiTracker->Update(m_cvFrame);
    }
    else
    {
        for (obstacledetectutils::Obstacle& stNewDetection : vNewlyDetectedObstacles)
        {
            bool bMatchedObstacleToExistingTracker = m_pMultiTracker->InitTracker(m_cvFrame, stNewDetection.pBoundingBox, constants::BBOX_TRACKER_TYPE);
            if (!bMatchedObstacleToExistingTracker)
            {
                m_vDetectedObstacles.emplace_back(stNewDetection);
            }
            else
            {
                for (obstacledetectutils::Obstacle& stExistingObstacle : m_vDetectedObstacles)
                {
                    if (stNewDetection.pBoundingBox == stExistingObstacle.pBoundingBox)
                    {
                        stExistingObstacle.nID         = stNewDetection.nID;
                        stExistingObstacle.dConfidence = stNewDetection.dConfidence;
                    }
                }
            }
        }
        m_pMultiTracker->Update(m_cvFrame);
    }

    for (std::vector<obstacledetectutils::Obstacle>::iterator itObstacle = m_vDetectedObstacles.begin(); itObstacle != m_vDetectedObstacles.end();)
    {
        if (itObstacle->pBoundingBox->x == 0 && itObstacle->pBoundingBox->y == 0 && itObstacle->pBoundingBox->width == 0 && itObstacle->pBoundingBox->height == 0)
        {
            itObstacle = m_vDetectedObstacles.erase(itObstacle);
        }
        else
        {
            ++itObstacle;
        }
    }

    if (m_bUsingZedCamera)
    {
        if (!m_cvPointCloud.empty())
        {
            // Get the rover pose from the waypoint handler.
            m_stRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();
            for (obstacledetectutils::Obstacle& stObstacle : m_vDetectedObstacles)
            {
                // Use either width of height for the neighborhood size.
                int nNeighborhoodSize = std::min(stObstacle.pBoundingBox->width, stObstacle.pBoundingBox->height);
                // Geolocate the obstacle in the point cloud.
                // @todo Geolocate the obstacle based on the segmentation mask
                geoops::Waypoint stGeolocation =
                    geoloc::GeolocateBox(m_cvPointCloud, m_stRoverPose, cv::Point(stObstacle.pBoundingBox->x, stObstacle.pBoundingBox->y), nNeighborhoodSize);
                // Since this is a obstacle detection, set the obstacle's waypoint type appropriately.
                stGeolocation.eType = geoops::WaypointType::eObstacleWaypoint;

                // Check if the geolocation is valid.
                if (stGeolocation != geoops::Waypoint())
                {
                    // Calculate the geo measurement and print the distance to the tag.
                    geoops::GeoMeasurement stMeasurement = geoops::CalculateGeoMeasurement(m_stRoverPose.GetUTMCoordinate(), stGeolocation.GetUTMCoordinate());
                    // Check that the distance is in a reasonable range.
                    if (stMeasurement.dDistanceMeters > 0.0 && stMeasurement.dDistanceMeters < 25.0)
                    {
                        // Set the tag's geolocation.
                        stObstacle.stGeolocatedPosition = stGeolocation;
                        // Use the rover heading and the azimuth angle to calculate the relative heading to the tag.
                        stObstacle.dYawAngle = numops::AngularDifference(m_stRoverPose.GetCompassHeading(), stMeasurement.dStartRelativeBearing);
                        // Set the straight line distance to the tag.
                        stObstacle.dStraightLineDistance = stMeasurement.dDistanceMeters;
                    }
                }
            }
        }
    }
    else
    {
        // Estimate the positions of the tags using some basic trig.
        for (obstacledetectutils::Obstacle& stObstacle : m_vDetectedObstacles)
        {
            obstacledetectutils::EstimatePoseFromCameraFrame(stObstacle);
        }
    }
}
