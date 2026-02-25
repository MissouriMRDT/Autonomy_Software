/******************************************************************************
 * @brief This code will run continuously in a separate thread. New frames from
 *     the given camera are grabbed and the objects for the camera image are detected,
 *     filtered, and stored. Then any requests for the current objects are fulfilled.
 *
 * @file ObjectDetector.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "./ObjectDetector.h"
#include "../../AutonomyGlobals.h"
#include "../../util/vision/Geolocate.hpp"
#include "./TorchObjectDetection.hpp"

/// \cond

/// \endcond

/******************************************************************************
 * @brief Construct a new Object Detector:: Object Detector object.
 *
 * @param pBasicCam - A pointer to the BasicCam to use for detection.
 * @param bEnableTracking - Whether or not to enable tracking of detected objects.
 * @param nDetectorMaxFPS - The max FPS limit the detector can run at.
 * @param bEnableRecordingFlag - Whether or not this ObjectDetector's overlay output should be recorded.
 * @param nNumDetectedObjectsRetrievalThreads - The number of threads to use when fulfilling
 *                                           requests for the detected objects. Default is 5.
 * @param bUsingGpuMats - Whether or not the given camera name will be using GpuMats.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
ObjectDetector::ObjectDetector(std::shared_ptr<BasicCamera> pBasicCam,
                               const bool bEnableTracking,
                               const int nDetectorMaxFPS,
                               const bool bEnableRecordingFlag,
                               const int nNumDetectedObjectsRetrievalThreads,
                               const bool bUsingGpuMats)
{
    // Initialize member variables.
    m_pCamera                             = pBasicCam;
    m_bEnableTracking                     = bEnableTracking;
    m_bUsingZedCamera                     = false;    // Toggle ZED functions off.
    m_bEnableRecordingFlag                = bEnableRecordingFlag;
    m_nNumDetectedObjectsRetrievalThreads = nNumDetectedObjectsRetrievalThreads;
    m_bUsingGpuMats                       = bUsingGpuMats;
    m_bTorchInitialized                   = false;
    m_bTorchEnabled                       = false;
    m_bCameraIsOpened                     = false;
    m_szCameraName                        = pBasicCam->GetCameraLocation();
    m_stRoverPose                         = geoops::RoverPose();

    // Create a multi-tracker for tracking multiple objects from the torch detectors.
    m_pMultiTracker = std::make_shared<tracking::MultiTracker>(constants::BBOX_TRACKER_LOST_TIMEOUT,
                                                               constants::BBOX_TRACKER_MAX_TRACK_TIME,
                                                               constants::BBOX_TRACKER_IOU_MATCH_THRESHOLD);

    // Set max IPS of main thread.
    this->SetMainThreadIPSLimit(nDetectorMaxFPS);

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "ObjectDetector created for camera at path/index: {}", m_szCameraName);
}

/******************************************************************************
 * @brief Construct a new Object Detector:: Object Detector object.
 *
 * @param pZEDCam - A pointer to the ZEDCamera to use for detection.
 * @param bEnableTracking - Whether or not to enable tracking of detected objects.
 * @param nDetectorMaxFPS - The max FPS limit the detector can run at.
 * @param bEnableRecordingFlag - Whether or not this ObjectDetector's overlay output should be recorded.
 * @param nNumDetectedObjectsRetrievalThreads - The number of threads to use when fulfilling
 *                                           requests for the detected objects. Default is 5.
 * @param bUsingGpuMats - Whether or not the given camera name will be using GpuMats.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
ObjectDetector::ObjectDetector(std::shared_ptr<ZEDCamera> pZEDCam,
                               const bool bEnableTracking,
                               const int nDetectorMaxFPS,
                               const bool bEnableRecordingFlag,
                               const int nNumDetectedObjectsRetrievalThreads,
                               const bool bUsingGpuMats)
{
    // Initialize member variables.
    m_pCamera                             = pZEDCam;
    m_bEnableTracking                     = bEnableTracking;
    m_bUsingZedCamera                     = true;    // Toggle ZED functions on.
    m_bEnableRecordingFlag                = bEnableRecordingFlag;
    m_nNumDetectedObjectsRetrievalThreads = nNumDetectedObjectsRetrievalThreads;
    m_bUsingGpuMats                       = bUsingGpuMats;
    m_bTorchInitialized                   = false;
    m_bTorchEnabled                       = false;
    m_bCameraIsOpened                     = false;
    m_szCameraName                        = pZEDCam->GetCameraModel() + "_" + std::to_string(pZEDCam->GetCameraSerial());
    m_stRoverPose                         = geoops::RoverPose();

    // Create a multi-tracker for tracking multiple objects from the torch detectors.
    m_pMultiTracker = std::make_shared<tracking::MultiTracker>(constants::BBOX_TRACKER_LOST_TIMEOUT,
                                                               constants::BBOX_TRACKER_IOU_MATCH_THRESHOLD,
                                                               constants::BBOX_TRACKER_IOU_MATCH_THRESHOLD);

    // Set max IPS of main thread.
    this->SetMainThreadIPSLimit(nDetectorMaxFPS);

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "ObjectDetector created for camera: {}", m_szCameraName);
}

/******************************************************************************
 * @brief Destroy the Object Detector:: Object Detector object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
ObjectDetector::~ObjectDetector()
{
    // Stop threaded code.
    this->RequestStop();
    this->Join();

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "ObjectDetector for camera {} has been destroyed.", this->GetCameraName());
}

/******************************************************************************
 * @brief This method will run continuously in a separate thread. New frames from
 *      the given camera are grabbed and the objects for the camera image are detected
 *      using the PyTorch interpreter. The detected objects are then filtered and stored.
 *      Then any requests for the current objects are fulfilled via a call and join of the
 *      thread pooled code.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
void ObjectDetector::ThreadedContinuousCode()
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
                             "ObjectDetector start was attempted for ZED camera with serial number {}, but camera never properly opened or it has been closed/rebooted! "
                             "This object detector will now stop.",
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
                             "ObjectDetector start was attempted for BasicCam at {}, but camera never properly opened or it has become disconnected!",
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

        // Check if the camera is setup to use CPU or GPU mats.
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
                    // Drop alpha channel.
                    cv::cvtColor(m_cvFrame, m_cvFrame, cv::COLOR_BGRA2BGR);
                }
                else
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "ObjectDetector unable to get point cloud from ZEDCam!");
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
                    LOG_WARNING(logging::g_qSharedLogger, "ObjectDetector unable to get point cloud from ZEDCam!");
                }
                if (!fuRegularFrameCopyStatus.get())
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "ObjectDetector unable to get regular frame from ZEDCam!");
                }
            }
        }
        else
        {
            // Grab frames from camera.
            fuRegularFrameCopyStatus = std::dynamic_pointer_cast<BasicCamera>(m_pCamera)->RequestFrameCopy(m_cvFrame);

            // Wait for point cloud to be retrieved.
            if (!fuRegularFrameCopyStatus.get())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "ObjectDetector unable to get RGB image from BasicCam!");
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

        // Clear the list of newly detected objects.
        m_vNewlyDetectedObjects.clear();
        // Clone frames.
        m_cvTorchOverlayFrame = m_cvFrame.clone();
        m_cvTorchProcFrame    = m_cvFrame.clone();
        // Copy the camera frame to the pre-processing frame and overlay frame.
        cv::cvtColor(m_cvTorchProcFrame, m_cvTorchProcFrame, cv::COLOR_BGR2RGB);

        // Check if torch detection if turned on.
        if (m_bTorchEnabled)
        {
            // Detect objects in the image.
            std::vector<objectdetectutils::Object> vNewTorchObjects =
                torchobject::Detect(m_cvTorchProcFrame, *m_pTorchDetector, m_fTorchMinObjectConfidence, m_fTorchNMSThreshold);
            // Add Torch objects to the list of newly detected objects.
            m_vNewlyDetectedObjects.insert(m_vNewlyDetectedObjects.end(), vNewTorchObjects.begin(), vNewTorchObjects.end());
        }

        // Set the FOV of the camera in the object structs for this detector's camera.
        for (objectdetectutils::Object& stObject : m_vNewlyDetectedObjects)
        {
            // Set object FOV parameter to this object detectors camera's FOV.
            stObject.dHorizontalFOV = m_pCamera->GetPropHorizontalFOV();
        }

        // Merge the newly detected objects with the pre-existing detected objects.
        this->UpdateDetectedObjects(m_vNewlyDetectedObjects);

        // Draw object overlays onto normal image.
        torchobject::DrawDetections(m_cvTorchOverlayFrame, m_vDetectedObjects);
        /////////////////////////////////////////////////////////////////////////////////////
    }

    // Acquire a shared_lock on the detected objects copy queue.
    std::shared_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Check if the detected object copy queue is empty.
    if (!m_qDetectedObjectDrawnOverlayFramesCopySchedule.empty() || !m_qDetectedObjectCopySchedule.empty())
    {
        size_t siQueueLength = m_qDetectedObjectDrawnOverlayFramesCopySchedule.size() + m_qDetectedObjectCopySchedule.size();
        // Start the thread pool to store multiple copies of the detected objects to the requesting threads
        this->RunDetachedPool(siQueueLength, m_nNumDetectedObjectsRetrievalThreads);
        // Wait for thread pool to finish.
        this->JoinPool();
        // Release lock on frame copy queue.
        lkSchedulers.unlock();
    }
}

/******************************************************************************
 * @brief This method will run in a thread pool. It will be called by the main thread
 *      and will run the code within the PooledLinearCode() method. This is meant to be
 *      used as an internal utility of the child class to further improve parallelization.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
void ObjectDetector::PooledLinearCode()
{
    /////////////////////////////
    //  Detection Overlay Frame queue.
    /////////////////////////////
    // Acquire sole writing access to the detectedObjectCopySchedule.
    std::unique_lock<std::shared_mutex> lkObjectOverlayFrameQueue(m_muFrameCopyMutex);
    // Check if there are unfulfilled requests.
    if (!m_qDetectedObjectDrawnOverlayFramesCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::FrameFetchContainer<cv::Mat> stContainer = m_qDetectedObjectDrawnOverlayFramesCopySchedule.front();
        // Pop out of queue.
        m_qDetectedObjectDrawnOverlayFramesCopySchedule.pop();
        // Release lock.
        lkObjectOverlayFrameQueue.unlock();

        // Check which frame we should copy.
        switch (stContainer.eFrameType)
        {
            case PIXEL_FORMATS::eObjectDetection: *stContainer.pFrame = m_cvTorchOverlayFrame.clone(); break;
            default: *stContainer.pFrame = m_cvTorchOverlayFrame.clone(); break;
        }

        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedFrameStatus->set_value(true);
    }

    /////////////////////////////
    //  Object queue.
    /////////////////////////////
    // Acquire sole writing access to the detectedObjectCopySchedule.
    std::unique_lock<std::shared_mutex> lkObjectQueue(m_muArucoDataCopyMutex);
    // Check if there are unfulfilled requests.
    if (!m_qDetectedObjectCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::DataFetchContainer<std::vector<objectdetectutils::Object>> stContainer = m_qDetectedObjectCopySchedule.front();
        // Pop out of queue.
        m_qDetectedObjectCopySchedule.pop();
        // Release lock.
        lkObjectQueue.unlock();

        // Copy the detected objects to the target location
        *stContainer.pData = m_vDetectedObjects;

        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedDataStatus->set_value(true);
    }
}

/******************************************************************************
 * @brief Request a copy of the frame containing the detected objects from all
 *      detection methods drawn onto the frame.
 *
 * @param cvFrame - The cv::Mat frame to copy the detection overlay image to.
 * @return std::future<bool> - The future that will be set to true when the frame is copied.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
std::future<bool> ObjectDetector::RequestDetectionOverlayFrame(cv::Mat& cvFrame)
{
    // Assemble the DataFetchContainer.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvFrame, PIXEL_FORMATS::eObjectDetection);

    // Acquire lock on pool copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qDetectedObjectDrawnOverlayFramesCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Request a copy of the most update to date vector of the detected objects
 *    from all detection methods.
 *
 * @param vObjects - The vector of detected objects to copy the detected objects to.
 * @return std::future<bool> - The future that will be set to true when the objects are copied.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
std::future<bool> ObjectDetector::RequestDetectedObjects(std::vector<objectdetectutils::Object>& vObjects)
{
    // Assemble the DataFetchContainer.
    containers::DataFetchContainer<std::vector<objectdetectutils::Object>> stContainer(vObjects);

    // Acquire lock on pool copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qDetectedObjectCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedDataStatus->get_future();
}

/******************************************************************************
 * @brief Initialize the PyTorch interpreter for object detection.
 *
 * @param szModelPath - The path to the PyTorch model file.
 * @param eDevice - The hardware device to use for inference (e.g., CPU or GPU).
 * @return true - Model was opened and loaded successfully onto the torch device.
 * @return false - Model was not opened and loaded successfully onto the torch device.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
bool ObjectDetector::InitTorchDetection(const std::string& szModelPath, yolomodel::pytorch::PyTorchInterpreter::HardwareDevices eDevice)
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
        LOG_ERROR(logging::g_qSharedLogger, "Unable to initialize Torch detection for ObjectDetector.");
        // Update member variable.
        m_bTorchInitialized = false;
        // Return status.
        return false;
    }
}

/******************************************************************************
 * @brief Enable the PyTorch detection method for this ObjectDetector.
 *
 * @param fMinObjectConfidence - The minimum confidence threshold for detected objects.
 * @param fNMSThreshold - The non-maximum suppression threshold for filtering overlapping detections.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
void ObjectDetector::EnableTorchDetection(const float fMinObjectConfidence, const float fNMSThreshold)
{
    // Update member variables.
    m_fTorchMinObjectConfidence = fMinObjectConfidence;
    m_fTorchNMSThreshold        = fNMSThreshold;

    // Check if torch model has been initialized.
    if (m_bTorchInitialized)
    {
        // Update member variable.
        m_bTorchEnabled = true;
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Tried to enable torch detection for ObjectDetector but it has not been initialized yet!");
        // Update member variable.
        m_bTorchEnabled = false;
    }
}

/******************************************************************************
 * @brief Set the flag to enable or disable object detection with the torch model.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
void ObjectDetector::DisableTorchDetection()
{
    // Update member variable.
    m_bTorchEnabled = false;
}

/******************************************************************************
 * @brief Set the max FPS of the detector.
 *
 * @param nRecordingFPS - The max FPS of the detector.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
void ObjectDetector::SetDetectorMaxFPS(const int nRecordingFPS)
{
    // Set the max iterations per second of the main thread.
    this->SetMainThreadIPSLimit(nRecordingFPS);
}

/******************************************************************************
 * @brief Set the flag to enable or disable recording of the overlay output.
 *
 * @param bEnableRecordingFlag - The flag to enable or disable recording of the overlay output.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
void ObjectDetector::SetEnableRecordingFlag(const bool bEnableRecordingFlag)
{
    // Update member variable.
    m_bEnableRecordingFlag = bEnableRecordingFlag;
}

/******************************************************************************
 * @brief Check if the ObjectDetector is ready to be used.
 *
 * @return true - The ObjectDetector is ready to be used.
 * @return false - The ObjectDetector is not ready to be used.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
bool ObjectDetector::GetIsReady()
{
    // Create instance variables.
    bool bDetectorIsReady = false;

    // Check if this detectors thread is currently running.
    if (this->GetThreadState() == AutonomyThreadState::eRunning)
    {
        // Check if using ZEDCam or BasicCam.
        if (m_bUsingZedCamera)
        {
            // Check if camera is NOT open.
            if (std::dynamic_pointer_cast<ZEDCamera>(m_pCamera)->GetCameraIsOpen())
            {
                // Set camera opened toggle.
                bDetectorIsReady = true;
            }
        }
        else
        {
            // Check if camera is NOT open.
            if (std::dynamic_pointer_cast<BasicCamera>(m_pCamera)->GetCameraIsOpen())
            {
                // Set camera opened toggle.
                bDetectorIsReady = true;
            }
        }
    }

    // Return if this detector is ready or not.
    return bDetectorIsReady;
}

/******************************************************************************
 * @brief Get the max FPS of the detector.
 *
 * @return int - The max FPS of the detector.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
int ObjectDetector::GetDetectorMaxFPS() const
{
    // Return the max FPS of the detector.
    return this->GetMainThreadMaxIPS();
}

/******************************************************************************
 * @brief Get the flag to enable or disable recording of the overlay output.
 *
 * @return true - The flag to enable or disable recording of the overlay output.
 * @return false - The flag to enable or disable recording of the overlay output.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
bool ObjectDetector::GetEnableRecordingFlag() const
{
    // Return the enable recording flag.
    return m_bEnableRecordingFlag;
}

/******************************************************************************
 * @brief Get the camera name.
 *
 * @return std::string - The camera name.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
std::string ObjectDetector::GetCameraName()
{
    // Return the camera name.
    return m_szCameraName;
}

/******************************************************************************
 * @brief Get the process frame resolution.
 *
 * @return cv::Size - The process frame resolution.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
cv::Size ObjectDetector::GetProcessFrameResolution() const
{
    // Check if using a ZED camera.
    if (m_bUsingZedCamera)
    {
        // Concatenate camera model name and serial number.
        return std::dynamic_pointer_cast<ZEDCamera>(m_pCamera)->GetPropResolution();
    }
    else
    {
        // Concatenate camera path or index.
        return std::dynamic_pointer_cast<BasicCamera>(m_pCamera)->GetPropResolution();
    }
}

/******************************************************************************
 * @brief Update the detected objects with the newly detected objects.
 *
 * @param vNewlyDetectedObjects - The vector of newly detected objects to update the detected objects with.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
void ObjectDetector::UpdateDetectedObjects(std::vector<objectdetectutils::Object>& vNewlyDetectedObjects)
{
    // Check if tracking is enabled.
    if (m_bEnableTracking)
    {
        // Check if the given object vector is empty
        if (vNewlyDetectedObjects.empty())
        {
            // Since the objects are empty that means the detector has not detected any new ground truth objects.
            // In this case we will fallback to relying on the multi-tracker to track the objects and just update the objects
            // stored in the m_vDetectedObjects vector.
            // This is necessary because the torch detector is not perfect and may not detect all objects in the frame
            // and it doesn't have the ability to track objects over time.
            // We will use the multi-tracker to track the objects over time and update the bounding box data for the objects.

            // Update the multi-tracker with the current frame.
            m_pMultiTracker->Update(m_cvFrame);
        }
        else
        {
            // Loop through the newly detected objects.
            for (objectdetectutils::Object& stObject : vNewlyDetectedObjects)
            {
                // Add the newly detected objects to the multi-tracker.
                bool bMatchedObjectToExistingTracker = m_pMultiTracker->InitTracker(m_cvFrame, stObject.pBoundingBox, constants::BBOX_TRACKER_TYPE);
                // Check if the object was matched to an existing tracker.
                if (!bMatchedObjectToExistingTracker)
                {
                    // Add the new object to the member variable list.
                    m_vDetectedObjects.emplace_back(stObject);
                }
                else
                {
                    // Find the object with the same bounding box pointer and update the ID and confidence.
                    for (objectdetectutils::Object& stExistingObject : m_vDetectedObjects)
                    {
                        // Check if the bounding box pointers are the same.
                        if (stObject.pBoundingBox == stExistingObject.pBoundingBox)
                        {
                            // Update the ID and confidence of the existing object.
                            stExistingObject.dConfidence = stObject.dConfidence;
                        }
                    }
                }
            }

            // Update the multi-tracker with the current frame.
            m_pMultiTracker->Update(m_cvFrame);
        }

        // Loop through the detected objects and check if there are any we need to remove, and also update the time last seen.
        for (std::vector<objectdetectutils::Object>::iterator itObject = m_vDetectedObjects.begin(); itObject != m_vDetectedObjects.end();)
        {
            // Check if the bounding box is 0,0,0,0.
            if (itObject->pBoundingBox->x == 0 && itObject->pBoundingBox->y == 0 && itObject->pBoundingBox->width == 0 && itObject->pBoundingBox->height == 0)
            {
                // Remove the object from the vector.
                itObject = m_vDetectedObjects.erase(itObject);
            }
            else
            {
                ++itObject;
            }
        }
    }
    else
    {
        // If tracking is not enabled, we will just clear the detected objects and add the new ones.
        m_vDetectedObjects.clear();
        // Loop through the newly detected objects and add them to the detected objects vector.
        for (objectdetectutils::Object& stObject : vNewlyDetectedObjects)
        {
            // Set the object creation time to 0. The objects aren't being tracked, so we can't really tell their age.
            stObject.tmCreation = std::chrono::system_clock::time_point::min();

            // Add the new object to the member variable list.
            m_vDetectedObjects.emplace_back(stObject);
        }
    }

    // Check if we are using a ZED camera.
    if (m_bUsingZedCamera)
    {
        // Check if the point cloud is empty.
        if (!m_cvPointCloud.empty())
        {
            // Get the rover pose from the waypoint handler.
            m_stRoverPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();

            // Find the camera's position in UTM coordinates by applying the camera offset to the rover pose.
            geoops::UTMCoordinate stCamera = m_stRoverPose.GetUTMCoordinate();
            stCamera.dEasting += m_pCamera->GetCameraPoseOffset().dPosX;
            stCamera.dNorthing += m_pCamera->GetCameraPoseOffset().dPosY;
            stCamera.dAltitude += m_pCamera->GetCameraPoseOffset().dPosZ;
            // Update the rover pose's heading to match the camera's heading. We will need to calculate the camera's heading using the quaternion.
            double dSinYCosP      = 2.0 * (m_pCamera->GetCameraPoseOffset().dQW * m_pCamera->GetCameraPoseOffset().dQZ +
                                      m_pCamera->GetCameraPoseOffset().dQX * m_pCamera->GetCameraPoseOffset().dQY);
            double dCosYCosP      = 1.0 - 2.0 * (m_pCamera->GetCameraPoseOffset().dQY * m_pCamera->GetCameraPoseOffset().dQY +
                                            m_pCamera->GetCameraPoseOffset().dQZ * m_pCamera->GetCameraPoseOffset().dQZ);
            double dCameraHeading = std::atan2(dSinYCosP, dCosYCosP) * (180.0 / CV_PI);
            // Recreate the rover pose with the camera's adjusted position and heading.
            geoops::RoverPose stCameraPose = geoops::RoverPose(stCamera, dCameraHeading);

            // Loop through the objects and use their center point to lookup their distance in the point cloud.
            for (objectdetectutils::Object& stObject : m_vDetectedObjects)
            {
                // Use either width of height for the neighborhood size.
                int nNeighborhoodSize = std::min(stObject.pBoundingBox->width, stObject.pBoundingBox->height);
                // Geolocate the object in the point cloud.
                stObject.stGeolocatedPosition = geoloc::GeolocateBox(
                    m_cvPointCloud,
                    stCameraPose,
                    cv::Point(stObject.pBoundingBox->x + stObject.pBoundingBox->width / 2, stObject.pBoundingBox->y + stObject.pBoundingBox->height / 2),
                    nNeighborhoodSize);

                // Since this is a object detection, set the object's waypoint type appropriately.
                stObject.stGeolocatedPosition.eType = geoops::WaypointType::eObjectWaypoint;
                // Depending on the class name of the model, set the object type.
                if (stObject.szClassName == "mallet")
                {
                    stObject.eDetectionType = objectdetectutils::ObjectDetectionType::eMallet;
                }
                else if (stObject.szClassName == "bottle")
                {
                    stObject.eDetectionType = objectdetectutils::ObjectDetectionType::eWaterBottle;
                }
                else if (stObject.szClassName == "pick")
                {
                    stObject.eDetectionType = objectdetectutils::ObjectDetectionType::eRockPick;
                }

                // Calculate the geo measurement and print the distance to the object.
                geoops::GeoMeasurement stMeasurement =
                    geoops::CalculateGeoMeasurement(m_stRoverPose.GetUTMCoordinate(), stObject.stGeolocatedPosition.GetUTMCoordinate());
                // Set the straight line distance to the object.
                stObject.dStraightLineDistance = stMeasurement.dDistanceMeters;
                // Use the rover heading and the azimuth angle to calculate the relative heading to the object.
                stObject.dYawAngle = numops::AngularDifference(m_stRoverPose.GetCompassHeading(), stMeasurement.dStartRelativeBearing);
            }
        }
    }
    else
    {
        // Estimate the positions of the objects using some basic trig.
        for (objectdetectutils::Object& stObject : m_vDetectedObjects)
        {
            // Use some trig to get the location of the object.
            objectdetectutils::EstimatePoseFromCameraFrame(stObject);
        }
    }
}
