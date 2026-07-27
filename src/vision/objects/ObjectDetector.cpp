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
 * @brief Register this detector's demand for the camera data it consumes. Runs
 *      exactly once, on the first loop iteration. Holding these subscriptions for
 *      the detector's lifetime is what tells the camera to keep retrieving and
 *      publishing these data types; a type nobody subscribes to is never retrieved.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void ObjectDetector::EnsureCameraSubscriptions()
{
    // Subscribe exactly once, no matter how many times the loop runs.
    std::call_once(m_ocCameraSubscribeOnce,
                   [this]()
                   {
                       // Check whether we are consuming from a ZED camera or a basic camera.
                       if (m_bUsingZedCamera)
                       {
                           // Dynamic cast so we can reach the ZED specific publishers.
                           std::shared_ptr<ZEDCamera> pZEDCamera = std::dynamic_pointer_cast<ZEDCamera>(m_pCamera);
                           // Subscribe to whichever memory channel this detector was configured for.
                           if (m_bUsingGpuMats)
                           {
                               // Express demand on the GPU channels.
                               m_subCameraFrame      = pZEDCamera->GetFrameGPUPublisher().Subscribe();
                               m_subCameraPointCloud = pZEDCamera->GetPointCloudGPUPublisher().Subscribe();
                           }
                           else
                           {
                               // Express demand on the CPU channels.
                               m_subCameraFrame      = pZEDCamera->GetFrameCPUPublisher().Subscribe();
                               m_subCameraPointCloud = pZEDCamera->GetPointCloudCPUPublisher().Subscribe();
                           }
                       }
                       else
                       {
                           // Basic cameras publish a single BGRA frame channel and no point cloud.
                           m_subCameraFrame = std::dynamic_pointer_cast<BasicCamera>(m_pCamera)->GetFramePublisher().Subscribe();
                       }
                   });
}

/******************************************************************************
 * @brief Load the newest published camera snapshots into this detector's working
 *      frames. Each snapshot is loaded once into a local, so the data cannot change
 *      underneath us while we read it. If the camera has not published a new frame
 *      since the last detection pass, nothing is copied and false is returned so the
 *      caller can skip an entire redundant pass.
 *
 * @return true - New frame data was loaded and detection should run.
 * @return false - Nothing new to process; skip this detection pass.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
bool ObjectDetector::LoadLatestCameraFrames()
{
    // The sequence number of the frame snapshot we are about to process.
    unsigned long long ullFrameSequence = 0;

    // Check whether we are consuming from a ZED camera or a basic camera.
    if (m_bUsingZedCamera)
    {
        // Dynamic cast so we can reach the ZED specific publishers.
        std::shared_ptr<ZEDCamera> pZEDCamera = std::dynamic_pointer_cast<ZEDCamera>(m_pCamera);

        // Check if the ZED camera is returning cv::cuda::GpuMat or cv::Mat.
        if (m_bUsingGpuMats)
        {
            // Load both GPU snapshots once into locals.
            pubsub::Publisher<cv::cuda::GpuMat>::SharedSnapshot pFrameSnapshot = pZEDCamera->GetFrameGPUPublisher().Get();
            pubsub::Publisher<cv::cuda::GpuMat>::SharedSnapshot pCloudSnapshot = pZEDCamera->GetPointCloudGPUPublisher().Get();
            // Nothing has been published yet.
            if (pFrameSnapshot == nullptr || pCloudSnapshot == nullptr)
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "ObjectDetector unable to get point cloud or frame from ZEDCam!");
                return false;
            }
            // Skip the pass entirely if the camera has not published a new frame.
            ullFrameSequence = pFrameSnapshot->ullSequence;
            if (ullFrameSequence == m_ullLastProcessedFrameSequence)
            {
                // Count the skip so the short circuit can be verified, then bail out.
                m_ullSkippedFrameCount.fetch_add(1, std::memory_order_relaxed);
                return false;
            }
            // Download mats from GPU memory. Done here, on this thread, off the camera's critical path.
            pFrameSnapshot->tData.download(m_cvFrame);
            pCloudSnapshot->tData.download(m_cvPointCloud);
            // Drop alpha channel.
            cv::cvtColor(m_cvFrame, m_cvFrame, cv::COLOR_BGRA2BGR);
        }
        else
        {
            // Load both CPU snapshots once into locals.
            pubsub::Publisher<cv::Mat>::SharedSnapshot pFrameSnapshot = pZEDCamera->GetFrameCPUPublisher().Get();
            pubsub::Publisher<cv::Mat>::SharedSnapshot pCloudSnapshot = pZEDCamera->GetPointCloudCPUPublisher().Get();
            // Nothing has been published yet.
            if (pFrameSnapshot == nullptr || pCloudSnapshot == nullptr)
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "ObjectDetector unable to get point cloud or regular frame from ZEDCam!");
                return false;
            }
            // Skip the pass entirely if the camera has not published a new frame.
            ullFrameSequence = pFrameSnapshot->ullSequence;
            if (ullFrameSequence == m_ullLastProcessedFrameSequence)
            {
                // Count the skip so the short circuit can be verified, then bail out.
                m_ullSkippedFrameCount.fetch_add(1, std::memory_order_relaxed);
                return false;
            }
            // Copy the immutable snapshots into our working frames.
            pFrameSnapshot->tData.copyTo(m_cvFrame);
            pCloudSnapshot->tData.copyTo(m_cvPointCloud);
        }
    }
    else
    {
        // Load the basic camera's frame snapshot once into a local.
        pubsub::Publisher<cv::Mat>::SharedSnapshot pFrameSnapshot = std::dynamic_pointer_cast<BasicCamera>(m_pCamera)->GetFramePublisher().Get();
        // Nothing has been published yet.
        if (pFrameSnapshot == nullptr)
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "ObjectDetector unable to get RGB image from BasicCam!");
            return false;
        }
        // Skip the pass entirely if the camera has not published a new frame.
        ullFrameSequence = pFrameSnapshot->ullSequence;
        if (ullFrameSequence == m_ullLastProcessedFrameSequence)
        {
            // Count the skip so the short circuit can be verified, then bail out.
            m_ullSkippedFrameCount.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        // Copy the immutable snapshot into our working frame.
        pFrameSnapshot->tData.copyTo(m_cvFrame);
    }

    // Remember which frame we processed so the next pass can detect a repeat.
    m_ullLastProcessedFrameSequence = ullFrameSequence;

    // New data was loaded; detection should run.
    return true;
}

/******************************************************************************
 * @brief This method will run continuously in a separate thread. New frames from
 *      the given camera are grabbed and the objects for the camera image are detected
 *      using the PyTorch interpreter. The detected objects are then filtered and stored,
 *      then published for consumers to read on their own schedule.
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

            // This thread NEVER stops itself because its camera is not ready. It idles until the
            // camera reports open, so startup ordering does not matter and a camera that connects
            // late (or reconnects after a dropout) is picked up automatically. Only RequestStop()
            // from the owner ends this thread.
            if (m_bLastKnownCameraOpenState)
            {
                // Remember the new state so we log the transition exactly once.
                m_bLastKnownCameraOpenState = false;

                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "ObjectDetector for ZED camera with serial number {} is waiting for its camera to open. Detection is paused until it does.",
                            std::dynamic_pointer_cast<ZEDCamera>(m_pCamera)->GetCameraSerial());
            }
        }
        else
        {
            // Set camera opened toggle.
            m_bCameraIsOpened = true;

            // Log the not-ready -> ready transition exactly once.
            if (!m_bLastKnownCameraOpenState)
            {
                // Remember the new state so the recovery is reported a single time.
                m_bLastKnownCameraOpenState = true;

                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger,
                         "ObjectDetector for ZED camera with serial number {} now has an open camera. Resuming detection.",
                         std::dynamic_pointer_cast<ZEDCamera>(m_pCamera)->GetCameraSerial());
            }
        }
    }
    else
    {
        // Check if camera is NOT open.
        if (!std::dynamic_pointer_cast<BasicCamera>(m_pCamera)->GetCameraIsOpen())
        {
            // Set camera opened toggle.
            m_bCameraIsOpened = false;

            // This thread NEVER stops itself because its camera is not ready. It idles until the
            // camera reports open, so startup ordering does not matter and a camera that connects
            // late (or reconnects after a dropout) is picked up automatically. Only RequestStop()
            // from the owner ends this thread.
            if (m_bLastKnownCameraOpenState)
            {
                // Remember the new state so we log the transition exactly once.
                m_bLastKnownCameraOpenState = false;

                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "ObjectDetector for BasicCam at {} is waiting for its camera to open. Detection is paused until it does.",
                            std::dynamic_pointer_cast<BasicCamera>(m_pCamera)->GetCameraLocation());
            }
        }
        else
        {
            // Set camera opened toggle.
            m_bCameraIsOpened = true;

            // Log the not-ready -> ready transition exactly once.
            if (!m_bLastKnownCameraOpenState)
            {
                // Remember the new state so the recovery is reported a single time.
                m_bLastKnownCameraOpenState = true;

                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger,
                         "ObjectDetector for BasicCam at {} now has an open camera. Resuming detection.",
                         std::dynamic_pointer_cast<BasicCamera>(m_pCamera)->GetCameraLocation());
            }
        }
    }

    // Check if camera is opened.
    if (m_bCameraIsOpened)
    {
        // Register demand for the camera data we consume (once, on the first iteration).
        this->EnsureCameraSubscriptions();

        // Load the newest published camera snapshots. This is a lock-free read that never blocks
        // on the camera's loop. Returns false when there is nothing new to process, in which case
        // we skip this entire detection pass rather than redoing work on an identical frame.
        if (!this->LoadLatestCameraFrames())
        {
            // Nothing new from the camera; skip this pass.
            return;
        }

        // If the thread is shutting down, break out of the loop gracefully
        if (this->GetThreadState() != AutonomyThreadState::eRunning)
        {
            return;
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
        m_cvDetectionOverlayFrame = m_cvFrame.clone();
        m_cvTorchProcFrame        = m_cvFrame.clone();
        // Copy the camera frame to the pre-processing frame and overlay frame.
        cv::cvtColor(m_cvTorchProcFrame, m_cvTorchProcFrame, cv::COLOR_BGR2RGB);

        // Check if torch detection if turned on.
        if (m_bTorchEnabled)
        {
            // Atomically load the model shared_ptr into a local so a concurrent InitTorchDetection()
            // swap can't invalidate it mid-inference (the local keeps the old model alive).
            std::shared_ptr<yolomodel::pytorch::PyTorchInterpreter> pTorchDetector = std::atomic_load_explicit(&m_pTorchDetector, std::memory_order_acquire);
            if (pTorchDetector != nullptr)
            {
                // Detect objects in the image.
                std::vector<objectdetectutils::Object> vNewTorchObjects =
                    torchobject::Detect(m_cvTorchProcFrame, *pTorchDetector, m_fTorchMinObjectConfidence, m_fTorchNMSThreshold);

                // Add Torch objects to the list of newly detected objects.
                m_vNewlyDetectedObjects.insert(m_vNewlyDetectedObjects.end(), vNewTorchObjects.begin(), vNewTorchObjects.end());
            }
        }

        // Set the FOV of the camera in the object structs for this detector's camera.
        for (objectdetectutils::Object& stObject : m_vNewlyDetectedObjects)
        {
            // Set the UUID of the detector that detected this object to this ObjectDetector's camera name so we can associate it with this detector.
            stObject.szDetectorUUID = this->GetThreadUUID();
            // Set object FOV parameter to this object detectors camera's FOV.
            stObject.dHorizontalFOV = m_pCamera->GetPropHorizontalFOV();
        }

        // Merge the newly detected objects with the pre-existing detected objects.
        this->UpdateDetectedObjects(m_vNewlyDetectedObjects);

        // Draw object overlays onto normal image.
        torchobject::DrawDetections(m_cvDetectionOverlayFrame, m_vDetectedObjects);

        // Check if the detected objects vector is not empty.
        if (!m_vDetectedObjects.empty())
        {
            // It's not empty so we should have a valid overlay frame with detections drawn on it.
            m_cvLastGoodOverlayFrame = m_cvDetectionOverlayFrame.clone();
        }
        /////////////////////////////////////////////////////////////////////////////////////

        // Publish the freshly computed outputs to any subscribed consumers (deep copy once each).
        // Detection overlay frame.
        if (m_pubDetectionOverlay.HasSubscribers())
        {
            // Deep copy the overlay into a pooled snapshot and publish.
            std::shared_ptr<pubsub::Snapshot<cv::Mat>> pSlot = m_pubDetectionOverlay.Acquire();
            m_cvDetectionOverlayFrame.copyTo(pSlot->tData);
            m_pubDetectionOverlay.Publish(std::move(pSlot));
        }
        // Last good detection overlay frame.
        if (m_pubLastGoodOverlay.HasSubscribers())
        {
            // Deep copy the last-good overlay into a pooled snapshot and publish.
            std::shared_ptr<pubsub::Snapshot<cv::Mat>> pSlot = m_pubLastGoodOverlay.Acquire();
            m_cvLastGoodOverlayFrame.copyTo(pSlot->tData);
            m_pubLastGoodOverlay.Publish(std::move(pSlot));
        }
        // Detected objects. Published unconditionally rather than gated on demand: the objects are
        // already computed by the pass above, so publishing costs only a small vector copy, and
        // consumers reached through free functions (statemachine::LoadDetectedObjects) have nowhere
        // natural to hold a Subscription.
        {
            // Copy the objects, then deep-copy each shared bounding box so the published snapshot is
            // truly immutable (the tracker also holds and mutates those bounding boxes).
            std::shared_ptr<pubsub::Snapshot<std::vector<objectdetectutils::Object>>> pSlot = m_pubDetectedObjects.Acquire();
            pSlot->tData                                                                    = m_vDetectedObjects;
            for (objectdetectutils::Object& stObject : pSlot->tData)
            {
                // Give this snapshot its own bounding box instance.
                stObject.pBoundingBox = std::make_shared<cv::Rect2d>(*stObject.pBoundingBox);
            }
            m_pubDetectedObjects.Publish(std::move(pSlot));
        }
    }
}

/******************************************************************************
 * @brief Not used. Detected objects and overlay frames are handed to consumers by the
 *      publish-latest mechanism in ThreadedContinuousCode(); no per-consumer fan-out
 *      work remains.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void ObjectDetector::PooledLinearCode() {}

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
    std::shared_ptr<yolomodel::pytorch::PyTorchInterpreter> pNewDetector = std::make_shared<yolomodel::pytorch::PyTorchInterpreter>(szModelPath, eDevice);

    // Check if device/model was opened without issue.
    if (pNewDetector->IsReadyForInference())
    {
        // Atomically publish the ready model so the producer thread sees it without a race.
        std::atomic_store_explicit(&m_pTorchDetector, pNewDetector, std::memory_order_release);
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

            // Translate the camera's local offsets into global Easting/Northing.
            double dRoverHeadingRad = m_stRoverPose.GetCompassHeading() * (CV_PI / 180.0);
            double dRotatedX        = (m_pCamera->GetCameraPoseOffset().dPosY * sin(dRoverHeadingRad)) + (m_pCamera->GetCameraPoseOffset().dPosX * cos(dRoverHeadingRad));
            double dRotatedY        = (m_pCamera->GetCameraPoseOffset().dPosY * cos(dRoverHeadingRad)) - (m_pCamera->GetCameraPoseOffset().dPosX * sin(dRoverHeadingRad));

            // Apply the rotated offsets to the global coordinates.
            stCamera.dEasting += dRotatedX;
            stCamera.dNorthing += dRotatedY;
            stCamera.dAltitude += m_pCamera->GetCameraPoseOffset().dPosZ;

            // Creating variables for the camera pose offset values.
            double dQW = m_pCamera->GetCameraPoseOffset().dQW;
            double dQX = m_pCamera->GetCameraPoseOffset().dQX;
            double dQY = m_pCamera->GetCameraPoseOffset().dQY;
            double dQZ = m_pCamera->GetCameraPoseOffset().dQZ;

            // Update the rover pose's heading to match the camera's heading. We will need to calculate the camera's heading using the quaternion.
            double dSinYCosP      = 2.0 * (dQW * dQZ + dQX * dQY);
            double dCosYCosP      = 1.0 - 2.0 * (dQY * dQY + dQZ * dQZ);
            double dCameraHeading = std::atan2(dSinYCosP, dCosYCosP) * (180.0 / CV_PI);

            // Add the relative camera heading to the absolute rover heading
            double dAbsoluteCameraHeading = m_stRoverPose.GetCompassHeading() + dCameraHeading;

            // Recreate the rover pose with the camera's adjusted position and heading.
            geoops::RoverPose stCameraPose = geoops::RoverPose(stCamera, dAbsoluteCameraHeading);

            // Loop through the objects and use their center point to lookup their distance in the point cloud.
            for (objectdetectutils::Object& stObject : m_vDetectedObjects)
            {
                // Use either width of height for the neighborhood size.
                int nNeighborhoodSize = std::min(stObject.pBoundingBox->width, stObject.pBoundingBox->height);
                // Geolocate the object in the point cloud.
                geoops::Waypoint stGeolocation = geoloc::GeolocateBox(
                    m_cvPointCloud,
                    stCameraPose,
                    cv::Point(stObject.pBoundingBox->x + stObject.pBoundingBox->width / 2, stObject.pBoundingBox->y + stObject.pBoundingBox->height / 2),
                    nNeighborhoodSize);

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

                // Calculate the yaw angle to the tag using the center point of the tag and the camera's field of view.
                // This is a fallback in case the geolocation fails for some reason, we can still provide a relative angle to the tag.
                // Get the center X pixel coordinate of the object's bounding box.
                double dObjectCenterX = stObject.pBoundingBox->x + (stObject.pBoundingBox->width / 2.0);
                // Get the center X pixel coordinate of the camera frame.
                double dFrameCenterX = m_cvFrame.cols / 2.0;
                // Calculate the offset in pixels from the center of the camera frame.
                // (Positive offset = target is to the right, Negative = target is to the left)
                double dPixelOffsetX = dObjectCenterX - dFrameCenterX;
                // Calculate how many real-world degrees each pixel represents.
                double dDegreesPerPixel = stObject.dHorizontalFOV / static_cast<double>(m_cvFrame.cols);
                // Multiply the pixel offset by the degrees per pixel to get the relative yaw angle.
                stObject.dYawAngle = dPixelOffsetX * dDegreesPerPixel;
                // Explicitly set distance to 0.0 so the autonomy state machines know the depth map failed
                // and will properly fall back to using this calculated dYawAngle.
                stObject.dStraightLineDistance = 0.0;

                // Check if the geolocation is valid. If it is overwrite the yaw angle and distance with the geolocation data.
                if (stGeolocation != geoops::Waypoint())
                {
                    // Since this is a object detection, set the object's waypoint type appropriately.
                    stGeolocation.eType = geoops::WaypointType::eObjectWaypoint;
                    // Calculate the geo measurement and print the distance to the object.
                    geoops::GeoMeasurement stMeasurement =
                        geoops::CalculateGeoMeasurement(m_stRoverPose.GetUTMCoordinate(), stObject.stGeolocatedPosition.GetUTMCoordinate());

                    // Check that the distance is in a reasonable range.
                    if (stMeasurement.dDistanceMeters > 0.0 && stMeasurement.dDistanceMeters < 25.0)
                    {
                        // Set the object's geolocation.
                        stObject.stGeolocatedPosition = stGeolocation;
                        // Use the rover heading and the azimuth angle to calculate the relative heading to the object.
                        stObject.dYawAngle = numops::AngularDifference(m_stRoverPose.GetCompassHeading(), stMeasurement.dStartRelativeBearing);
                        // Set the straight line distance to the object.
                        stObject.dStraightLineDistance = stMeasurement.dDistanceMeters;
                    }
                }
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
