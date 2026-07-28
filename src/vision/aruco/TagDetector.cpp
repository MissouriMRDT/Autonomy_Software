/******************************************************************************
 * @brief Implements the TagDetector class.
 *
 * @file TagDetector.cpp
 * @author clayjay3 (claytonraycowen@gmail.com), jspencerpittman (jspencerpittman@gmail.com)
 * @date 2023-10-07
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "TagDetector.h"
#include "../../AutonomyGlobals.h"
#include "../../util/vision/Geolocate.hpp"
#include "./ArucoDetection.hpp"
#include "./TorchTagDetection.hpp"
#include <tracy/Tracy.hpp>

/******************************************************************************
 * @brief Construct a new TagDetector object.
 *
 * @param pBasicCam - A pointer to the BasicCam camera to get frames from for detection.
 * @param nArucoCornerRefinementMaxIterations - The number of iterations to use when refining marker corners.
 * @param nArucoCornerRefinementMethod - The refinement method to use.
 * @param nArucoMarkerBorderBits - The number of border unit squares around the marker.
 * @param bArucoDetectInvertedMarkers - Enable or disable upside-down marker detection.
 * @param bUseAruco3Detection - Whether or not to use the newer/faster method of detection. Experimental.
 * @param bEnableTracking - Whether or not to enable tracking of detected tags.
 * @param nDetectorMaxFPS - The max FPS limit the detector can run at.
 * @param bEnableRecordingFlag - Whether or not this TagDetector's overlay output should be recorded.
 * @param nNumDetectedTagsRetrievalThreads - The number of threads to use when fulfilling
 *                                           requests for the detected aruco tags. Default is 5.
 * @param bUsingGpuMats - Whether or not the given camera name will be using GpuMats.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-10
 ******************************************************************************/
TagDetector::TagDetector(std::shared_ptr<BasicCamera> pBasicCam,
                         const int nArucoCornerRefinementMaxIterations,
                         const int nArucoCornerRefinementMethod,
                         const int nArucoMarkerBorderBits,
                         const bool bArucoDetectInvertedMarkers,
                         const bool bUseAruco3Detection,
                         const bool bEnableTracking,
                         const int nDetectorMaxFPS,
                         const bool bEnableRecordingFlag,
                         const int nNumDetectedTagsRetrievalThreads,
                         const bool bUsingGpuMats)

{
    // Initialize member variables.
    m_pCamera                          = pBasicCam;
    m_bTorchInitialized                = false;
    m_bTorchEnabled                    = false;
    m_bEnableTracking                  = bEnableTracking;
    m_bUsingZedCamera                  = false;    // Toggle ZED functions off.
    m_bUsingGpuMats                    = bUsingGpuMats;
    m_bCameraIsOpened                  = false;
    m_nNumDetectedTagsRetrievalThreads = nNumDetectedTagsRetrievalThreads;
    m_szCameraName                     = pBasicCam->GetCameraLocation();
    m_bEnableRecordingFlag             = bEnableRecordingFlag;
    m_stRoverPose                      = geoops::RoverPose();

    // Setup aruco detector params.
    m_cvArucoDetectionParams                               = cv::aruco::DetectorParameters();
    m_cvArucoDetectionParams.cornerRefinementMaxIterations = nArucoCornerRefinementMaxIterations;
    m_cvArucoDetectionParams.cornerRefinementMethod        = nArucoCornerRefinementMethod;
    m_cvArucoDetectionParams.markerBorderBits              = nArucoMarkerBorderBits;
    m_cvArucoDetectionParams.detectInvertedMarker          = bArucoDetectInvertedMarkers;
    m_cvArucoDetectionParams.useAruco3Detection            = bUseAruco3Detection;
    // Get aruco dictionary and initialize aruco detector.
    m_cvTagDictionary = cv::aruco::getPredefinedDictionary(constants::ARUCO_DICTIONARY);
    m_cvArucoDetector = cv::aruco::ArucoDetector(m_cvTagDictionary, m_cvArucoDetectionParams);

    // Create a multi-tracker for tracking multiple tags from the torch detectors.
    m_pMultiTracker = std::make_shared<tracking::MultiTracker>(constants::BBOX_TRACKER_LOST_TIMEOUT,
                                                               constants::BBOX_TRACKER_MAX_TRACK_TIME,
                                                               constants::BBOX_TRACKER_IOU_MATCH_THRESHOLD);

    // Set max IPS of main thread.
    this->SetMainThreadIPSLimit(nDetectorMaxFPS);

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "TagDetector created for camera at path/index: {}", m_szCameraName);
}

/******************************************************************************
 * @brief Construct a new TagDetector object.
 *
 * @param pZEDCam - A pointer to the ZEDCam camera to get frames from for detection. Override for ZED camera.
 * @param nArucoCornerRefinementMaxIterations - The number of iterations to use when refining marker corners.
 * @param nArucoCornerRefinementMethod - The refinement method to use.
 * @param nArucoMarkerBorderBits - The number of border unit squares around the marker.
 * @param bArucoDetectInvertedMarkers - Enable or disable upside-down marker detection.
 * @param bUseAruco3Detection - Whether or not to use the newer/faster method of detection. Experimental.
 * @param bEnableTracking - Whether or not to enable tracking of detected tags.
 * @param nDetectorMaxFPS - The max FPS limit the detector can run at.
 * @param bEnableRecordingFlag - Whether or not this TagDetector's overlay output should be recorded.
 * @param nNumDetectedTagsRetrievalThreads - The number of threads to use when fulfilling
 *                                           requests for the detected aruco tags. Default is 5.
 * @param bUsingGpuMats - Whether or not the given camera name will be using GpuMats.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
TagDetector::TagDetector(std::shared_ptr<ZEDCamera> pZEDCam,
                         const int nArucoCornerRefinementMaxIterations,
                         const int nArucoCornerRefinementMethod,
                         const int nArucoMarkerBorderBits,
                         const bool bArucoDetectInvertedMarkers,
                         const bool bUseAruco3Detection,
                         const bool bEnableTracking,
                         const int nDetectorMaxFPS,
                         const bool bEnableRecordingFlag,
                         const int nNumDetectedTagsRetrievalThreads,
                         const bool bUsingGpuMats)
{
    // Initialize member variables.
    m_pCamera                          = pZEDCam;
    m_bTorchInitialized                = false;
    m_bTorchEnabled                    = false;
    m_bUsingZedCamera                  = true;    // Toggle ZED functions on.
    m_bEnableTracking                  = bEnableTracking;
    m_bUsingGpuMats                    = bUsingGpuMats;
    m_bCameraIsOpened                  = false;
    m_nNumDetectedTagsRetrievalThreads = nNumDetectedTagsRetrievalThreads;
    m_szCameraName                     = pZEDCam->GetCameraModel() + "_" + std::to_string(pZEDCam->GetCameraSerial());
    m_bEnableRecordingFlag             = bEnableRecordingFlag;
    m_IPS                              = IPS();
    m_stRoverPose                      = geoops::RoverPose();

    // Setup aruco detector params.
    m_cvArucoDetectionParams                               = cv::aruco::DetectorParameters();
    m_cvArucoDetectionParams.cornerRefinementMaxIterations = nArucoCornerRefinementMaxIterations;
    m_cvArucoDetectionParams.cornerRefinementMethod        = nArucoCornerRefinementMethod;
    m_cvArucoDetectionParams.markerBorderBits              = nArucoMarkerBorderBits;
    m_cvArucoDetectionParams.detectInvertedMarker          = bArucoDetectInvertedMarkers;
    m_cvArucoDetectionParams.useAruco3Detection            = bUseAruco3Detection;

    // Create a multi-tracker for tracking multiple tags from the torch detectors.
    m_pMultiTracker = std::make_shared<tracking::MultiTracker>(constants::BBOX_TRACKER_LOST_TIMEOUT,
                                                               constants::BBOX_TRACKER_IOU_MATCH_THRESHOLD,
                                                               constants::BBOX_TRACKER_IOU_MATCH_THRESHOLD);

    // Set max IPS of main thread.
    this->SetMainThreadIPSLimit(nDetectorMaxFPS);

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "TagDetector created for camera: {}", m_szCameraName);
}

/******************************************************************************
 * @brief Destroy the Tag Detector:: Tag Detector object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-09
 ******************************************************************************/
TagDetector::~TagDetector()
{
    // Stop threaded code.
    this->RequestStop();
    this->Join();

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "TagDetector for camera {} has been successfully destroyed.", this->GetCameraName());
}

/******************************************************************************
 * @brief Take this detector's read handles on the camera data it consumes. Runs
 *      exactly once, on the first loop iteration. Holding these read handles for
 *      the detector's lifetime is what tells the camera to keep retrieving and
 *      publishing these data types; a type nobody holds a Reader on is never retrieved.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void TagDetector::EnsureCameraReaders()
{
    // Take the read handles exactly once, no matter how many times the loop runs.
    std::call_once(m_ocCameraReadersOnce,
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
                               // Take read handles on the GPU channels. Holding them is what makes the
                               // camera retrieve these data types at all.
                               m_rdCameraFrameGPU      = pZEDCamera->GetFrameGPUReader();
                               m_rdCameraPointCloudGPU = pZEDCamera->GetPointCloudGPUReader();
                           }
                           else
                           {
                               // Take read handles on the CPU channels.
                               m_rdCameraFrameCPU      = pZEDCamera->GetFrameCPUReader();
                               m_rdCameraPointCloudCPU = pZEDCamera->GetPointCloudCPUReader();
                           }
                       }
                       else
                       {
                           // Basic cameras publish a single BGRA frame channel and no point cloud.
                           m_rdCameraFrameCPU = std::dynamic_pointer_cast<BasicCamera>(m_pCamera)->GetFrameReader();
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
bool TagDetector::LoadLatestCameraFrames()
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
            pubsub::SharedSnapshot<cv::cuda::GpuMat> pFrameSnapshot = m_rdCameraFrameGPU.Get();
            pubsub::SharedSnapshot<cv::cuda::GpuMat> pCloudSnapshot = m_rdCameraPointCloudGPU.Get();
            // Nothing has been published yet.
            if (pFrameSnapshot == nullptr || pCloudSnapshot == nullptr)
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "TagDetector unable to get point cloud or frame from ZEDCam!");
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
            pubsub::SharedSnapshot<cv::Mat> pFrameSnapshot = m_rdCameraFrameCPU.Get();
            pubsub::SharedSnapshot<cv::Mat> pCloudSnapshot = m_rdCameraPointCloudCPU.Get();
            // Nothing has been published yet.
            if (pFrameSnapshot == nullptr || pCloudSnapshot == nullptr)
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "TagDetector unable to get point cloud or regular frame from ZEDCam!");
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
        pubsub::SharedSnapshot<cv::Mat> pFrameSnapshot = m_rdCameraFrameCPU.Get();
        // Nothing has been published yet.
        if (pFrameSnapshot == nullptr)
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "TagDetector unable to get RGB image from BasicCam!");
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
 * @brief This code will run continuously in a separate thread. New frames from
 *      the given camera are grabbed and the tags for the camera image are detected,
 *      filtered, and stored. Then any requests for the current tags are fulfilled.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
void TagDetector::ThreadedContinuousCode()
{
    ZoneScopedC(tracy::Color::Magenta1);
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
                            "TagDetector for ZED camera with serial number {} is waiting for its camera to open. Detection is paused until it does.",
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
                         "TagDetector for ZED camera with serial number {} now has an open camera. Resuming detection.",
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
                            "TagDetector for BasicCam at {} is waiting for its camera to open. Detection is paused until it does.",
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
                         "TagDetector for BasicCam at {} now has an open camera. Resuming detection.",
                         std::dynamic_pointer_cast<BasicCamera>(m_pCamera)->GetCameraLocation());
            }
        }
    }

    // Check if camera is opened.
    if (m_bCameraIsOpened)
    {
        // Register demand for the camera data we consume (once, on the first iteration).
        this->EnsureCameraReaders();

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

        {
            ZoneScopedNC("Detect Tags", tracy::Color::Magenta2);
            // Check if the frame is empty.
            if (m_cvFrame.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Frame from camera is empty!");
                return;
            }

            // Clear the list of newly detected tags.
            m_vNewlyDetectedTags.clear();
            // Clone frames.
            m_cvArucoProcFrame = m_cvFrame.clone();
            // Detect tags in the image
            std::vector<tagdetectutils::ArucoTag> vNewOpenCVTags = arucotag::Detect(m_cvArucoProcFrame, m_cvArucoDetector);
            // Loop through the newly detected OpenCV tags and set their detector UUID to this TagDetector's camera name so we can associate them with this detector.
            for (tagdetectutils::ArucoTag& stTag : vNewOpenCVTags)
            {
                stTag.szDetectorUUID = this->GetThreadUUID();
            }
            // Add OpenCV tags to the list of newly detected tags.
            m_vNewlyDetectedTags.insert(m_vNewlyDetectedTags.end(), vNewOpenCVTags.begin(), vNewOpenCVTags.end());

            // Check if torch detection if turned on.
            if (m_bTorchEnabled)
            {
                // Atomically load the model shared_ptr into a local so a concurrent InitTorchDetection()
                // swap can't invalidate it mid-inference (the local keeps the old model alive).
                std::shared_ptr<yolomodel::pytorch::PyTorchInterpreter> pTorchDetector = std::atomic_load_explicit(&m_pTorchDetector, std::memory_order_acquire);
                if (pTorchDetector != nullptr)
                {
                    // Detect tags in the image.
                    std::vector<tagdetectutils::ArucoTag> vNewTorchTags =
                        torchtag::Detect(m_cvArucoProcFrame, *pTorchDetector, m_fTorchMinObjectConfidence, m_fTorchNMSThreshold);

                    // Add Torch tags to the list of newly detected tags.
                    m_vNewlyDetectedTags.insert(m_vNewlyDetectedTags.end(), vNewTorchTags.begin(), vNewTorchTags.end());
                }
            }

            // Set the FOV of the camera in the tag structs for this detector's camera.
            for (tagdetectutils::ArucoTag& stTag : m_vNewlyDetectedTags)
            {
                // Set the UUID of the detector that detected this tag to this TagDetector's camera name so we can associate it with this detector.
                stTag.szDetectorUUID = this->GetThreadUUID();
                // Set tag FOV parameter to this tag detectors camera's FOV.
                stTag.dHorizontalFOV = m_pCamera->GetPropHorizontalFOV();
            }
        }

        // Merge the newly detected tags with the pre-existing detected tags.
        this->UpdateDetectedTags(m_vNewlyDetectedTags);

        // Draw tag overlays onto normal image.
        arucotag::DrawDetections(m_cvArucoProcFrame, m_vDetectedArucoTags);
        torchtag::DrawDetections(m_cvArucoProcFrame, m_vDetectedArucoTags);

        // Check if the detected tags vector is empty. If not, set the last good detection overlay frame to the current one with detections drawn on it.
        if (!m_vDetectedArucoTags.empty())
        {
            m_cvLastGoodDetectionOverlayFrame = m_cvArucoProcFrame.clone();
        }
        /////////////////////////////////////////////////////////////////////////////////////

        // Publish the freshly computed outputs to any subscribed consumers (deep copy once each).
        // Detection overlay frame.
        if (m_pubDetectionOverlay.HasReaders())
        {
            // Deep copy the overlay into a pooled snapshot and publish.
            std::shared_ptr<pubsub::Snapshot<cv::Mat>> pSlot = m_pubDetectionOverlay.Acquire();
            m_cvArucoProcFrame.copyTo(pSlot->tData);
            m_pubDetectionOverlay.Publish(std::move(pSlot));
        }
        // Last good detection overlay frame.
        if (m_pubLastGoodOverlay.HasReaders())
        {
            // Deep copy the last-good overlay into a pooled snapshot and publish.
            std::shared_ptr<pubsub::Snapshot<cv::Mat>> pSlot = m_pubLastGoodOverlay.Acquire();
            m_cvLastGoodDetectionOverlayFrame.copyTo(pSlot->tData);
            m_pubLastGoodOverlay.Publish(std::move(pSlot));
        }
        // Detected aruco tags. Published unconditionally rather than gated on demand: the tags are
        // already computed by the pass above, so publishing costs only a small vector copy, and
        // consumers reached through free functions (statemachine::LoadDetectedTags) have nowhere
        // natural to hold a Subscription.
        {
            // Copy the tags, then deep-copy each shared bounding box so the published snapshot is
            // truly immutable (ArucoTag::operator= only shallow-copies pBoundingBox, which the
            // tracker also holds and mutates).
            std::shared_ptr<pubsub::Snapshot<std::vector<tagdetectutils::ArucoTag>>> pSlot = m_pubDetectedTags.Acquire();
            pSlot->tData                                                                   = m_vDetectedArucoTags;
            for (tagdetectutils::ArucoTag& stTag : pSlot->tData)
            {
                // Give this snapshot its own bounding box instance.
                stTag.pBoundingBox = std::make_shared<cv::Rect2d>(*stTag.pBoundingBox);
            }
            m_pubDetectedTags.Publish(std::move(pSlot));
        }
    }
}

/******************************************************************************
 * @brief Not used. Detected tags and overlay frames are handed to consumers by the
 *      publish-latest mechanism in ThreadedContinuousCode(); no per-consumer fan-out
 *      work remains.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void TagDetector::PooledLinearCode() {}

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
bool TagDetector::InitTorchDetection(const std::string& szModelPath, yolomodel::pytorch::PyTorchInterpreter::HardwareDevices eDevice)
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
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-03-06
 ******************************************************************************/
void TagDetector::EnableTorchDetection(const float fMinObjectConfidence, const float fNMSThreshold)
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
        LOG_WARNING(logging::g_qSharedLogger, "Tried to enable torch detection for TagDetector but it has not been initialized yet!");
        // Update member variable.
        m_bTorchEnabled = false;
    }
}

/******************************************************************************
 * @brief Set flag to stop tag detection with the torch model.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-03-06
 ******************************************************************************/
void TagDetector::DisableTorchDetection()
{
    // Update member variables.
    m_bTorchEnabled = false;
}

/******************************************************************************
 * @brief Mutator for the desired max FPS for this detector.
 *
 * @param nRecordingFPS - The max frames per second to detect tags at.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-22
 ******************************************************************************/
void TagDetector::SetDetectorMaxFPS(const int nRecordingFPS)
{
    // Set the max iterations per second of the recording handler.
    this->SetMainThreadIPSLimit(nRecordingFPS);
}

/******************************************************************************
 * @brief Mutator for the Enable Recording Flag private member
 *
 * @param bEnableRecordingFlag - Whether or not recording should be enabled for this detector.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-31
 ******************************************************************************/
void TagDetector::SetEnableRecordingFlag(const bool bEnableRecordingFlag)
{
    m_bEnableRecordingFlag = bEnableRecordingFlag;
}

/******************************************************************************
 * @brief Accessor for the status of this TagDetector.
 *
 * @return true - The detector is running and detecting tags from the camera.
 * @return false - The detector thread and/or camera is not running/opened.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-04
 ******************************************************************************/
bool TagDetector::GetIsReady()
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
 * @brief Accessor for the desired max FPS for this detector.
 *
 * @return int - The max frames per second the detector can run at.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-22
 ******************************************************************************/
int TagDetector::GetDetectorMaxFPS() const
{
    // Return member variable value.
    return this->GetMainThreadMaxIPS();
}

/******************************************************************************
 * @brief Accessor for the Enable Recording Flag private member.
 *
 * @return true - Recording for this detector has been requested/flagged.
 * @return false - This detector should not be recorded.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-31
 ******************************************************************************/
bool TagDetector::GetEnableRecordingFlag() const
{
    return m_bEnableRecordingFlag;
}

/******************************************************************************
 * @brief Accessor for the camera name or path that this TagDetector is tied to.
 *
 * @return std::string - The name/path/index of the camera used by this TagDetector.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-01
 ******************************************************************************/
std::string TagDetector::GetCameraName()
{
    return m_szCameraName;
}

/******************************************************************************
 * @brief Accessor for the resolution of the process image used for tag detection.
 *
 * @return cv::Size - The resolution stored in an OpenCV cv::Size.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-01
 ******************************************************************************/
cv::Size TagDetector::GetProcessFrameResolution() const
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
 * @brief Updates the detected torch tags including tracking the detected tags over time
 *        and removing tags that haven't been seen for long enough.
 *
 * @param vNewlyDetectedTags - Input vector of TorchTag structs containing the tag info.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-03-15
 ******************************************************************************/
void TagDetector::UpdateDetectedTags(std::vector<tagdetectutils::ArucoTag>& vNewlyDetectedTags)
{
    ZoneScopedC(tracy::Color::Magenta3);

    // Check if tracking is enabled.
    if (m_bEnableTracking)
    {
        // Check if the given tag vector is empty
        if (vNewlyDetectedTags.empty())
        {
            // Since the tags are empty that means the detector has not detected any new ground truth tags.
            // In this case we will fallback to relying on the multi-tracker to track the tags and just update the tags
            // stored in the m_vDetectedArucoTags vector.
            // This is necessary because the torch detector is not perfect and may not detect all tags in the frame
            // and it doesn't have the ability to track tags over time.
            // We will use the multi-tracker to track the tags over time and update the bounding box data for the tags.

            // Update the multi-tracker with the current frame.
            m_pMultiTracker->Update(m_cvFrame);
        }
        else
        {
            // Loop through the newly detected tags.
            for (tagdetectutils::ArucoTag& stTag : vNewlyDetectedTags)
            {
                // Add the newly detected tags to the multi-tracker.
                bool bMatchedTagToExistingTracker = m_pMultiTracker->InitTracker(m_cvFrame, stTag.pBoundingBox, constants::BBOX_TRACKER_TYPE);
                // Check if the tag was matched to an existing tracker.
                if (!bMatchedTagToExistingTracker)
                {
                    // Add the new tag to the member variable list.
                    m_vDetectedArucoTags.emplace_back(stTag);
                }
                else
                {
                    // Find the tag with the same bounding box pointer and update the ID and confidence.
                    for (tagdetectutils::ArucoTag& stExistingTag : m_vDetectedArucoTags)
                    {
                        // Check if the bounding box pointers are the same.
                        if (stTag.pBoundingBox == stExistingTag.pBoundingBox)
                        {
                            // Update the ID and confidence of the existing tag.
                            stExistingTag.nID         = stTag.nID;
                            stExistingTag.dConfidence = stTag.dConfidence;
                        }
                    }
                }
            }

            // Update the multi-tracker with the current frame.
            m_pMultiTracker->Update(m_cvFrame);
        }

        // Loop through the detected tags and check if there are any we need to remove, and also update the time last seen.
        for (std::vector<tagdetectutils::ArucoTag>::iterator itTag = m_vDetectedArucoTags.begin(); itTag != m_vDetectedArucoTags.end();)
        {
            // Check if the bounding box is 0,0,0,0.
            if (itTag->pBoundingBox->x == 0 && itTag->pBoundingBox->y == 0 && itTag->pBoundingBox->width == 0 && itTag->pBoundingBox->height == 0)
            {
                // Remove the tag from the vector.
                itTag = m_vDetectedArucoTags.erase(itTag);
            }
            else
            {
                ++itTag;
            }
        }
    }
    else
    {
        // If tracking is not enabled, we will just clear the detected tags and add the new ones.
        m_vDetectedArucoTags.clear();
        // Loop through the newly detected tags and add them to the detected tags vector.
        for (tagdetectutils::ArucoTag& stTag : vNewlyDetectedTags)
        {
            // Set the tag creation time to 0. The tags aren't being tracked, so we can't really tell their age.
            stTag.tmCreation = std::chrono::system_clock::time_point::min();

            // Add the new tag to the member variable list.
            m_vDetectedArucoTags.emplace_back(stTag);
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

            // Apply the rotated offsets to the global coordinates
            stCamera.dEasting += dRotatedX;
            stCamera.dNorthing += dRotatedY;
            stCamera.dAltitude += m_pCamera->GetCameraPoseOffset().dPosZ;

            // Creating variables for the quaternion values.
            double dQW = m_pCamera->GetCameraPoseOffset().dQW;
            double dQX = m_pCamera->GetCameraPoseOffset().dQX;
            double dQY = m_pCamera->GetCameraPoseOffset().dQY;
            double dQZ = m_pCamera->GetCameraPoseOffset().dQZ;
            // Update the rover pose's heading to match the camera's heading. We will need to calculate the camera's heading using the quaternion.
            double dSinYCosP      = 2.0 * (dQW * dQY + dQX * dQZ);
            double dCosYCosP      = 1.0 - 2.0 * (dQX * dQX + dQY * dQY);
            double dCameraHeading = std::atan2(dSinYCosP, dCosYCosP) * (180.0 / CV_PI);

            // Add the relative camera heading to the absolute rover heading
            double dAbsoluteCameraHeading = numops::InputAngleModulus<double>(m_stRoverPose.GetCompassHeading() + dCameraHeading, 0.0, 360.0);

            // Recreate the rover pose with the camera's adjusted absolute position and absolute heading
            geoops::RoverPose stCameraPose = geoops::RoverPose(stCamera, dAbsoluteCameraHeading);

            // LOG_NOTICE(logging::g_qSharedLogger,
            //            "RoverPose GPS: {} {} | RoverPose Heading: {}",
            //            stCameraPose.GetGPSCoordinate().dLatitude,
            //            stCameraPose.GetGPSCoordinate().dLongitude,
            //            stCameraPose.GetCompassHeading());
            // // Recreate the rover pose with the camera's adjusted position and heading.
            // geoops::RoverPose stCameraPose = geoops::RoverPose(stCamera, dCameraHeading);

            // Loop through the tags and use their center point to lookup their distance in the point cloud.
            for (tagdetectutils::ArucoTag& stTag : m_vDetectedArucoTags)
            {
                // Use either width of height for the neighborhood size.
                int nNeighborhoodSize = std::min(stTag.pBoundingBox->width, stTag.pBoundingBox->height);
                // Geolocate the tag in the point cloud.
                geoops::Waypoint stGeolocation =
                    geoloc::GeolocateBox(m_cvPointCloud,
                                         stCameraPose,
                                         cv::Point(stTag.pBoundingBox->x + stTag.pBoundingBox->width / 2, stTag.pBoundingBox->y + stTag.pBoundingBox->height / 2),
                                         nNeighborhoodSize);

                // Calculate the yaw angle to the tag using the center point of the tag and the camera's field of view.
                // This is a fallback in case the geolocation fails for some reason, we can still provide a relative angle to the tag.
                // Get the center X pixel coordinate of the tag's bounding box.
                double dTagCenterX = stTag.pBoundingBox->x + (stTag.pBoundingBox->width / 2.0);
                // Get the center X pixel coordinate of the camera frame.
                double dFrameCenterX = m_cvFrame.cols / 2.0;
                // Calculate the offset in pixels from the center of the camera frame.
                // (Positive offset = target is to the right, Negative = target is to the left)
                double dPixelOffsetX = dTagCenterX - dFrameCenterX;
                // Calculate how many real-world degrees each pixel represents.
                double dDegreesPerPixel = stTag.dHorizontalFOV / static_cast<double>(m_cvFrame.cols);
                // Multiply the pixel offset by the degrees per pixel to get the relative yaw angle.
                stTag.dYawAngle = dPixelOffsetX * dDegreesPerPixel;
                // Explicitly set distance to 0.0 so the autonomy state machines know the depth map failed
                // and will properly fall back to using this calculated dYawAngle.
                stTag.dStraightLineDistance = 0.0;

                // Check if the geolocation is valid. If it is overwrite the yaw angle and distance with the geolocation data.
                if (stGeolocation != geoops::Waypoint())
                {
                    // Since this is a tag detection, set the tag's waypoint type appropriately.
                    stGeolocation.eType = geoops::WaypointType::eTagWaypoint;
                    // Calculate the geo measurement and print the distance to the tag.
                    geoops::GeoMeasurement stMeasurement = geoops::CalculateGeoMeasurement(m_stRoverPose.GetUTMCoordinate(), stGeolocation.GetUTMCoordinate());

                    // Check that the distance is in a reasonable range.
                    if (stMeasurement.dDistanceMeters > 0.0 && stMeasurement.dDistanceMeters < 25.0)
                    {
                        // Set the tag's geolocation.
                        stTag.stGeolocatedPosition = stGeolocation;
                        // Use the rover heading and the azimuth angle to calculate the relative heading to the tag.
                        stTag.dYawAngle = numops::AngularDifference(m_stRoverPose.GetCompassHeading(), stMeasurement.dStartRelativeBearing);
                        // Set the straight line distance to the tag.
                        stTag.dStraightLineDistance = stMeasurement.dDistanceMeters;
                    }
                }
            }
        }
    }
    else
    {
        // Estimate the positions of the tags using some basic trig.
        for (tagdetectutils::ArucoTag& stTag : m_vDetectedArucoTags)
        {
            // Use some trig to get the location of the tag.
            tagdetectutils::EstimatePoseFromCameraFrame(stTag);
        }
    }
}
