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
#include "../../util/vision/ImageOperations.hpp"

/******************************************************************************
 * @brief Construct a new TagDetector object.
 *
 * @param pBasicCam - A pointer to the BasicCam camera to get frames from for detection.
 * @param nArucoCornerRefinementMaxIterations - The number of iterations to use when refining marker corners.
 * @param nArucoCornerRefinementMethod - The refinement method to use.
 * @param nArucoMarkerBorderBits - The number of border unit squares around the marker.
 * @param bArucoDetectInvertedMarkers - Enable or disable upside-down marker detection.
 * @param bUseAruco3Detection - Whether or not to use the newer/faster method of detection. Experimental.
 * @param bEnableRecordingFlag - Whether or not this TagDetector's overlay output should be recorded.
 * @param nNumDetectedTagsRetrievalThreads - The number of threads to use when fulfilling
 *                                           requests for the detected aruco tags. Default is 5.
 * @param bUsingGpuMats - Whether or not the given camera name will be using GpuMats.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-10
 ******************************************************************************/
TagDetector::TagDetector(BasicCam* pBasicCam,
                         const int nArucoCornerRefinementMaxIterations,
                         const int nArucoCornerRefinementMethod,
                         const int nArucoMarkerBorderBits,
                         const bool bArucoDetectInvertedMarkers,
                         const bool bUseAruco3Detection,
                         const bool bEnableRecordingFlag,
                         const int nNumDetectedTagsRetrievalThreads,
                         const bool bUsingGpuMats)
{
    // Initialize member variables.
    m_pCamera                          = pBasicCam;
    m_bUsingZedCamera                  = false;    // Toggle ZED functions off.
    m_nNumDetectedTagsRetrievalThreads = nNumDetectedTagsRetrievalThreads;
    m_bUsingGpuMats                    = bUsingGpuMats;
    m_szCameraName                     = dynamic_cast<BasicCam*>(pBasicCam)->GetCameraLocation();
    m_bEnableRecordingFlag             = bEnableRecordingFlag;
    m_IPS                              = IPS();

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
 * @param bEnableRecordingFlag - Whether or not this TagDetector's overlay output should be recorded.
 * @param nNumDetectedTagsRetrievalThreads - The number of threads to use when fulfilling
 *                                           requests for the detected aruco tags. Default is 5.
 * @param bUsingGpuMats - Whether or not the given camera name will be using GpuMats.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
TagDetector::TagDetector(ZEDCam* pZEDCam,
                         const int nArucoCornerRefinementMaxIterations,
                         const int nArucoCornerRefinementMethod,
                         const int nArucoMarkerBorderBits,
                         const bool bArucoDetectInvertedMarkers,
                         const bool bUseAruco3Detection,
                         const bool bEnableRecordingFlag,
                         const int nNumDetectedTagsRetrievalThreads,
                         const bool bUsingGpuMats)
{
    // Initialize member variables.
    m_pCamera                          = pZEDCam;
    m_bUsingZedCamera                  = true;    // Toggle ZED functions off.
    m_nNumDetectedTagsRetrievalThreads = nNumDetectedTagsRetrievalThreads;
    m_bUsingGpuMats                    = bUsingGpuMats;
    m_szCameraName                     = dynamic_cast<ZEDCam*>(pZEDCam)->GetCameraModel() + "_" + std::to_string(dynamic_cast<ZEDCam*>(pZEDCam)->GetCameraSerial());
    m_bEnableRecordingFlag             = bEnableRecordingFlag;
    m_IPS                              = IPS();

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
    LOG_DEBUG(logging::g_qSharedLogger, "TagDetector for camera {} had been successfully stopped.", this->GetCameraName());
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
    // Create instance variables.
    bool bCameraIsOpened = true;

    // Check if using ZEDCam or BasicCam.
    if (m_bUsingZedCamera)
    {
        // Check if camera is NOT open.
        if (!dynamic_cast<ZEDCam*>(m_pCamera)->GetCameraIsOpen())
        {
            // Shutdown threads for this ZEDCam.
            this->RequestStop();
            // Set camera opened toggle.
            bCameraIsOpened = false;

            // Submit logger message.
            LOG_CRITICAL(logging::g_qSharedLogger,
                         "Camera start was attempted for ZED camera with serial number {}, but camera never properly opened or it has been closed/rebooted!",
                         dynamic_cast<ZEDCam*>(m_pCamera)->GetCameraSerial());
        }
    }
    else
    {
        // Check if camera is NOT open.
        if (!dynamic_cast<BasicCam*>(m_pCamera)->GetCameraIsOpen())
        {
            // Shutdown threads for this BasicCam.
            this->RequestStop();
            // Set camera opened toggle.
            bCameraIsOpened = false;

            // Submit logger message.
            LOG_CRITICAL(logging::g_qSharedLogger,
                         "Camera start was attempted for BasicCam at {}, but camera never properly opened or it has become disconnected!",
                         dynamic_cast<BasicCam*>(m_pCamera)->GetCameraLocation());
        }
    }

    // Check if camera is opened.
    if (bCameraIsOpened)
    {
        // Create future for indicating when the frame has been copied.
        std::future<bool> fuPointCloudCopyStatus;

        // Check if the camera is setup to use CPU or GPU mats.
        if (m_bUsingZedCamera)
        {
            // Check if the ZED camera is returning cv::cuda::GpuMat or cv:Mat.
            if (m_bUsingGpuMats)
            {
                // Grabs point cloud from ZEDCam. Dynamic casts Camera to ZEDCam* so we can use ZEDCam methods.
                fuPointCloudCopyStatus = dynamic_cast<ZEDCam*>(m_pCamera)->RequestPointCloudCopy(m_cvGPUPointCloud);

                // Wait for point cloud to be retrieved.
                if (fuPointCloudCopyStatus.get())
                {
                    // Download mat from GPU memory.
                    m_cvGPUPointCloud.download(m_cvPointCloud);
                    // Split and store colors from point cloud.
                    imgops::SplitPointCloudColors(m_cvPointCloud, m_cvFrame);
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
                fuPointCloudCopyStatus = dynamic_cast<ZEDCam*>(m_pCamera)->RequestPointCloudCopy(m_cvPointCloud);

                // Wait for point cloud to be retrieved.
                if (fuPointCloudCopyStatus.get())
                {
                    // Split and store colors from point cloud.
                    imgops::SplitPointCloudColors(m_cvPointCloud, m_cvFrame);
                }
                else
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "TagDetector unable to get point cloud from ZEDCam!");
                }
            }
        }
        else
        {
            // Grab frames from camera.
            fuPointCloudCopyStatus = dynamic_cast<BasicCam*>(m_pCamera)->RequestFrameCopy(m_cvFrame);

            // Wait for point cloud to be retrieved.
            if (!fuPointCloudCopyStatus.get())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "TagDetector unable to get point cloud from BasicCam!");
            }
        }

        /////////////////////////////////////////
        // Actual detection logic goes here.
        /////////////////////////////////////////
        // Drop the Alpha channel from the image copy to preproc frame.
        cv::cvtColor(m_cvFrame, m_cvProcFrame, cv::COLOR_BGRA2BGR);
        // Run image through some pre-processing step to improve detection.
        arucotag::PreprocessFrame(m_cvProcFrame, m_cvProcFrame);
        // Detect tags in the image
        std::vector<arucotag::ArucoTag> vNewlyDetectedTags = arucotag::Detect(m_cvProcFrame, m_cvArucoDetector);
        // Draw tag overlays onto normal image.
        arucotag::DrawDetections(m_cvProcFrame, vNewlyDetectedTags);

        // // Estimate the positions of the tags using the point cloud
        // for (arucotag::ArucoTag& stTag : vNewlyDetectedTags)
        // {
        //     // Use the point cloud to get the location of the tag.
        //     arucotag::EstimatePoseFromPointCloud(cvPointCloud, stTag);
        // }

        // Merge the newly detected tags with the pre-existing detected tags
        this->UpdateDetectedTags(vNewlyDetectedTags);

        // Call FPS tick.
        m_IPS.Tick();
        /////////////////////////////////////////////////////////////////////////////////////

        // Acquire a shared_lock on the detected tags copy queue.
        std::shared_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
        // Check if the detected tag copy queue is empty.
        if (!m_qDetectedArucoTagCopySchedule.empty() || !m_qDetectedTensorflowTagCopySchedule.empty() || !m_qDetectedTagDrawnOverlayFrames.empty())
        {
            size_t siQueueLength =
                std::max({m_qDetectedArucoTagCopySchedule.size(), m_qDetectedTensorflowTagCopySchedule.size(), m_qDetectedTagDrawnOverlayFrames.size()});
            // Start the thread pool to store multiple copies of the detected tags to the requesting threads
            this->RunDetachedPool(siQueueLength, m_nNumDetectedTagsRetrievalThreads);
            // Wait for thread pool to finish.
            this->JoinPool();
            // Release lock on frame copy queue.
            lkSchedulers.unlock();
        }
    }
}

/******************************************************************************
 * @brief This method holds the code that is ran in the thread pool started by
 *      the ThreadedLinearCode() method. It copies the data from the different
 *      data objects to references of the same type stored in a queue filled by the
 *      Request methods.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-08
 ******************************************************************************/
void TagDetector::PooledLinearCode()
{
    /////////////////////////////
    //  Detection Overlay Frame queue.
    /////////////////////////////
    // Acquire sole writing access to the detectedTagCopySchedule.
    std::unique_lock<std::mutex> lkTagOverlayFrameQueue(m_muFrameCopyMutex);
    // Check if there are unfulfilled requests.
    if (!m_qDetectedTagDrawnOverlayFrames.empty())
    {
        // Get frame container out of queue.
        containers::FrameFetchContainer<cv::Mat> stContainer = m_qDetectedTagDrawnOverlayFrames.front();
        // Pop out of queue.
        m_qDetectedTagDrawnOverlayFrames.pop();
        // Release lock.
        lkTagOverlayFrameQueue.unlock();

        // Check which frame we should copy.
        switch (stContainer.eFrameType)
        {
            case eArucoDetection: *(stContainer.pFrame) = m_cvProcFrame; break;
            default: *(stContainer.pFrame) = m_cvProcFrame;
        }

        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedFrameStatus->set_value(true);
    }

    /////////////////////////////
    //  ArucoTag queue.
    /////////////////////////////
    // Acquire sole writing access to the detectedTagCopySchedule.
    std::unique_lock<std::mutex> lkArucoTagQueue(m_muArucoDataCopyMutex);
    // Check if there are unfulfilled requests.
    if (!m_qDetectedArucoTagCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::DataFetchContainer<std::vector<arucotag::ArucoTag>> stContainer = m_qDetectedArucoTagCopySchedule.front();
        // Pop out of queue.
        m_qDetectedArucoTagCopySchedule.pop();
        // Release lock.
        lkArucoTagQueue.unlock();

        // Copy the detected tags to the target location
        *(stContainer.pData) = m_vDetectedArucoTags;

        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedDataStatus->set_value(true);
    }

    /////////////////////////////
    //  TensorflowTag queue.
    /////////////////////////////
    // Acquire sole writing access to the detectedTagCopySchedule.
    std::unique_lock<std::mutex> lkTensorflowTagQueue(m_muTensorflowDataCopyMutex);
    // Check if there are unfulfilled requests.
    if (!m_qDetectedTensorflowTagCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::DataFetchContainer<std::vector<tensorflowtag::TensorflowTag>> stContainer = m_qDetectedTensorflowTagCopySchedule.front();
        // Pop out of queue.
        m_qDetectedTensorflowTagCopySchedule.pop();
        // Release lock.
        lkTensorflowTagQueue.unlock();

        // Copy the detected tags to the target location
        *(stContainer.pData) = m_vDetectedTensorTags;

        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedDataStatus->set_value(true);
    }
}

/******************************************************************************
 * @brief Request a copy of a frame containing the tag detection overlays from the
 *      aruco and tensorflow library.
 *
 * @param cvFrame - The frame to copy the detection overlay image to.
 * @return std::future<bool> - The future that should be waited on before using the passed in frame.
 *                      Future will be true or false based on whether or not the frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-11
 ******************************************************************************/
std::future<bool> TagDetector::RequestDetectionOverlayFrame(cv::Mat& cvFrame)
{
    // Assemble the DataFetchContainer.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvFrame, eArucoDetection);

    // Acquire lock on pool copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qDetectedTagDrawnOverlayFrames.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Request the most up to date vector of detected tags from OpenCV's Aruco
 *      algorithm.
 *
 * @param vArucoTags - The vector the detected aruco tags will be saved to.
 * @return std::future<bool> - The future that should be waited on before using the passed in tag vector.
 *                      Future will be true or false based on whether or not the tags were successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
std::future<bool> TagDetector::RequestDetectedArucoTags(std::vector<arucotag::ArucoTag>& vArucoTags)
{
    // Assemble the DataFetchContainer.
    containers::DataFetchContainer<std::vector<arucotag::ArucoTag>> stContainer(vArucoTags);

    // Acquire lock on pool copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append detected tag fetch container to the schedule queue.
    m_qDetectedArucoTagCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedDataStatus->get_future();
}

/******************************************************************************
 * @brief Request the most up to date vector of detected tags from our custom tensorflow
 *      model.
 *
 * @param vTensorflowTags - The vector the detected tensorflow tags will be saved to.
 * @return std::future<bool> - The future that should be waited on before using the passed in tag vector.
 *                      Future will be true or false based on whether or not the tags were successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
std::future<bool> TagDetector::RequestDetectedTensorflowTags(std::vector<tensorflowtag::TensorflowTag>& vTensorflowTags)
{
    // Assemble the DataFetchContainer.
    containers::DataFetchContainer<std::vector<tensorflowtag::TensorflowTag>> stContainer(vTensorflowTags);

    // Acquire lock on pool copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append detected tag fetch container to the schedule queue.
    m_qDetectedTensorflowTagCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedDataStatus->get_future();
}

/******************************************************************************
 * @brief Updates the detected aruco tags including forgetting tags that haven't been seen for long enough.
 *      If a new tag is spotted: add it to the detected tags vector
 *      If a tag has been spotted again: update the tags distance and angle
 *      If a tag hasn't been seen for a while: remove it from the vector
 *
 * @param vNewlyDetectedTags - Input vector of ArucoTag structs containing the tag info.
 *
 * @author jspencerpittman (jspencerpittman@gmail.com)
 * @date 2023-10-06
 ******************************************************************************/
void TagDetector::UpdateDetectedTags(std::vector<arucotag::ArucoTag>& vNewlyDetectedTags)
{
    // Sort tags from least to greatest.
    std::sort(vNewlyDetectedTags.begin(),
              vNewlyDetectedTags.end(),
              [](const arucotag::ArucoTag& stTag1, const arucotag::ArucoTag& stTag2) { return stTag1.nID < stTag2.nID; });

    // Get the beginning of the new tags and the current tags vector.
    std::vector<arucotag::ArucoTag>::iterator itNewItr = vNewlyDetectedTags.begin();
    std::vector<arucotag::ArucoTag>::iterator itOldItr = m_vDetectedArucoTags.begin();

    // Create vector for storing new tags.
    std::vector<arucotag::ArucoTag> vNewTags;

    // Here we process tags from both the newly detected and previously detected tags in the order of increasing id.
    while (itNewItr != vNewlyDetectedTags.end() || itOldItr != m_vDetectedArucoTags.end())
    {
        // If the id's match then update the previously detected tag.
        if (itNewItr != vNewlyDetectedTags.end() && itOldItr != m_vDetectedArucoTags.end() && itOldItr->nID == itNewItr->nID)
        {
            // Update data for tag.
            itOldItr->dYawAngle             = itNewItr->dYawAngle;
            itOldItr->dStraightLineDistance = itNewItr->dStraightLineDistance;
            itOldItr->vCorners              = itNewItr->vCorners;
            itOldItr->nFramesSinceLastHit   = 0;
            itOldItr->nHits                 = std::max(itOldItr->nHits + 1, constants::ARUCO_VALIDATION_THRESHOLD);

            // Move to next tags.
            itOldItr++;
            itNewItr++;
        }
        // If a previously detected tag wasn't detected in the frame
        else if (itOldItr != m_vDetectedArucoTags.end() && (itNewItr == vNewlyDetectedTags.end() || itOldItr->nID < itNewItr->nID))
        {
            // Increment hit counter.
            itOldItr->nFramesSinceLastHit++;

            // Check if the tag should be removed.
            if ((itOldItr->nHits >= constants::ARUCO_VALIDATION_THRESHOLD && itOldItr->nFramesSinceLastHit >= constants::ARUCO_VALIDATED_TAG_FORGET_THRESHOLD) ||
                !(itOldItr->nHits >= constants::ARUCO_VALIDATION_THRESHOLD && itOldItr->nFramesSinceLastHit >= constants::ARUCO_UNVALIDATED_TAG_FORGET_THRESHOLD))
            {
                // Remove the tag from the detected tags member variable.
                itOldItr = m_vDetectedArucoTags.erase(itOldItr);
            }
            else
            {
                // Decrement the old iterator.
                itOldItr++;
            }
        }
        // A tag was detected for the first time.
        else if (itNewItr != vNewlyDetectedTags.end())
        {
            // Set the new tags attributes for a first detection.
            itNewItr->nHits               = 1;
            itNewItr->nFramesSinceLastHit = 0;

            // Add tag to new tags vector.
            vNewTags.push_back(*itNewItr);

            // Increment the new iterator.
            itNewItr++;
        }
    }

    // Loop through the new tag vector.
    for (arucotag::ArucoTag& stTag : vNewTags)
    {
        // Add the newly detected tags to the member variable list
        m_vDetectedArucoTags.push_back(stTag);
    }
}

/******************************************************************************
 * @brief Updates the detected tensorflow tags including forgetting tags that haven't been seen for long enough.
 *      If a new tag is spotted: add it to the detected tags vector
 *      If a tag has been spotted again: update the tags distance and angle
 *      If a tag hasn't been seen for a while: remove it from the vector
 *
 * @param vNewlyDetectedTags - Input vector of TensorflowTag structs containing the tag info.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
void TagDetector::UpdateDetectedTags(std::vector<tensorflowtag::TensorflowTag>& vNewlyDetectedTags)
{
    // Sort tags from least to greatest.
    std::sort(vNewlyDetectedTags.begin(),
              vNewlyDetectedTags.end(),
              [](const tensorflowtag::TensorflowTag& stTag1, const tensorflowtag::TensorflowTag& stTag2) { return stTag1.nID < stTag2.nID; });

    // Get the beginning of the new tags and the current tags vector.
    std::vector<tensorflowtag::TensorflowTag>::iterator itNewItr = vNewlyDetectedTags.begin();
    std::vector<tensorflowtag::TensorflowTag>::iterator itOldItr = m_vDetectedTensorTags.begin();

    // Create vector for storing new tags.
    std::vector<tensorflowtag::TensorflowTag> vNewTags;

    // Here we process tags from both the newly detected and previously detected tags in the order of increasing id.
    while (itNewItr != vNewlyDetectedTags.end() || itOldItr != m_vDetectedTensorTags.end())
    {
        // If the id's match then update the previously detected tag.
        if (itNewItr != vNewlyDetectedTags.end() && itOldItr != m_vDetectedTensorTags.end() && itOldItr->nID == itNewItr->nID)
        {
            // Update data for tag.
            itOldItr->dYawAngle             = itNewItr->dYawAngle;
            itOldItr->dStraightLineDistance = itNewItr->dStraightLineDistance;
            itOldItr->vCorners              = itNewItr->vCorners;
            itOldItr->nFramesSinceLastHit   = 0;
            itOldItr->nHits                 = std::max(itOldItr->nHits + 1, constants::ARUCO_VALIDATION_THRESHOLD);

            // Move to next tags.
            itOldItr++;
            itNewItr++;
        }
        // If a previously detected tag wasn't detected in the frame
        else if (itOldItr != m_vDetectedTensorTags.end() && (itNewItr == vNewlyDetectedTags.end() || itOldItr->nID < itNewItr->nID))
        {
            // Increment hit counter.
            itOldItr->nFramesSinceLastHit++;

            // Check if the tag should be removed.
            if ((itOldItr->nHits >= constants::ARUCO_VALIDATION_THRESHOLD && itOldItr->nFramesSinceLastHit >= constants::ARUCO_VALIDATED_TAG_FORGET_THRESHOLD) ||
                !(itOldItr->nHits >= constants::ARUCO_VALIDATION_THRESHOLD && itOldItr->nFramesSinceLastHit >= constants::ARUCO_UNVALIDATED_TAG_FORGET_THRESHOLD))
            {
                // Remove the tag from the detected tags member variable.
                itOldItr = m_vDetectedTensorTags.erase(itOldItr);
            }
            else
            {
                // Decrement the old iterator.
                itOldItr++;
            }
        }
        // A tag was detected for the first time.
        else if (itNewItr != vNewlyDetectedTags.end())
        {
            // Set the new tags attributes for a first detection.
            itNewItr->nHits               = 1;
            itNewItr->nFramesSinceLastHit = 0;

            // Add tag to new tags vector.
            vNewTags.push_back(*itNewItr);

            // Increment the new iterator.
            itNewItr++;
        }
    }

    // Loop through the new tag vector.
    for (tensorflowtag::TensorflowTag& stTag : vNewTags)
    {
        // Add the newly detected tags to the member variable list
        m_vDetectedTensorTags.push_back(stTag);
    }
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
 * @brief Accessor for the Frame I P S private member.
 *
 * @return IPS& - The detector objects iteration per second counter.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-10
 ******************************************************************************/
IPS& TagDetector::GetIPS()
{
    // Return Iterations Per Second counter.
    return m_IPS;
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
    if (this->GetThreadState() == eRunning)
    {
        // Check if using ZEDCam or BasicCam.
        if (m_bUsingZedCamera)
        {
            // Check if camera is NOT open.
            if (dynamic_cast<ZEDCam*>(m_pCamera)->GetCameraIsOpen())
            {
                // Set camera opened toggle.
                bDetectorIsReady = true;
            }
        }
        else
        {
            // Check if camera is NOT open.
            if (dynamic_cast<BasicCam*>(m_pCamera)->GetCameraIsOpen())
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
        return dynamic_cast<ZEDCam*>(m_pCamera)->GetPropResolution();
    }
    else
    {
        // Concatenate camera path or index.
        return dynamic_cast<BasicCam*>(m_pCamera)->GetPropResolution();
    }
}
