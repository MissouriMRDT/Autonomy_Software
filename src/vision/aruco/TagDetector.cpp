/******************************************************************************
 * @brief Implements the TagDetector class.
 *
 * @file TagDetector.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
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
 * @param nNumDetectedTagsRetrievalThreads - The number of threads to use when fulfilling
 *                                           requests for the detected aruco tags. Default is 5.
 * @param bUsingGpuMats - Whether or not the given camera name will be using GpuMats.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
TagDetector::TagDetector(BasicCam* pBasicCam, int nNumDetectedTagsRetrievalThreads, const bool bUsingGpuMats)
{
    // Initialize member variables.
    m_pCamera                          = dynamic_cast<BasicCam*>(pBasicCam);
    m_bUsingZedCamera                  = false;    // Toggle ZED functions off.
    m_nNumDetectedTagsRetrievalThreads = nNumDetectedTagsRetrievalThreads;
    m_bUsingGpuMats                    = bUsingGpuMats;
    m_IPS                              = IPS();
}

/******************************************************************************
 * @brief Construct a new TagDetector object.
 *
 * @param pZEDCam - A pointer to the ZEDCam camera to get frames from for detection. Override for ZED camera.
 * @param nNumDetectedTagsRetrievalThreads - The number of threads to use when fulfilling
 *                                           requests for the detected aruco tags. Default is 5.
 * @param bUsingGpuMats - Whether or not the given camera name will be using GpuMats.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
TagDetector::TagDetector(ZEDCam* pZEDCam, int nNumDetectedTagsRetrievalThreads, const bool bUsingGpuMats)
{
    // Initialize member variables.
    m_pCamera                          = dynamic_cast<ZEDCam*>(pZEDCam);
    m_bUsingZedCamera                  = true;    // Toggle ZED functions off.
    m_nNumDetectedTagsRetrievalThreads = nNumDetectedTagsRetrievalThreads;
    m_bUsingGpuMats                    = bUsingGpuMats;
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
    // Create persistent object for holding frames.
    cv::Mat cvFrame;
    cv::Mat cvPointCloud;
    cv::cuda::GpuMat cvGPUPointCloud;

    // Create future for indicating when the frame has been copied.
    std::future<bool> fuPointCloudCopyStatus;

    // Check if the camera is setup to use CPU or GPU mats.
    if (m_bUsingZedCamera)
    {
        // Check if the ZED camera is returning cv::cuda::GpuMat or cv:Mat.
        if (m_bUsingGpuMats)
        {
            // Grabs point cloud from ZEDCam. Dynamic casts Camera to ZEDCam* so we can use ZEDCam methods.
            fuPointCloudCopyStatus = dynamic_cast<ZEDCam*>(m_pCamera)->RequestPointCloudCopy(cvGPUPointCloud);

            // Wait for point cloud to be retrieved.
            if (fuPointCloudCopyStatus.get())
            {
                // Download mat from GPU memory.
                cvGPUPointCloud.download(cvPointCloud);
                // Split and store colors from point cloud.
                imgops::SplitPointCloudColors(cvPointCloud, cvFrame);
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
            fuPointCloudCopyStatus = dynamic_cast<ZEDCam*>(m_pCamera)->RequestPointCloudCopy(cvPointCloud);

            // Wait for point cloud to be retrieved.
            if (fuPointCloudCopyStatus.get())
            {
                // Split and store colors from point cloud.
                imgops::SplitPointCloudColors(cvPointCloud, cvFrame);
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
        fuPointCloudCopyStatus = dynamic_cast<BasicCam*>(m_pCamera)->RequestFrameCopy(cvFrame);

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
    // Detect tags in the image
    std::vector<arucotag::ArucoTag> vNewlyDetectedTags = arucotag::Detect(cvFrame);

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
    if (!m_qDetectedArucoTagCopySchedule.empty() || !m_qDetectedTensorflowTagCopySchedule.empty())
    {
        size_t siQueueLength = std::max({m_qDetectedArucoTagCopySchedule.size(), m_qDetectedTensorflowTagCopySchedule.size()});
        // Start the thread pool to store multiple copies of the detected tags to the requesting threads
        this->RunDetachedPool(siQueueLength, m_nNumDetectedTagsRetrievalThreads);
        // Wait for thread pool to finish.
        this->JoinPool();
        // Release lock on frame copy queue.
        lkSchedulers.unlock();
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
 * @brief Request the most up to date vector of detected tags from OpenCV's Aruco
 *      algorithm.
 *
 * @param vArucoTags - The vector the detected aruco tags will be saved to.
 * @return std::future<bool> - The future that should be waited on before using the passed in tag vector.
 *                      Future will be true or false based on whether or not the tags where successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
std::future<bool> TagDetector::RequestDetectedArucoTags(std::vector<arucotag::ArucoTag>& vArucoTags)
{
    // Assemble the DataFetchContainer.
    containers::DataFetchContainer<std::vector<arucotag::ArucoTag>> stContainer(vArucoTags);

    // Acquire lock on detected tags copy queue.
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
 *                      Future will be true or false based on whether or not the tags where successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
std::future<bool> TagDetector::RequestDetectedTensorflowTags(std::vector<tensorflowtag::TensorflowTag>& vTensorflowTags)
{
    // Assemble the DataFetchContainer.
    containers::DataFetchContainer<std::vector<tensorflowtag::TensorflowTag>> stContainer(vTensorflowTags);

    // Acquire lock on detected tags copy queue.
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
        // Add the newly detected tags to the list
        vNewlyDetectedTags.push_back(stTag);
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
        // Add the newly detected tags to the list
        vNewlyDetectedTags.push_back(stTag);
    }
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
