#include "ArucoThread.h"

ArucoThread::ArucoThread(const int nNumDetectedTagsRetrievalThreads)
{
    m_nNumDetectedTagsRetrievalThreads = nNumDetectedTagsRetrievalThreads;

    m_pZedCam                          = g_pCameraHandler->GetZED(CameraHandlerThread::eHeadMainCam);
}

std::future<bool> ArucoThread::RequestDetectedArucoTags(std::vector<aruco::ArucoTag>& arucoTagVec)
{
    // Assemble the DataFetchContainer.
    containers::DataFetchContainer<std::vector<aruco::ArucoTag>> stContainer(arucoTagVec);

    // Acquire lock on detected tags copy queue.
    std::unique_lock<std::mutex> lkScheduler(m_muPoolScheduleMatrix);
    // Append detected tag fetch container to the schedule queue.
    m_qDetectedTagCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedDataStatus->get_future();
}

void ArucoThread::UpdateDetectedTags(std::vector<aruco::ArucoTag>& vNewlyDetectedTags)
{
    std::sort(vNewlyDetectedTags.begin(), vNewlyDetectedTags.end(), [](const aruco::ArucoTag& a, const aruco::ArucoTag& b) { a.id < b.id; });

    auto newItr = vNewlyDetectedTags.begin();
    auto oldItr = m_vDetectedTags.begin();

    std::vector<int> vForgetTags;

    while (newItr != vNewlyDetectedTags.end() && oldItr != m_vDetectedTags.end())
    {
        if (oldItr->id == newItr->id)
        {
            oldItr->angle              = newItr->angle;
            oldItr->distance           = newItr->distance;
            oldItr->corners            = newItr->corners;
            oldItr->framesSinceLastHit = 0;
            oldItr->hits               = std::max(oldItr->hits + 1, constants::ARUCO_VALIDATION_THRESHOLD);

            oldItr++;
            newItr++;
        }
        else if (oldItr->id < newItr->id)
        {
            oldItr->framesSinceLastHit++;
            bool deletion = false;
            if (oldItr->hits == constants::ARUCO_VALIDATION_THRESHOLD)
            {
                if (oldItr->framesSinceLastHit >= constants::ARUCO_VALIDATED_TAG_FORGET_THRESHOLD)
                {
                    oldItr   = m_vDetectedTags.erase(oldItr);
                    deletion = true;
                }
            }
            else
            {
                if (oldItr->framesSinceLastHit >= constants::ARUCO_UNVALIDATED_TAG_FORGET_THRESHOLD)
                {
                    oldItr   = m_vDetectedTags.erase(oldItr);
                    deletion = true;
                }
            }

            if (!deletion)
                oldItr++;
        }
        else
        {
            m_vDetectedTags.push_back(*newItr);
            newItr++;
        }
    }

    while (newItr != vNewlyDetectedTags.end())
    {
        newItr->hits               = 1;
        newItr->framesSinceLastHit = 0;
        m_vDetectedTags.push_back(*newItr);
        newItr++;
    }

    while (oldItr != m_vDetectedTags.end())
    {
        oldItr->framesSinceLastHit++;
        bool deletion = false;
        if (oldItr->hits == constants::ARUCO_VALIDATION_THRESHOLD)
        {
            if (oldItr->framesSinceLastHit >= constants::ARUCO_VALIDATED_TAG_FORGET_THRESHOLD)
            {
                oldItr   = m_vDetectedTags.erase(oldItr);
                deletion = true;
            }
        }
        else
        {
            if (oldItr->framesSinceLastHit >= constants::ARUCO_UNVALIDATED_TAG_FORGET_THRESHOLD)
            {
                oldItr   = m_vDetectedTags.erase(oldItr);
                deletion = true;
            }
        }

        if (!deletion)
            oldItr++;
    }

    std::sort(m_vDetectedTags.begin(), m_vDetectedTags.end(), [](const aruco::ArucoTag& a, const aruco::ArucoTag& b) { a.id < b.id; });
}

void ArucoThread::ThreadedContinuousCode()
{
    // hold camera frames
    cv::Mat cvPointCloud;
    cv::Mat cvPointCloudColor;
    cv::cuda::GpuMat cvGPUPointCloud;

    // indicates when the frame has been copied
    std::future<bool> fuPointCloudCopyStatus;

    // Check if the camera is setup to use CPU or GPU mats.
    if (constants::ZED_MAINCAM_USE_GPU_MAT)
    {
        // Grab frames from camera.
        fuPointCloudCopyStatus = m_pZedCam->RequestPointCloudCopy(cvGPUPointCloud);
    }
    else
    {
        // Grab frames from camera.
        fuPointCloudCopyStatus = m_pZedCam->RequestFrameCopy(cvPointCloud);
    }

    // Wait for the frames to be copied.
    if (fuPointCloudCopyStatus.get())
    {
        // Check if the camera is setup to use CPU or GPU mats.
        if (constants::ZED_MAINCAM_USE_GPU_MAT)
        {
            // Download memory from gpu mats if necessary.
            cvGPUPointCloud.download(cvPointCloud);
        }
        imgops::SplitPointCloudColors(cvPointCloud, cvPointCloudColor);

        // Detect tags in the image
        std::vector vNewlyDetectedTags = aruco::Detect(cvPointCloudColor);

        // Estimate the positions of the tags using the point cloud
        for (auto& tag : vNewlyDetectedTags)
            EstimatePoseFromPointCloud(cvPointCloud, tag);

        // Merge the newly detected tags with the pre-existing detected tags
        UpdateDetectedTags(vNewlyDetectedTags);
    }

    // Acquire a shared_lock on the detected tags copy queue.
    std::unique_lock<std::mutex> lkSchedulers(m_muPoolScheduleMatrix);
    // Check if the detected tag copy queue is empty.
    if (!m_qDetectedTagCopySchedule.empty())
    {
        size_t siQueueLength = m_qDetectedTagCopySchedule.size();
        // Start the thread pool to store multiple copies of the detected tags to the requesting threads
        this->RunDetachedPool(siQueueLength, m_nNumDetectedTagsRetrievalThreads);
        // Wait for thread pool to finish.
        this->JoinPool();
        // Release lock on frame copy queue.
        lkSchedulers.unlock();
    }
}

void ArucoThread::PooledLinearCode()
{
    // Acquire sole writing access to the detectedTagCopySchedule
    std::unique_lock<std::mutex> lkTagQueue(m_muPoolScheduleMatrix);

    // If there are unfulfilled requests
    if (!m_qDetectedTagCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::DataFetchContainer<std::vector<aruco::ArucoTag>> stContainer = m_qDetectedTagCopySchedule.front();
        // Pop out of queue.
        m_qDetectedTagCopySchedule.pop();
        // Release lock.
        lkTagQueue.unlock();

        // Acquire reading access to the detected tags
        std::shared_lock<std::shared_mutex> lkDetectedTags(m_muDetectedTagsMutex);

        // Copy the detected tags to the target location
        *(stContainer.pData) = m_vDetectedTags;

        // Release locks
        lkDetectedTags.unlock();
    }
}
