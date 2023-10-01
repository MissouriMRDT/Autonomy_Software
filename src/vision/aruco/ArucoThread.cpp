#include "ArucoThread.h"

ArucoThread::ArucoThread(const int nNumDetectedTagsRetrievalThreads) :
{
    m_nNumDetectedTagsRetrievalThreads = nNumDetectedTagsRetrievalThreads;

    // initialize aruco detector
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(constants::ARUCO_DICTIONARY);
    m_arucoDetector                  = ArucoDetector(dictionary);

    m_pZedCam                        = g_pCameraHandler->GetZED(CameraHandlerThread::eHeadMainCam);
}

std::future<bool> ArucoThread::RequestDetectedArucoTags(std::vector<ArucoTag>& arucoTagVec)
{
    // Assemble the DataFetchContainer.
    containers::DataFetchContainer<std::vector<ArucoTag>> stContainer(arucoTagVec);

    // Acquire lock on detected tags copy queue.
    std::unique_lock<std::mutex> lkScheduler(m_muPoolScheduleMatrix);
    // Append detected tag fetch container to the schedule queue.
    m_qDetectedTagCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedDataStatus->get_future();
}

void ArucoThread::ThreadedContinuousCode()
{
    // TODO: support pose estimation-this would include depth maps
    // TODO: pass reference to vector to ArucoDetector.Detect instead

    // hold camera frames
    cv::Mat cvNormalFrame;
    cv::cuda::GpuMat cvGPUNormalFrame;

    // indicates when the frame has been copied
    std::future<bool> fuFrameCopyStatus;

    // Check if the camera is setup to use CPU or GPU mats.
    if (constants::ZED_MAINCAM_USE_GPU_MAT)
    {
        // Grab frames from camera.
        fuFrameCopyStatus = m_pZedCam->RequestFrameCopy(cvGPUNormalFrame);
    }
    else
    {
        // Grab frames from camera.
        fuFrameCopyStatus = m_pZedCam->RequestFrameCopy(cvNormalFrame);
    }

    // Wait for the frames to be copied.
    if (fuFrameCopyStatus.get())
    {
        // Check if the camera is setup to use CPU or GPU mats.
        if (constants::ZED_MAINCAM_USE_GPU_MAT)
        {
            // Download memory from gpu mats if necessary.
            cvGPUNormalFrame.download(cvNormalFrame);
        }

        // Acquire lock on detected tags
        std::unique_lock<std::mutex> lkDetectedTags(m_muDetectedTagsMutex);
        // Update detected tags
        m_vDetectedTags = m_arucoDetector.Detect(cvNormalFrame);    // FIXME: No need to aquire a lock since the detect function call is not running in a seperate thread.
        // Release lock on detected tags
        lkDetectedTags.unlock();
    }

    // Acquire a shared_lock on the detected tags copy queue.
    std::shared_lock<std::mutex> lkSchedulers(m_muPoolScheduleMatrix);
    // Check if the detected tag copy queue is empty.
    if (!m_qDetectedTagCopySchedule.empty())
    {
        // FIXME: Won't compile.
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
        containers::DataFetchContainer<std::vector<ArucoTag>> stContainer = m_qDetectedTagCopySchedule.front();
        // Pop out of queue.
        m_qDetectedTagCopySchedule.pop();
        // Release lock.
        lkTagQueue.unlock();

        // Acquire reading access to the detected tags
        std::shared_lock<std::mutex> lkDetectedTags(m_muDetectedTagsMutex);

        // Copy the detected tags to the target location
        *(stContainer.pData) = m_vDetectedTags;

        // Release locks
        lkDetectedTags.unlock();
    }
}
