#include "ArucoThread.h"

ArucoThread::ArucoThread(CameraHandlerThread* cameraHandlerThread,
                        const int nNumDetectedTagsRetrievalThreads) 
    : m_pCameraHandlerThread(cameraHandlerThread), 
    nNumDetectedTagsRetrievalThreads(nNumDetectedTagsRetrievalThreads)
{
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    m_arucoDetector                  = ArucoDetector(dictionary);
}

std::future<bool> ArucoThread::RequestDetectedArucoTags(std::vector<ArucoTag>& arucoTagVec)
{
    // Assemble the DataFetchContainer.
    containers::DataFetchContainer<std::vector<ArucoTag>> stContainer(arucoTagVec);

    // Acquire lock on detected tags copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append detected tag fetch container to the schedule queue.
    m_qDetectedTagCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

void ArucoThread::ThreadContinousCode()
{
    // TODO: better handling of zedCamera pointer
    // TODO: support pose estimation-this would include depth maps
    // TODO: pass reference to vector to ArucoDetector.Detect instead

    ZEDCam* zedCamera = m_pCameraHandlerThread->GetZED(CameraHandlerThread::eHeadMainCam);

    cv::Mat cvNormalFrame;
    cv::cuda::GpuMat cvGPUNormalFrame;

    std::future<bool> fuFrameCopyStatus;

    // Check if the camera is setup to use CPU or GPU mats.
    if (constants::ZED_MAINCAM_USE_GPU_MAT)
    {
        // Grab frames from camera.
        fuFrameCopyStatus = zedCamera->RequestFrameCopy(cvGPUNormalFrame);
    }
    else
    {
        // Grab frames from camera.
        fuFrameCopyStatus = ExampleZEDCam1->RequestFrameCopy(cvNormalFrame);
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

        // Update detected tags
        m_muDetectedTagsCopyMutex.lock();
        m_detectedTags = m_arucoDetector.Detect(cvNormalFrame);
        m_muDetectedTagsCopyMutex.unlock();
    }

    // Acquire a shared_lock on the detected tags copy queue.
    std::shared_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Check if the frame copy queue is empty.
    if (!m_qDetectedTagCopySchedule.empty())
    {
        size_t = siQueueLength = m_qDetectedTagCopySchedule.size();
        // Start the thread pool to store multiple copies of the sl::Mat into the given cv::Mats.
        this->RunDetachedPool(siQueueLength, m_nNumFrameRetrievalThreads);
        // Wait for thread pool to finish.
        this->JoinPool();
        // Release lock on frame copy queue.
        lkSchedulers.unlock();
    }
}
