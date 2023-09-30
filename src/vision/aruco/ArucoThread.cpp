#include "ArucoThread.h"

// FIXME: Remove CameraHandlerThread.
ArucoThread::ArucoThread(CameraHandlerThread* cameraHandlerThread, const int nNumDetectedTagsRetrievalThreads) :
    m_pCameraHandlerThread(cameraHandlerThread),
    nNumDetectedTagsRetrievalThreads(nNumDetectedTagsRetrievalThreads)    // FIXME: Don't initialize class member variables like this. This method should only be used to
                                                                          // initialize members of inherited classes. Move these to the constructor body.
{
    // initialize aruco detector
    cv::aruco::Dictionary dictionary =
        cv::aruco::getPredefinedDictionary(DEFAULT_DICTIONARY);    // Change this to use the new variable in the constants namespace. (constants::ARUCO_DICTIONARY)

    m_arucoDetector = ArucoDetector(dictionary);
}

std::future<bool> ArucoThread::RequestDetectedArucoTags(std::vector<ArucoTag>& arucoTagVec)
{
    // Assemble the DataFetchContainer.
    containers::DataFetchContainer<std::vector<ArucoTag>> stContainer(arucoTagVec);

    // Acquire lock on detected tags copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);    // FIXME: Spelled wrong in header, make sure you are using devcontainer.
    // Append detected tag fetch container to the schedule queue.
    m_qDetectedTagCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();    // FIXME: Make sure you are developing within the container.
}

void ArucoThread::ThreadContinousCode()
{
    // TODO: better handling of zedCamera pointer
    // TODO: support pose estimation-this would include depth maps
    // TODO: pass reference to vector to ArucoDetector.Detect instead

    // Access zed camera
    // FIXME: Change m_pCameraHandlerThread to g_pCameraHandler.
    ZEDCam* zedCamera = m_pCameraHandlerThread->GetZED(CameraHandlerThread::eHeadMainCam);    // Add ZEDCam* m_pMainCam to header file and replace zedCam with m_pZedCam.
    // FIXME: No need to get a pointer ever iteration. Just get the pointer once in the constructor and store it in a member variable.

    // hold camera frames
    cv::Mat cvNormalFrame;
    cv::cuda::GpuMat cvGPUNormalFrame;

    // indicates when the frame has been copied
    std::future<bool> fuFrameCopyStatus;

    // Check if the camera is setup to use CPU or GPU mats.
    if (constants::ZED_MAINCAM_USE_GPU_MAT)    // FIXME: You need to #include "AutonomyConstants.h" to access the constants namespace.
    {
        // Grab frames from camera.
        fuFrameCopyStatus = zedCamera->RequestFrameCopy(cvGPUNormalFrame);
    }
    else
    {
        // Grab frames from camera.
        fuFrameCopyStatus = zedCamera->RequestFrameCopy(cvNormalFrame);
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
        m_detectedTags = m_arucoDetector.Detect(cvNormalFrame);    // FIXME: No need to aquire a lock since the detect function call is not running in a seperate thread.
        // Release lock on detected tags
        lkDetectedTags.unlock();
    }

    // Acquire a shared_lock on the detected tags copy queue.
    std::shared_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Check if the detected tag copy queue is empty.
    if (!m_qDetectedTagCopySchedule.empty())
    {
        // FIXME: Won't compile.
        size_t = siQueueLength = m_qDetectedTagCopySchedule.size();
        // Start the thread pool to store multiple copies of the detected tags to the requesting threads
        this->RunDetachedPool(siQueueLength, m_nNumFrameRetrievalThreads);
        // Wait for thread pool to finish.
        this->JoinPool();
        // Release lock on frame copy queue.
        lkSchedulers.unlock();
    }
}

ArucoThread::PooledLinearCode()
{
    // Acquire sole writing access to the detectedTagCopySchedule
    std::unique_lock<std::shared_mutex> lkTagQueue(m_muPoolScheduleMatrix);

    // If there are unfulfilled requests
    if (!m_qDetectedTagCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::DataFetchContainer<std::vector<ArucoTag>> stContainer = m_qDetectedTagCopySchedule.front();
        // Pop out of queue.
        m_qDetectedTagCopySchedule.pop();
        // Release lock.
        lkTagQueue.unlock();

        // Acquire sole writing access to the container
        std::unique_lock<std::mutex> lkConditionLock(
            stContainer
                .muConditionMutex);    // FIXME: Once the container is copied out of the queue, you now have sole ownership of the resource. No need to aquire a lock.

        // Acquire reading access to the detected tags
        std::shared_lock<std::mutex> lkDetectedTags(m_muDetectedTagsMutex);

        // Copy the detected tags to the target location
        stContainer.tData = m_vDetectedTags;    // FIXME: tData shoudl be pData and is a pointer, dereference it. *(stContainer.pData) = m_vDetectedTags;

        // Release locks
        lkConditionLock.unlock();
        lkDetectedTags.unlock();

        // Use the condition variable to notify other waiting threads that this thread is finished.
        stContainer.cdMatWriteSuccess
            .notify_all();    // FIXME: We don't use condition variables anymore, this is handled by promises and future. See BasicCam again for a good example.
    }
}
