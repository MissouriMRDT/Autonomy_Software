#include "ArucoThread.h"

ArucoThread::ArucoThread(ZEDCam<cv::Mat>* camera) : m_pCamera(camera), m_bPoseEstimationActive(true) {}

ArucoThread::ArucoThread(BasicCam<cv::Mat>* camera) : m_pCamera(camera), m_bPoseEstimationActive(false) {}

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

void ArucoThread::ThreadContinousCode() {}
