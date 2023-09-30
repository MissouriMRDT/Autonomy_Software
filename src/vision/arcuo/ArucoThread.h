#ifndef ARUCO_THREAD_H_
#define ARUCO_THREAD_H_

#include <algorithm>
#include <future>
#include <shared_mutex>
#include <string>
#include <vector>

#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/opencv.hpp>

#include "./../../interfaces/AutonomyThread.hpp"
#include "./../../interfaces/Camera.hpp"
#include "./../../util/vision/ArucoSamplesUtility.hpp"
#include "./../../util/vision/FetchContainers.hpp"
#include "./Aruco.h"

class ArucoThread : public AutonomyThread<void>
{
    public:
        ArucoThread(ZEDCam<cv::Mat>* camera);
        ArucoThread(BasicCAM<cv::Mat>* camera);
        std::future<bool> RequestDetectedArucoTags(std::vector<ArucoTag>& arucoTagVec);

    private:
        bool m_bPoseEstimationActive;
        Camera<cv::Mat>* m_pCamera;

        std::vector<ArucoTag> m_vDetectedTags;

        std::queue<std::reference_wrapper<containers::DataFetchContainer<std::vector<ArucoTag>&>>> m_qDetectedTagCopySchedule;
        std::mutex m_muDetectedTagsCopyMutex;

        std::shared_mutex m_muPoolScheduleMatrix;

        void ThreadContinousCode() override;
        void PooledLinearCode() override;
};

#endif