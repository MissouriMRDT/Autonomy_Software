/******************************************************************************
 * @brief Provides aruco detection and pose estimation capabilities in a multithreaded
 *      fashion using both OpenCV's ArUco library and a custom Tensorflow detection model.
 *
 * @file TagDetector.h
 * @author jspencerpittman (jspencerpittman@gmail.com)
 * @date 2023-10-01
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef TAG_DETECTOR_H
#define TAG_DETECTOR_H

#include <future>
#include <shared_mutex>
#include <vector>

#include "../../interfaces/AutonomyThread.hpp"
#include "../../vision/cameras/BasicCam.h"
#include "../../vision/cameras/ZEDCam.h"
#include "./ArucoDetection.hpp"
#include "./TensorflowDetection.hpp"

/******************************************************************************
 * @brief Run's Aruco detection & camera pose estimation in a multithreading environment
 *
 * What are the threads doing?
 * Continuous Thread:
 *  In this thread we are constantly getting images and depth maps from the necessary
 *  cameras. We then detect the tags in the image and estimate their location with respect to
 *  the rover.
 * Pooled Threads:
 *  Copy the vector of detected tags to all of the threads requesting it through the
 *  RequestDetectedArucoTags(...) function.
 *
 * @author jspencerpittman (jspencerpittman@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
class TagDetector : public AutonomyThread<void>
{
    public:
        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////
        TagDetector(BasicCam* pBasicCam, const int nNumDetectedTagsRetrievalThreads = 5, const bool bUsingGpuMats = false);
        TagDetector(ZEDCam* pZEDCam, const int nNumDetectedTagsRetrievalThreads = 5, const bool bUsingGpuMats = false);
        std::future<bool> RequestDetectedArucoTags(std::vector<arucotag::ArucoTag>& vArucoTags);
        std::future<bool> RequestDetectedTensorflowTags(std::vector<tensorflowtag::TensorflowTag>& vTensorflowTags);
        IPS& GetIPS();

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        // Class member variables.

        Camera<cv::Mat>* m_pCamera;
        bool m_bUsingZedCamera;
        bool m_bUsingGpuMats;
        int m_nNumDetectedTagsRetrievalThreads;
        IPS m_IPS;

        // Detected tags storage.
        std::vector<arucotag::ArucoTag> m_vDetectedArucoTags;
        std::vector<tensorflowtag::TensorflowTag> m_vDetectedTensorTags;

        // Queues and mutexes for scheduling and copying data to other threads.

        std::queue<containers::DataFetchContainer<std::vector<arucotag::ArucoTag>>> m_qDetectedArucoTagCopySchedule;
        std::queue<containers::DataFetchContainer<std::vector<tensorflowtag::TensorflowTag>>> m_qDetectedTensorflowTagCopySchedule;
        std::shared_mutex m_muPoolScheduleMutex;
        std::mutex m_muArucoDataCopyMutex;
        std::mutex m_muTensorflowDataCopyMutex;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////
        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
        void UpdateDetectedTags(std::vector<arucotag::ArucoTag>& vNewlyDetectedTags);
        void UpdateDetectedTags(std::vector<tensorflowtag::TensorflowTag>& vNewlyDetectedTags);
};

#endif
