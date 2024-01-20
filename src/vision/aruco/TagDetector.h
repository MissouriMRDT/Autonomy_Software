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

#include "../../interfaces/AutonomyThread.hpp"
#include "../../vision/cameras/BasicCam.h"
#include "../../vision/cameras/ZEDCam.h"
#include "./ArucoDetection.hpp"
#include "./TensorflowDetection.hpp"

/// \cond
#include <future>
#include <shared_mutex>
#include <vector>

/// \endcond

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
 * @author jspencerpittman (jspencerpittman@gmail.com), clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
class TagDetector : public AutonomyThread<void>
{
    public:
        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////
        TagDetector(BasicCam* pBasicCam,
                    const int nArucoCornerRefinementMaxIterations = 30,
                    const int nArucoCornerRefinementMethod        = cv::aruco::CORNER_REFINE_NONE,
                    const int nArucoMarkerBorderBits              = 1,
                    const bool bArucoDetectInvertedMarkers        = false,
                    const bool bUseAruco3Detection                = false,
                    const bool bEnableRecordingFlag               = false,
                    const int nNumDetectedTagsRetrievalThreads    = 5,
                    const bool bUsingGpuMats                      = false);
        TagDetector(ZEDCam* pZEDCam,
                    const int nArucoCornerRefinementMaxIterations = 30,
                    const int nArucoCornerRefinementMethod        = cv::aruco::CORNER_REFINE_NONE,
                    const int nArucoMarkerBorderBits              = 1,
                    const bool bArucoDetectInvertedMarkers        = false,
                    const bool bUseAruco3Detection                = false,
                    const bool bEnableRecordingFlag               = false,
                    const int nNumDetectedTagsRetrievalThreads    = 5,
                    const bool bUsingGpuMats                      = false);
        ~TagDetector();
        std::future<bool> RequestDetectionOverlayFrame(cv::Mat& cvFrame);
        std::future<bool> RequestDetectedArucoTags(std::vector<arucotag::ArucoTag>& vArucoTags);
        std::future<bool> RequestDetectedTensorflowTags(std::vector<tensorflowtag::TensorflowTag>& vTensorflowTags);
        void SetEnableRecordingFlag(const bool bEnableRecordingFlag);
        bool GetIsReady();
        bool GetEnableRecordingFlag() const;
        std::string GetCameraName();
        cv::Size GetProcessFrameResolution() const;

    private:
        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
        void UpdateDetectedTags(std::vector<arucotag::ArucoTag>& vNewlyDetectedTags);
        void UpdateDetectedTags(std::vector<tensorflowtag::TensorflowTag>& vNewlyDetectedTags);

        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        // Class member variables.

        Camera<cv::Mat>* m_pCamera;
        cv::aruco::ArucoDetector m_cvArucoDetector;
        cv::aruco::DetectorParameters m_cvArucoDetectionParams;
        cv::aruco::Dictionary m_cvTagDictionary;
        bool m_bUsingZedCamera;
        bool m_bUsingGpuMats;
        int m_nNumDetectedTagsRetrievalThreads;
        std::string m_szCameraName;
        std::atomic_bool m_bEnableRecordingFlag;

        // Detected tags storage.

        std::vector<arucotag::ArucoTag> m_vDetectedArucoTags;
        std::vector<tensorflowtag::TensorflowTag> m_vDetectedTensorTags;

        // Create frames for storing images and point clouds.

        cv::Mat m_cvFrame;
        cv::Mat m_cvProcFrame;
        cv::Mat m_cvPointCloud;
        cv::cuda::GpuMat m_cvGPUPointCloud;

        // Queues and mutexes for scheduling and copying data to other threads.

        std::queue<containers::FrameFetchContainer<cv::Mat>> m_qDetectedTagDrawnOverlayFrames;
        std::queue<containers::DataFetchContainer<std::vector<arucotag::ArucoTag>>> m_qDetectedArucoTagCopySchedule;
        std::queue<containers::DataFetchContainer<std::vector<tensorflowtag::TensorflowTag>>> m_qDetectedTensorflowTagCopySchedule;
        std::shared_mutex m_muPoolScheduleMutex;
        std::mutex m_muFrameCopyMutex;
        std::mutex m_muArucoDataCopyMutex;
        std::mutex m_muTensorflowDataCopyMutex;
};

#endif
