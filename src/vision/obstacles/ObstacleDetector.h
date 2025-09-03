/******************************************************************************
 * @brief Provides FastSAM-based obstacle segmentation and classification utilizing
 *      FastSAM (Fast Segment Anything) instance segmentation and the ZED pc library
 *
 * @file ObstacleDetector.h
 * @author Donovan Bale (donovan@balehaus.org)
 * @date 2025-05-02
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include "../../interfaces/BasicCamera.hpp"
#include "../../interfaces/ZEDCamera.hpp"
#include "../../util/vision/BoundingBoxTracking.h"
#include "../../util/vision/ObstacleDetectionUtility.hpp"
#include "../../util/vision/YOLOModel.hpp"

/// \cond
#include <future>
#include <shared_mutex>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief Run FastSAM obstacle detection & camera pose estimation in multithreaded environment.
 *      Given a camera, detection will be run continuously on the camera frames.
 *
 * What are the threads doing?
 * Ask src/vision/aruco/TagDetector.h, I stole it from there.
 *
 *
 * @author Donovan Bale (donovan@balehaus.org)
 * @date 2025-05-02
 ******************************************************************************/
class ObstacleDetector : public AutonomyThread<void>
{
    public:
        ////////////////////////////////////////
        // Declare public methods.
        ////////////////////////////////////////
        ObstacleDetector(std::shared_ptr<BasicCamera> pBasicCam,
                         const int nDetectorMaxFPS                      = 30,
                         const bool bEnable_tracking                    = false,
                         const bool bEnableRecordingFlag                = false,
                         const int nNumDetectedObstacleRetrievalThreads = 5,
                         const bool bUsingGpuMats                       = false);
        ObstacleDetector(std::shared_ptr<ZEDCamera> pZEDCam,
                         const int nDetectorMaxFPS                      = 30,
                         const bool bEnable_tracking                    = false,
                         const bool bEnableRecordingFlag                = false,
                         const int nNumDetectedObstacleRetrievalThreads = 5,
                         const bool bUsingGpuMats                       = false);
        ~ObstacleDetector();

        std::future<bool> RequestDetectionOverlayFrame(cv::Mat& cvFrame);
        std::future<bool> RequestDetectedObstacles(std::vector<obstacledetectutils::Obstacle>& vObstacles);
        bool InitTorchDetection(const std::string& szModelPath,
                                yolomodel::pytorch::PyTorchInterpreter::HardwareDevices eDevice = yolomodel::pytorch::PyTorchInterpreter::HardwareDevices::eCUDA);

        ///////////////////////////////////////
        // Mutators.
        ///////////////////////////////////////

        void SetModelConfidenceParameters(const float fMinObjectConfidence = 0.5f, const float fIOUThreshold = 0.4f);
        void SetDetectorFPS(const int nRecordingFPS);
        void SetEnableRecordingFlag(const bool bEnableRecordingFlag);

        //////////////////////////////////////
        // Accessors.
        //////////////////////////////////////

        bool GetIsReady();
        int GetDetectorFPS() const;
        bool GetEnableRecordingFlag() const;
        std::string GetCameraName();
        cv::Size GetProcessFrameResolution() const;

    private:
        //////////////////////////////////////
        // Declare private methods
        //////////////////////////////////////

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;

        void UpdateDetectedObstacles(std::vector<obstacledetectutils::Obstacle>& vNewlyDetectedObstacles);

        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        // Class member variables.

        std::shared_ptr<Camera<cv::Mat>> m_pCamera;
        std::shared_ptr<yolomodel::pytorch::PyTorchInterpreter> m_pTorchDetector;

        std::atomic<float> m_fTorchMinObjectConfidence;
        std::atomic<float> m_fTorchIOUThreshold;
        std::atomic_bool m_bTorchInitialized;
        std::atomic_bool m_bTorchEnabled;
        std::shared_ptr<tracking::MultiTracker> m_pMultiTracker;
        bool m_bUsingZedCamera;
        bool m_bUsingGpuMats;
        bool m_bCameraIsOpened;
        bool m_bEnableTracking;
        int m_nNumDetectedObstacleRetrievalThreads;
        std::string m_szCameraName;
        std::atomic_bool m_bEnableRecordingFlag;

        // Detected obstacle storage.

        std::vector<obstacledetectutils::Obstacle> m_vNewlyDetectedObstacles;
        std::vector<obstacledetectutils::Obstacle> m_vDetectedObstacles;

        // Create frames for storing images and point clouds.

        cv::Mat m_cvFrame;
        cv::cuda::GpuMat m_cvGPUFrame;
        cv::Mat m_cvTorchProcFrame;
        cv::Mat m_cvPointCloud;
        cv::cuda::GpuMat m_cvGPUPointCloud;

        // Queues and mutexes for scheduling and copying data to other threads.

        std::queue<containers::FrameFetchContainer<cv::Mat>> m_qDetectedObstacleDrawnOverlayFramesCopySchedule;    // wtf
        std::queue<containers::DataFetchContainer<std::vector<obstacledetectutils::Obstacle>>> m_qDetectedObstacleCopySchedule;
        std::shared_mutex m_muPoolScheduleMutex;
        std::shared_mutex m_muFrameCopyMutex;
        std::shared_mutex m_muObstacleDataCopyMutex;
};

#endif
