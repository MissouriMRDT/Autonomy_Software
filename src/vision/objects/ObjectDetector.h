/******************************************************************************
 * @brief Implements the ObjectDetector class.
 *
 * @file ObjectDetector.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-24
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include "../../interfaces/BasicCamera.hpp"
#include "../../interfaces/ZEDCamera.hpp"
#include "../../util/vision/ObjectDetectionUtility.hpp"
#include "../../util/vision/YOLOModel.hpp"

/// \cond
#include <future>
#include <shared_mutex>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief This class implements a modular and easy to use object detector for a single
 *      camera. Given a camera name, this class will detect objects using the depth measure
 *      from a ZED camera and/or inferenced objects from a custom trained model.
 *      This class and it's detections are ran in a different thread.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-24
 ******************************************************************************/
class ObjectDetector : public AutonomyThread<void>
{
    public:
        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////
        ObjectDetector(std::shared_ptr<BasicCamera> pBasicCam,
                       const bool bEnableTracking                    = false,
                       const int nDetectorMaxFPS                     = 30,
                       const bool bEnableRecordingFlag               = false,
                       const int nNumDetectedObjectsRetrievalThreads = 5,
                       const bool bUsingGpuMats                      = false);
        ObjectDetector(std::shared_ptr<ZEDCamera> pZEDCam,
                       const bool bEnableTracking                    = false,
                       const int nDetectorMaxFPS                     = 30,
                       const bool bEnableRecordingFlag               = false,
                       const int nNumDetectedObjectsRetrievalThreads = 5,
                       const bool bUsingGpuMats                      = false);
        ~ObjectDetector();
        std::future<bool> RequestDetectionOverlayFrame(cv::Mat& cvFrame);
        std::future<bool> RequestDetectedObjects(std::vector<objectdetectutils::Object>& vObjects);
        bool InitTorchDetection(const std::string& szModelPath,
                                yolomodel::pytorch::PyTorchInterpreter::HardwareDevices eDevice = yolomodel::pytorch::PyTorchInterpreter::HardwareDevices::eCUDA);

        /////////////////////////////////////////
        // Mutators.
        /////////////////////////////////////////

        void EnableTorchDetection(const float fMinObjectConfidence = 0.4f, const float fNMSThreshold = 0.6f);
        void DisableTorchDetection();
        void SetDetectorMaxFPS(const int nRecordingFPS);
        void SetEnableRecordingFlag(const bool bEnableRecordingFlag);

        /////////////////////////////////////////
        // Accessors.
        /////////////////////////////////////////

        bool GetIsReady();
        int GetDetectorMaxFPS() const;
        bool GetEnableRecordingFlag() const;
        std::string GetCameraName();
        cv::Size GetProcessFrameResolution() const;

    private:
        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
        void UpdateDetectedObjects(std::vector<objectdetectutils::Object>& vNewlyDetectedObjects);

        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        // Class member variables.

        std::shared_ptr<Camera<cv::Mat>> m_pCamera;
        std::shared_ptr<yolomodel::pytorch::PyTorchInterpreter> m_pTorchDetector;
        std::atomic<float> m_fTorchMinObjectConfidence;
        std::atomic<float> m_fTorchNMSThreshold;
        std::atomic_bool m_bTorchInitialized;
        std::atomic_bool m_bTorchEnabled;
        std::shared_ptr<tracking::MultiTracker> m_pMultiTracker;
        bool m_bUsingZedCamera;
        bool m_bUsingGpuMats;
        bool m_bCameraIsOpened;
        bool m_bEnableTracking;
        int m_nNumDetectedObjectsRetrievalThreads;
        std::string m_szCameraName;
        std::atomic_bool m_bEnableRecordingFlag;

        // Detected tags storage.

        std::vector<objectdetectutils::Object> m_vNewlyDetectedObjects;
        std::vector<objectdetectutils::Object> m_vDetectedObjects;

        // Rover position for tag geolocalization.
        geoops::RoverPose m_stRoverPose;

        // Create frames for storing images and point clouds.

        cv::Mat m_cvFrame;
        cv::cuda::GpuMat m_cvGPUFrame;
        cv::Mat m_cvTorchProcFrame;
        cv::Mat m_cvPointCloud;
        cv::cuda::GpuMat m_cvGPUPointCloud;

        // Queues and mutexes for scheduling and copying data to other threads.

        std::queue<containers::FrameFetchContainer<cv::Mat>> m_qDetectedObjectDrawnOverlayFramesCopySchedule;
        std::queue<containers::DataFetchContainer<std::vector<objectdetectutils::Object>>> m_qDetectedObjectCopySchedule;
        std::shared_mutex m_muPoolScheduleMutex;
        std::shared_mutex m_muFrameCopyMutex;
        std::shared_mutex m_muArucoDataCopyMutex;
};

#endif
