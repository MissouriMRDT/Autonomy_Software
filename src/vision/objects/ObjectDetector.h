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
#include "../../util/threading/Publisher.hpp"
#include "../../util/vision/ObjectDetectionUtility.hpp"
#include "../../util/vision/YOLOModel.hpp"

/// \cond
#include <future>
#include <memory>
#include <mutex>
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

        /////////////////////////////////////////
        // Publish-latest data channels out. Consumers hold a Reader and Get() the newest
        // immutable snapshot without blocking this detector's loop.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Read handle for the detection-overlay frame channel.
         * @return pubsub::Reader<cv::Mat> - A demand-carrying read handle for the detection overlay channel.
         ******************************************************************************/
        pubsub::Reader<cv::Mat> GetDetectionOverlayReader() { return m_pubDetectionOverlay.CreateReader(); }

        /******************************************************************************
         * @brief Read handle for the last-good detection-overlay frame channel.
         * @return pubsub::Reader<cv::Mat> - A demand-carrying read handle for the last-good overlay channel.
         ******************************************************************************/
        pubsub::Reader<cv::Mat> GetLastGoodOverlayReader() { return m_pubLastGoodOverlay.CreateReader(); }

        /******************************************************************************
         * @brief Read handle for the detected objects channel.
         * @return pubsub::Reader<std::vector<objectdetectutils::Object>> - A demand-carrying read handle for the objects channel.
         ******************************************************************************/
        pubsub::Reader<std::vector<objectdetectutils::Object>> GetDetectedObjectsReader() { return m_pubDetectedObjects.CreateReader(); }

        /******************************************************************************
         * @brief Accessor for the number of detection passes skipped because the camera
         *      had not published a new frame since the last pass. Used to verify that the
         *      sequence-number short circuit is actually saving work.
         *
         * @return unsigned long long - The cumulative number of skipped detection passes.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        unsigned long long GetSkippedFrameCount() const { return m_ullSkippedFrameCount.load(std::memory_order_relaxed); }

    private:
        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
        void UpdateDetectedObjects(std::vector<objectdetectutils::Object>& vNewlyDetectedObjects);
        void EnsureCameraReaders();
        bool LoadLatestCameraFrames();

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

        // Edge-triggered logging of the camera's readiness. This detector idles (rather than
        // stopping itself) whenever its camera is not open, so without this it would log the same
        // "waiting for camera" line every iteration at its full detector FPS.
        bool m_bLastKnownCameraOpenState = true;
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
        cv::Mat m_cvLastGoodOverlayFrame;
        cv::Mat m_cvDetectionOverlayFrame;
        cv::Mat m_cvTorchProcFrame;
        cv::Mat m_cvPointCloud;

        // Demand for the camera data this detector consumes. Held for this detector's whole
        // lifetime so the camera retrieves and publishes only what is actually being used.

        std::once_flag m_ocCameraReadersOnce;
        // Typed read handles onto the camera channels this detector consumes. Only the pair
        // matching the camera's memory mode is ever active; the other stays default constructed
        // and inactive. Each handle both expresses demand (the camera retrieves nothing without
        // it) and is the only way to read the channel, so the two can never drift apart.
        pubsub::Reader<cv::Mat> m_rdCameraFrameCPU;
        pubsub::Reader<cv::Mat> m_rdCameraPointCloudCPU;
        pubsub::Reader<cv::cuda::GpuMat> m_rdCameraFrameGPU;
        pubsub::Reader<cv::cuda::GpuMat> m_rdCameraPointCloudGPU;

        // Sequence number of the last camera frame this detector actually ran detection on. Used
        // to skip an entire detection pass when the camera has not published a new frame yet.

        unsigned long long m_ullLastProcessedFrameSequence = 0;
        std::atomic<unsigned long long> m_ullSkippedFrameCount{0};

        // Publish-latest data channels out (see accessors above). Each is given an explicit
        // preallocation and growth ceiling so steady state allocates nothing and a consumer that
        // leaks snapshots trips the ceiling and is logged as an error.
        // The two overlay channels are demand gated (a full-frame clone each), while the detected
        // object channel publishes unconditionally: the objects are already computed by the
        // detection pass, so publishing them costs only a small vector copy and every consumer of
        // this detector wants them.
        pubsub::Publisher<cv::Mat> m_pubDetectionOverlay{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<cv::Mat> m_pubLastGoodOverlay{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<std::vector<objectdetectutils::Object>> m_pubDetectedObjects{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
};

#endif
