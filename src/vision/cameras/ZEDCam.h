/******************************************************************************
 * @brief Defines the ZEDCam class.
 *
 * @file ZEDCam.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-25
 *
 * @copyright Copyright MRDT 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef ZEDCAM_H
#define ZEDCAM_H

#include "../../AutonomyConstants.h"
#include "../../interfaces/AutonomyThread.hpp"
#include "../../interfaces/Camera.hpp"

/// \cond
#include <future>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <sl/Fusion.hpp>

/// \endcond

/******************************************************************************
 * @brief This class implements and interfaces with the most common ZEDSDK cameras
 *  and features. It is designed in such a way that multiple other classes/threads
 *  can safely call any method of an object of this class withing resource corruption
 *  or slowdown of the camera.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
 ******************************************************************************/
class ZEDCam : public Camera<cv::Mat>, public AutonomyThread<void>
{
    public:
        /////////////////////////////////////////
        // Declare public structs that are specific to and used within this class.
        /////////////////////////////////////////

        struct ZedObjectData;

        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////

        ZEDCam(const int nPropResolutionX,
               const int nPropResolutionY,
               const int nPropFramesPerSecond,
               const double dPropHorizontalFOV,
               const double dPropVerticalFOV,
               const bool bEnableRecordingFlag,
               const float fMinSenseDistance           = constants::ZED_DEFAULT_MINIMUM_DISTANCE,
               const float fMaxSenseDistance           = constants::ZED_DEFAULT_MAXIMUM_DISTANCE,
               const bool bMemTypeGPU                  = false,
               const bool bUseHalfDepthPrecision       = false,
               const bool bEnableFusionMaster          = false,
               const int nNumFrameRetrievalThreads     = 10,
               const unsigned int unCameraSerialNumber = 0);
        ~ZEDCam();
        std::future<bool> RequestFrameCopy(cv::Mat& cvFrame) override;
        std::future<bool> RequestFrameCopy(cv::cuda::GpuMat& cvGPUFrame);
        std::future<bool> RequestDepthCopy(cv::Mat& cvDepth, const bool bRetrieveMeasure = true);
        std::future<bool> RequestDepthCopy(cv::cuda::GpuMat& cvGPUDepth, const bool bRetrieveMeasure = true);
        std::future<bool> RequestPointCloudCopy(cv::Mat& cvPointCloud);
        std::future<bool> RequestPointCloudCopy(cv::cuda::GpuMat& cvGPUPointCloud);
        sl::ERROR_CODE ResetPositionalTracking();
        sl::ERROR_CODE TrackCustomBoxObjects(std::vector<ZedObjectData>& vCustomObjects);
        sl::ERROR_CODE RebootCamera();
        sl::FUSION_ERROR_CODE SubscribeFusionToCameraUUID(sl::CameraIdentifier& slCameraUUID);
        sl::CameraIdentifier PublishCameraToFusion();

        /////////////////////////////////////////
        // Setters for class member variables.
        /////////////////////////////////////////

        sl::ERROR_CODE EnablePositionalTracking();
        void DisablePositionalTracking();
        sl::ERROR_CODE SetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO);
        sl::ERROR_CODE EnableSpatialMapping(const int nTimeoutSeconds = 10);
        void DisableSpatialMapping();
        sl::ERROR_CODE EnableObjectDetection(const bool bEnableBatching = false);
        void DisableObjectDetection();

        /////////////////////////////////////////
        // Accessors for class member variables.
        /////////////////////////////////////////

        bool GetCameraIsOpen() override;
        bool GetUsingGPUMem() const;
        std::string GetCameraModel();
        unsigned int GetCameraSerial();
        std::future<bool> RequestPositionalPoseCopy(sl::Pose& slPose);
        std::future<bool> RequestFusionGeoPoseCopy(sl::GeoPose& slGeoPose);
        bool GetPositionalTrackingEnabled();
        sl::SPATIAL_MAPPING_STATE GetSpatialMappingState();
        sl::SPATIAL_MAPPING_STATE ExtractSpatialMapAsync(std::future<sl::Mesh>& fuMeshFuture);
        bool GetObjectDetectionEnabled();
        std::future<bool> RequestObjectsCopy(std::vector<sl::ObjectData>& vObjectData);
        std::future<bool> RequestBatchedObjectsCopy(std::vector<sl::ObjectsBatch>& vBatchedObjectData);

    private:
        /////////////////////////////////////////
        // Declare class constants.
        /////////////////////////////////////////
        const std::memory_order ATOMIC_MEMORY_ORDER_METHOD = std::memory_order_relaxed;

        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        // ZED Camera specific.

        sl::Camera m_slCamera;
        std::shared_mutex m_muCameraMutex;
        sl::InitParameters m_slCameraParams;
        sl::RuntimeParameters m_slRuntimeParams;
        sl::Fusion m_slFusionInstance;
        sl::InitFusionParameters m_slFusionParams;
        sl::MEASURE m_slDepthMeasureType;
        sl::PositionalTrackingParameters m_slPoseTrackingParams;
        sl::PositionalTrackingFusionParameters m_slFusionPoseTrackingParams;
        sl::Pose m_slCameraPose;
        sl::GeoPose m_slFusionGeoPose;
        sl::SpatialMappingParameters m_slSpatialMappingParams;
        sl::ObjectDetectionParameters m_slObjectDetectionParams;
        sl::BatchParameters m_slObjectDetectionBatchParams;
        sl::Objects m_slDetectedObjects;
        std::vector<sl::ObjectsBatch> m_slDetectedObjectsBatched;
        sl::MEM m_slMemoryType;
        bool m_bCameraIsFusionMaster;
        int m_nNumFrameRetrievalThreads;
        unsigned int m_unCameraSerialNumber;

        // Mats for storing frames and measures.

        sl::Mat m_slFrame;
        sl::Mat m_slDepthImage;
        sl::Mat m_slDepthMeasure;
        sl::Mat m_slPointCloud;

        // Queues and mutexes for scheduling and copying camera frames and data to other threads.

        std::queue<containers::FrameFetchContainer<cv::cuda::GpuMat>> m_qGPUFrameCopySchedule;
        std::queue<containers::DataFetchContainer<std::vector<ZedObjectData>>> m_qCustomBoxIngestSchedule;
        std::queue<containers::DataFetchContainer<sl::Pose>> m_qPoseCopySchedule;
        std::queue<containers::DataFetchContainer<sl::GeoPose>> m_qGeoPoseCopySchedule;
        std::queue<containers::DataFetchContainer<std::vector<double>>> m_qIMUDataCopySchedule;
        std::queue<containers::DataFetchContainer<std::vector<sl::ObjectData>>> m_qObjectDataCopySchedule;
        std::queue<containers::DataFetchContainer<std::vector<sl::ObjectsBatch>>> m_qObjectBatchedDataCopySchedule;
        std::mutex m_muCustomBoxIngestMutex;
        std::mutex m_muPoseCopyMutex;
        std::mutex m_muGeoPoseCopyMutex;
        std::mutex m_muObjectDataCopyMutex;
        std::mutex m_muObjectBatchedDataCopyMutex;
        std::atomic<bool> m_bNormalFramesQueued;
        std::atomic<bool> m_bDepthFramesQueued;
        std::atomic<bool> m_bPointCloudsQueued;
        std::atomic<bool> m_bPosesQueued;
        std::atomic<bool> m_bGeoPosesQueued;
        std::atomic<bool> m_bObjectsQueued;
        std::atomic<bool> m_bBatchedObjectsQueued;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
};
#endif
