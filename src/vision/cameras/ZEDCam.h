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

#include <future>
#include <opencv2/opencv.hpp>
#include <shared_mutex>
#include <sl/Camera.hpp>

#include "../../AutonomyConstants.h"
#include "../../interfaces/AutonomyThread.hpp"
#include "../../interfaces/Camera.hpp"
#include "../../util/vision/FetchContainers.hpp"

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
               const float fMinSenseDistance           = constants::ZED_DEFAULT_MINIMUM_DISTANCE,
               const float fMaxSenseDistance           = constants::ZED_DEFAULT_MAXIMUM_DISTANCE,
               const bool bMemTypeGPU                  = false,
               const bool bUseHalfDepthPrecision       = false,
               const unsigned int unCameraSerialNumber = 0);
        ~ZEDCam();
        bool GrabFrame(cv::Mat& cvFrame) override;
        bool GrabFrame(cv::cuda::GpuMat& cvGPUFrame);
        bool GrabDepth(cv::Mat& cvDepth, const bool bRetrieveMeasure = true);
        bool GrabDepth(cv::cuda::GpuMat& cvGPUDepth, const bool bRetrieveMeasure = true);
        bool GrabPointCloud(cv::Mat& cvPointCloud);
        bool GrabPointCloud(cv::cuda::GpuMat& cvGPUPointCloud);
        sl::ERROR_CODE ResetPositionalTracking();
        sl::ERROR_CODE TrackCustomBoxObjects(std::vector<ZedObjectData>& vCustomObjects);
        sl::ERROR_CODE RebootCamera();

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
        bool GetPositionalPose(sl::Pose& slPose);
        bool GetPositionalTrackingEnabled();
        bool GetIMUData(std::vector<double>& vIMUValues);
        sl::SPATIAL_MAPPING_STATE GetSpatialMappingState();
        sl::SPATIAL_MAPPING_STATE ExtractSpatialMapAsync(std::future<sl::Mesh>& fuMeshFuture);
        bool GetObjectDetectionEnabled();
        bool GetObjects(std::vector<sl::ObjectData>& vObjectData);
        bool GetBatchedObjects(std::vector<sl::ObjectsBatch>& vBatchedObjectData);

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        // ZED Camera specific.

        sl::Camera m_slCamera;
        std::shared_mutex m_muCameraMutex;
        sl::InitParameters m_slCameraParams;
        sl::RuntimeParameters m_slRuntimeParams;
        sl::MEASURE m_slDepthMeasureType;
        sl::SensorsData m_slSensorData;
        sl::PositionalTrackingParameters m_slPoseTrackingParams;
        sl::Pose m_slCameraPose;
        sl::SpatialMappingParameters m_slSpatialMappingParams;
        sl::ObjectDetectionParameters m_slObjectDetectionParams;
        sl::BatchParameters m_slObjectDetectionBatchParams;
        sl::Objects m_slDetectedObjects;
        std::vector<sl::ObjectsBatch> m_slDetectedObjectsBatched;
        sl::MEM m_slMemoryType;
        unsigned int m_unCameraSerialNumber;

        // Mats for storing frames and measures.

        sl::Mat m_slFrame;
        sl::Mat m_slDepthImage;
        sl::Mat m_slDepthMeasure;
        sl::Mat m_slPointCloud;

        // Queues and mutexes for scheduling and copying camera frames and data to other threads.

        std::queue<std::reference_wrapper<containers::FrameFetchContainer<cv::Mat&>>> m_qFrameCopySchedule;
        std::queue<std::reference_wrapper<containers::FrameFetchContainer<cv::cuda::GpuMat&>>> m_qGPUFrameCopySchedule;
        std::queue<std::reference_wrapper<containers::DataFetchContainer<std::vector<ZedObjectData>&>>> m_qCustomBoxInjestSchedule;
        std::queue<std::reference_wrapper<containers::DataFetchContainer<sl::Pose&>>> m_qPoseCopySchedule;
        std::queue<std::reference_wrapper<containers::DataFetchContainer<std::vector<double>&>>> m_qIMUDataCopySchedule;
        std::queue<std::reference_wrapper<containers::DataFetchContainer<std::vector<sl::ObjectData>&>>> m_qObjectDataCopySchedule;
        std::queue<std::reference_wrapper<containers::DataFetchContainer<std::vector<sl::ObjectsBatch>&>>> m_qObjectBatchedDataCopySchedule;
        std::shared_mutex m_muPoolScheduleMutex;
        std::mutex m_muFrameCopyMutex;
        std::mutex m_muCustomBoxInjestMutex;
        std::mutex m_muPoseCopyMutex;
        std::mutex m_muIMUDataCopyMutex;
        std::mutex m_muObjectDataCopyMutex;
        std::mutex m_muObjectBatchedDataCopyMutex;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
};
#endif
