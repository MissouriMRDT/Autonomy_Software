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

class ZEDCam : public Camera<cv::Mat>, public AutonomyThread<void>
{
    private:
        // Declare public structs that are specific to and used within this class.
        struct FrameFetchContainer;

        // Declare private member variables.

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
        sl::MEM m_slMemoryType;
        sl::Mat m_slFrame;
        std::queue<std::reference_wrapper<FrameFetchContainer>> m_qFrameCopySchedule;
        std::shared_mutex m_muFrameScheduleMutex;
        std::mutex m_muFrameCopyMutex;

        // Declare private methods.

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;    // This does nothing for now. No need for threadpools.

    public:
        // Declare public structs that are specific to and used within this class.
        struct ZedObjectData;

        // Declare public methods and member variables.

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
        // sl::Mat GrabDepth(const bool bRetrieveMeasure);
        // sl::Mat GrabPointCloud();
        sl::ERROR_CODE ResetPositionalTracking();
        sl::ERROR_CODE TrackCustomBoxObjects(std::vector<ZedObjectData>& vCustomObjects);
        sl::ERROR_CODE RebootCamera();

        // Setters for class member variables.
        sl::ERROR_CODE EnablePositionalTracking();
        void DisablePositionalTracking();
        sl::ERROR_CODE SetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO);
        sl::ERROR_CODE EnableSpatialMapping(const int nTimeoutSeconds = 10);
        void DisableSpatialMapping();
        sl::ERROR_CODE EnableObjectDetection(const bool bEnableBatching = false);
        void DisableObjectDetection();

        // Accessors for class member variables.
        bool GetCameraIsOpen() override;
        std::string GetCameraModel();
        unsigned int GetCameraSerial();
        sl::Pose GetPositionalPose(const sl::REFERENCE_FRAME slPositionReference = sl::REFERENCE_FRAME::WORLD);
        bool GetPositionalTrackingEnabled();
        std::vector<double> GetIMUData();
        sl::SPATIAL_MAPPING_STATE GetSpatialMappingState();
        sl::SPATIAL_MAPPING_STATE ExtractSpatialMapAsync(std::future<sl::Mesh>& fuMeshFuture);
        bool GetObjectDetectionEnabled();
        std::vector<sl::ObjectData> GetObjects();
        std::vector<sl::ObjectsBatch> GetBatchedObjects();
};
#endif
