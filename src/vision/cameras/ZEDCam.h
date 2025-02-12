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

#include "../../interfaces/ZEDCamera.hpp"

/// \cond

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
class ZEDCam : public ZEDCamera
{
    public:
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
        std::future<bool> RequestFrameCopy(cv::cuda::GpuMat& cvGPUFrame) override;
        std::future<bool> RequestDepthCopy(cv::Mat& cvDepth, const bool bRetrieveMeasure = true) override;
        std::future<bool> RequestDepthCopy(cv::cuda::GpuMat& cvGPUDepth, const bool bRetrieveMeasure = true) override;
        std::future<bool> RequestPointCloudCopy(cv::Mat& cvPointCloud) override;
        std::future<bool> RequestPointCloudCopy(cv::cuda::GpuMat& cvGPUPointCloud) override;
        sl::ERROR_CODE ResetPositionalTracking() override;
        sl::ERROR_CODE TrackCustomBoxObjects(std::vector<ZedObjectData>& vCustomObjects) override;
        sl::ERROR_CODE RebootCamera() override;
        sl::FUSION_ERROR_CODE SubscribeFusionToCameraUUID(sl::CameraIdentifier& slCameraUUID) override;
        sl::CameraIdentifier PublishCameraToFusion() override;
        sl::FUSION_ERROR_CODE IngestGPSDataToFusion(geoops::GPSCoordinate stNewGPSLocation) override;

        /////////////////////////////////////////
        // Setters for class member variables.
        /////////////////////////////////////////

        sl::ERROR_CODE EnablePositionalTracking(const float fExpectedCameraHeightFromFloorTolerance = constants::ZED_DEFAULT_FLOOR_PLANE_ERROR) override;
        void DisablePositionalTracking() override;
        void SetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO) override;
        sl::ERROR_CODE EnableSpatialMapping() override;
        void DisableSpatialMapping() override;
        sl::ERROR_CODE EnableObjectDetection(const bool bEnableBatching = false) override;
        void DisableObjectDetection() override;

        /////////////////////////////////////////
        // Getters.
        /////////////////////////////////////////

        bool GetCameraIsOpen() override;
        bool GetUsingGPUMem() const override;
        std::string GetCameraModel() override;
        unsigned int GetCameraSerial() override;
        std::future<bool> RequestPositionalPoseCopy(Pose& stPose) override;
        std::future<bool> RequestFusionGeoPoseCopy(sl::GeoPose& slGeoPose) override;
        std::future<bool> RequestFloorPlaneCopy(sl::Plane& slPlane) override;
        bool GetPositionalTrackingEnabled() override;
        sl::PositionalTrackingStatus GetPositionalTrackingState() override;
        sl::FusedPositionalTrackingStatus GetFusedPositionalTrackingState() override;
        sl::SPATIAL_MAPPING_STATE GetSpatialMappingState() override;
        sl::SPATIAL_MAPPING_STATE ExtractSpatialMapAsync(std::future<sl::Mesh>& fuMeshFuture) override;
        bool GetObjectDetectionEnabled() override;
        std::future<bool> RequestObjectsCopy(std::vector<sl::ObjectData>& vObjectData) override;
        std::future<bool> RequestBatchedObjectsCopy(std::vector<sl::ObjectsBatch>& vBatchedObjectData) override;

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        // ZED Camera specific.

        sl::Camera m_slCamera;
        std::shared_mutex m_muCameraMutex;
        sl::InitParameters m_slCameraParams;
        sl::RuntimeParameters m_slRuntimeParams;
        sl::RecordingParameters m_slRecordingParams;
        sl::Fusion m_slFusionInstance;
        std::shared_mutex m_muFusionMutex;
        sl::InitFusionParameters m_slFusionParams;
        sl::MEASURE m_slDepthMeasureType;
        sl::PositionalTrackingParameters m_slPoseTrackingParams;
        sl::PositionalTrackingFusionParameters m_slFusionPoseTrackingParams;
        sl::Pose m_slCameraPose;
        sl::GeoPose m_slFusionGeoPose;
        sl::Plane m_slFloorPlane;
        sl::Transform m_slFloorTrackingTransform;
        sl::SpatialMappingParameters m_slSpatialMappingParams;
        sl::ObjectDetectionParameters m_slObjectDetectionParams;
        sl::BatchParameters m_slObjectDetectionBatchParams;
        sl::Objects m_slDetectedObjects;
        std::vector<sl::ObjectsBatch> m_slDetectedObjectsBatched;
        sl::MEM m_slMemoryType;
        sl::MODEL m_slCameraModel;
        float m_fExpectedCameraHeightFromFloorTolerance;

        // Pose tracking offsets. (ZEDSDK is broken and can't handle large translations internally)

        double m_dPoseOffsetX;
        double m_dPoseOffsetY;
        double m_dPoseOffsetZ;
        double m_dPoseOffsetXO;
        double m_dPoseOffsetYO;
        double m_dPoseOffsetZO;

        // Data from NavBoard.

        geoops::GPSCoordinate m_stCurrentGPSBasedPosition;

        // Mats for storing frames and measures.

        sl::Mat m_slFrame;
        sl::Mat m_slDepthImage;
        sl::Mat m_slDepthMeasure;
        sl::Mat m_slPointCloud;

        // Queues and mutexes for scheduling and copying camera frames and data to other threads.

        std::queue<containers::FrameFetchContainer<cv::cuda::GpuMat>> m_qGPUFrameCopySchedule;
        std::queue<containers::DataFetchContainer<std::vector<ZedObjectData>>> m_qCustomBoxIngestSchedule;
        std::queue<containers::DataFetchContainer<Pose>> m_qPoseCopySchedule;
        std::queue<containers::DataFetchContainer<sl::GeoPose>> m_qGeoPoseCopySchedule;
        std::queue<containers::DataFetchContainer<sl::Plane>> m_qFloorCopySchedule;
        std::queue<containers::DataFetchContainer<std::vector<sl::ObjectData>>> m_qObjectDataCopySchedule;
        std::queue<containers::DataFetchContainer<std::vector<sl::ObjectsBatch>>> m_qObjectBatchedDataCopySchedule;

        // Mutexes for copying frames from the ZEDSDK to the OpenCV Mats.

        std::shared_mutex m_muCustomBoxIngestMutex;
        std::shared_mutex m_muPoseCopyMutex;
        std::shared_mutex m_muGeoPoseCopyMutex;
        std::shared_mutex m_muFloorCopyMutex;
        std::shared_mutex m_muObjectDataCopyMutex;
        std::shared_mutex m_muObjectBatchedDataCopyMutex;

        // Atomic flags for checking if data is queued.

        std::atomic<bool> m_bNormalFramesQueued;
        std::atomic<bool> m_bDepthFramesQueued;
        std::atomic<bool> m_bPointCloudsQueued;
        std::atomic<bool> m_bPosesQueued;
        std::atomic<bool> m_bGeoPosesQueued;
        std::atomic<bool> m_bFloorsQueued;
        std::atomic<bool> m_bObjectsQueued;
        std::atomic<bool> m_bBatchedObjectsQueued;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
};
#endif
