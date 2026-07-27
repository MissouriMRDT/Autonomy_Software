/******************************************************************************
 * @brief Defines the ZEDCam class.
 *
 * @file ZEDCam.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-25
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef ZEDCAM_H
#define ZEDCAM_H

#include "../../interfaces/ZEDCamera.hpp"
#include "../../util/threading/RetryTimer.hpp"

/// \cond
#include <tracy/Tracy.hpp>

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
               const bool bEnableRecordingFlag         = false,
               const bool bExportSVORecordingFlag      = false,
               const float fMinSenseDistance           = constants::ZED_DEFAULT_MINIMUM_DISTANCE,
               const float fMaxSenseDistance           = constants::ZED_DEFAULT_MAXIMUM_DISTANCE,
               const bool bMemTypeGPU                  = false,
               const bool bUseHalfDepthPrecision       = false,
               const int nNumFrameRetrievalThreads     = 10,
               const unsigned int unCameraSerialNumber = 0);
        ~ZEDCam();
        sl::ERROR_CODE ResetPositionalTracking() override;
        sl::ERROR_CODE TrackCustomBoxObjects(std::vector<ZedObjectData>& vCustomObjects) override;
        sl::ERROR_CODE RebootCamera() override;

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
        bool GetPositionalTrackingEnabled() override;
        sl::PositionalTrackingStatus GetPositionalTrackingState() override;
        sl::SPATIAL_MAPPING_STATE GetSpatialMappingState() override;
        sl::SPATIAL_MAPPING_STATE ExtractSpatialMapAsync(std::future<sl::Mesh>& fuMeshFuture) override;
        bool GetObjectDetectionEnabled() override;

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        // ZED Camera specific. All of these are touched ONLY on the owning producer
        // thread (in ThreadedContinuousCode() and the Impl* command handlers it drains),
        // so no mutex guards them - single-thread access is guaranteed by construction.

        sl::Camera m_slCamera;
        sl::InitParameters m_slCameraParams;
        sl::RuntimeParameters m_slRuntimeParams;
        sl::RecordingParameters m_slRecordingParams;
        sl::MEASURE m_slDepthMeasureType;
        sl::PositionalTrackingParameters m_slPoseTrackingParams;
        sl::Pose m_slCameraPose;
        sl::Plane m_slFloorPlane;
        sl::Transform m_slFloorTrackingTransform;
        sl::SensorsData m_slSensorsData;
        sl::SpatialMappingParameters m_slSpatialMappingParams;
        sl::ObjectDetectionParameters m_slObjectDetectionParams;
        sl::BatchParameters m_slObjectDetectionBatchParams;
        sl::Objects m_slDetectedObjects;
        std::vector<sl::ObjectsBatch> m_slDetectedObjectsBatched;
        sl::MEM m_slMemoryType;
        sl::MODEL m_slCameraModel;
        float m_fExpectedCameraHeightFromFloorTolerance;
        // Reconnect pacing and edge-triggered open/closed logging. The producer thread never
        // stops itself for a missing camera; it idles, retries on this monotonic timer, and logs
        // only when the open state actually changes so an idling thread cannot flood the log.
        threadutils::RetryTimer m_tmReconnectTimer{constants::CAMERA_RECONNECT_RETRY_INTERVAL};
        bool m_bLastKnownOpenState = true;

        // Camera model string, cached once at open so GetCameraModel() needs no SDK call.
        std::string m_szCameraModelCached;

        // Counts producer iterations so snapshot-pool diagnostics can be logged on a fixed
        // iteration interval. Deliberately NOT a wall-clock modulus: the old queue-toggle reset
        // used one of those and could fire zero times or many times depending on timing.
        unsigned long long m_ullIterationCounter = 0;

        // Track if we should turn on features during camera replug.
        bool m_bEnablePositionalTrackingFlag;
        bool m_bEnableSpatialMappingFlag;
        bool m_bEnableObjectDetectionFlag;

        // Pose tracking offsets. (ZEDSDK is broken and can't handle large translations internally)

        double m_dPoseOffsetX;
        double m_dPoseOffsetY;
        double m_dPoseOffsetZ;
        double m_dPoseOffsetXO;
        double m_dPoseOffsetYO;
        double m_dPoseOffsetZO;

        // Producer-local sl::Mats the SDK retrieves into before we deep-copy and publish.
        sl::Mat m_slFrame;
        sl::Mat m_slDepthImage;
        sl::Mat m_slDepthMeasure;
        sl::Mat m_slPointCloud;

        // Pending async spatial-map extraction. Set by the ExtractSpatialMapAsync command
        // and completed by the producer loop polling the SDK, so all SDK access stays on
        // the owning thread (fixes the old unlocked std::async race).
        std::shared_ptr<std::promise<sl::Mesh>> m_pPendingSpatialMapPromise;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;

        // Owning-thread SDK operations. The public methods above post these to m_cmdQueue
        // (or the reconnect path calls them directly), so they always run on the producer
        // thread and never need a lock.
        sl::ERROR_CODE ImplEnablePositionalTracking(const float fExpectedCameraHeightFromFloorTolerance);
        void ImplDisablePositionalTracking();
        void ImplSetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO);
        sl::ERROR_CODE ImplEnableSpatialMapping();
        void ImplDisableSpatialMapping();
        sl::ERROR_CODE ImplEnableObjectDetection(const bool bEnableBatching);
        void ImplDisableObjectDetection();
        sl::ERROR_CODE ImplResetPositionalTracking();
        sl::ERROR_CODE ImplTrackCustomBoxObjects(const std::vector<sl::CustomBoxObjectData>& vCustomBoxData);
        sl::ERROR_CODE ImplRebootCamera();

        // Producer helpers.
        void LogSnapshotPoolDiagnostics();    // Periodically log snapshot pool misses / ceiling breaches.
        void RetrieveAndPublishData();        // Retrieve+publish every subscribed data type after a good grab.
        void PublishStatus();                 // Build and publish the CameraStatus snapshot.
        void PollPendingSpatialMap();         // Advance any in-flight async spatial-map extraction.
};
#endif
