/******************************************************************************
 * @brief Implements the ZEDCam class.
 *
 * @file ZEDCam.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "ZEDCam.h"
#include "../../AutonomyLogging.h"
#include "../../util/NumberOperations.hpp"
#include "../../util/vision/ImageOperations.hpp"

/******************************************************************************
 * @brief Construct a new Zed Cam:: Zed Cam object.
 *
 * @param nPropResolutionX - X res of camera. Must be smaller than ZED_BASE_RESOLUTION.
 * @param nPropResolutionY - Y res of camera. Must be smaller than ZED_BASE_RESOLUTION.
 * @param nPropFramesPerSecond - FPS camera is running at.
 * @param dPropHorizontalFOV - The horizontal field of view.
 * @param dPropVerticalFOV - The vertical field of view.
 * @param bEnableRecordingFlag - Whether or not this camera should be recorded.
 * @param fMinSenseDistance - The minimum distance to include in depth measures.
 * @param fMaxSenseDistance - The maximum distance to include in depth measures.
 * @param bMemTypeGPU - Whether or not to use the GPU memory for operations.
 * @param bUseHalfPrecision - Whether or not to use a float16 instead of float32 for depth measurements.
 * @param bEnableFusionMaster - Enables ZEDSDK Fusion integration for this camera. This camera will serve as the master instance for all fusion functions.
 * @param nNumFrameRetrievalThreads - The number of threads to use for copying frames/data to requests.
 * @param unCameraSerialNumber - The serial number of the camera to open.
 *
 * @note Do not set bEnableFusionMaster to true if you want to subscribe the camera to another camera! Only one camera should have Fusion enabled!
 *      To subscribe a camera to the camera running the master fusion instance, use the GetFusionInstance() and GetCameraSerial() functions.
 *      Refer to Fusion documentation for info on working with fusion functions: https://www.stereolabs.com/docs/api/classsl_1_1Fusion.html
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
ZEDCam::ZEDCam(const int nPropResolutionX,
               const int nPropResolutionY,
               const int nPropFramesPerSecond,
               const double dPropHorizontalFOV,
               const double dPropVerticalFOV,
               const bool bEnableRecordingFlag,
               const bool bExportSVORecordingFlag,
               const float fMinSenseDistance,
               const float fMaxSenseDistance,
               const bool bMemTypeGPU,
               const bool bUseHalfDepthPrecision,
               const int nNumFrameRetrievalThreads,
               const unsigned int unCameraSerialNumber) :
    ZEDCamera(nPropResolutionX,
              nPropResolutionY,
              nPropFramesPerSecond,
              dPropHorizontalFOV,
              dPropVerticalFOV,
              bEnableRecordingFlag,
              bMemTypeGPU,
              bUseHalfDepthPrecision,
              nNumFrameRetrievalThreads,
              unCameraSerialNumber)
{
    // Assign member variables.
    bMemTypeGPU ? m_slMemoryType = sl::MEM::GPU : m_slMemoryType = sl::MEM::CPU;
    bUseHalfDepthPrecision ? m_slDepthMeasureType = sl::MEASURE::DEPTH_U16_MM : m_slDepthMeasureType = sl::MEASURE::DEPTH;
    m_dPoseOffsetX                  = 0.0;
    m_dPoseOffsetY                  = 0.0;
    m_dPoseOffsetZ                  = 0.0;
    m_dPoseOffsetXO                 = 0.0;
    m_dPoseOffsetYO                 = 0.0;
    m_dPoseOffsetZO                 = 0.0;
    m_bEnablePositionalTrackingFlag = false;
    m_bEnableSpatialMappingFlag     = false;
    m_bEnableObjectDetectionFlag    = false;
    m_slCameraModel                 = sl::MODEL::LAST;
    m_szCameraModelCached           = "NOT_OPENED";

    // Setup camera params.
    m_slCameraParams.camera_resolution      = constants::ZED_BASE_RESOLUTION;
    m_slCameraParams.camera_fps             = nPropFramesPerSecond;
    m_slCameraParams.coordinate_units       = constants::ZED_MEASURE_UNITS;
    m_slCameraParams.coordinate_system      = constants::ZED_COORD_SYSTEM;
    m_slCameraParams.sdk_verbose            = constants::ZED_SDK_VERBOSE;
    m_slCameraParams.depth_mode             = constants::ZED_DEPTH_MODE;
    m_slCameraParams.depth_minimum_distance = fMinSenseDistance;
    m_slCameraParams.depth_maximum_distance = fMaxSenseDistance;
    m_slCameraParams.depth_stabilization    = constants::ZED_DEPTH_STABILIZATION;
    // Only set serial number if necessary.
    if (unCameraSerialNumber != static_cast<unsigned int>(0))
    {
        m_slCameraParams.input.setFromSerialNumber(unCameraSerialNumber);
    }

    // Setup camera runtime params.
    m_slRuntimeParams.enable_fill_mode = constants::ZED_SENSING_FILL;
    // Setup SVO recording parameters.
    m_slRecordingParams.compression_mode = constants::ZED_SVO_COMPRESSION;
    m_slRecordingParams.bitrate          = constants::ZED_SVO_BITRATE;

    // Setup positional tracking parameters.
    m_slPoseTrackingParams.mode                  = constants::ZED_POSETRACK_MODE;
    m_slPoseTrackingParams.enable_area_memory    = constants::ZED_POSETRACK_AREA_MEMORY;
    m_slPoseTrackingParams.enable_pose_smoothing = constants::ZED_POSETRACK_POSE_SMOOTHING;
    m_slPoseTrackingParams.set_floor_as_origin   = constants::ZED_POSETRACK_FLOOR_IS_ORIGIN;
    m_slPoseTrackingParams.enable_imu_fusion     = constants::ZED_POSETRACK_ENABLE_IMU_FUSION;
    m_slPoseTrackingParams.depth_min_range       = constants::ZED_POSETRACK_USABLE_DEPTH_MIN;
    m_slPoseTrackingParams.set_gravity_as_origin = constants::ZED_POSETRACK_USE_GRAVITY_ORIGIN;

    // Setup spatial mapping parameters.
    m_slSpatialMappingParams.map_type          = constants::ZED_MAPPING_TYPE;
    m_slSpatialMappingParams.resolution_meter  = constants::ZED_MAPPING_RESOLUTION_METER;
    m_slSpatialMappingParams.save_texture      = true;
    m_slSpatialMappingParams.use_chunk_only    = constants::ZED_MAPPING_USE_CHUNK_ONLY;
    m_slSpatialMappingParams.stability_counter = constants::ZED_MAPPING_STABILITY_COUNTER;
    // Set or auto-set max depth range for mapping.
    if (constants::ZED_MAPPING_RANGE_METER <= 0)
    {
        // Automatically guess the best mapping depth range.
        m_slSpatialMappingParams.range_meter = m_slSpatialMappingParams.getRecommendedRange(constants::ZED_MAPPING_RESOLUTION_METER, m_slCamera);
    }
    else
    {
        // Manually set.
        m_slSpatialMappingParams.range_meter = constants::ZED_MAPPING_RANGE_METER;
    }

    // Setup object detection/tracking parameters.
    m_slObjectDetectionParams.detection_model      = sl::OBJECT_DETECTION_MODEL::CUSTOM_BOX_OBJECTS;
    m_slObjectDetectionParams.enable_tracking      = constants::ZED_OBJDETECTION_TRACK_OBJ;
    m_slObjectDetectionParams.enable_segmentation  = constants::ZED_OBJDETECTION_SEGMENTATION;
    m_slObjectDetectionParams.filtering_mode       = constants::ZED_OBJDETECTION_FILTERING;
    m_slObjectDetectionParams.prediction_timeout_s = constants::ZED_OBJDETECTION_TRACKING_PREDICTION_TIMEOUT;
    // Setup object detection/tracking batch parameters.
    m_slObjectDetectionBatchParams.enable            = false;
    m_slObjectDetectionBatchParams.id_retention_time = constants::ZED_OBJDETECTION_BATCH_RETENTION_TIME;
    m_slObjectDetectionBatchParams.latency           = constants::ZED_OBJDETECTION_BATCH_LATENCY;
    m_slObjectDetectionParams.batch_parameters       = m_slObjectDetectionBatchParams;

    // Attempt to open camera.
    sl::ERROR_CODE slReturnCode = m_slCamera.open(m_slCameraParams);
    // Check if the camera was successfully opened.
    if (m_slCamera.isOpened())
    {
        // Update camera serial number if camera was opened with autodetect.
        m_unCameraSerialNumber = m_slCamera.getCameraInformation().serial_number;
        // Update camera model.
        m_slCameraModel = m_slCamera.getCameraInformation().camera_model;
        // Cache the model string once so GetCameraModel() needs no SDK call or lock later.
        m_szCameraModelCached = sl::toString(m_slCameraModel).get();
        // Check if the camera should record and output an SVO file.
        if (bExportSVORecordingFlag)
        {
            // Now that camera is opened get camera name and construct path. Use the cached model
            // string directly: GetCameraModel() reads the status publisher, which the producer
            // thread has not published to yet, so it would return "NOT_OPENED" here.
            std::string szSVOFilePath          = constants::LOGGING_OUTPUT_PATH_ABSOLUTE + "/" + logging::g_szProgramStartTimeString + "/" + m_szCameraModelCached + "_" +
                                                 std::to_string(this->GetCameraSerial());
            m_slRecordingParams.video_filename = szSVOFilePath.c_str();
            // Enable recording.
            sl::ERROR_CODE slReturnCode = m_slCamera.enableRecording(m_slRecordingParams);
            // Check if recording was enabled successfully.
            if (slReturnCode == sl::ERROR_CODE::SUCCESS)
            {
                // Submit logger message.
                LOG_DEBUG(logging::g_qSharedLogger,
                          "Successfully enabled SVO recording for {} ZED stereo camera with serial number {}.",
                          m_szCameraModelCached,
                          m_unCameraSerialNumber);
            }
            else
            {
                // Submit logger message.
                LOG_ERROR(logging::g_qSharedLogger,
                          "Failed to enable SVO recording for {} ZED stereo camera with serial number {}. sl::ERROR_CODE is {}",
                          m_szCameraModelCached,
                          m_unCameraSerialNumber,
                          sl::toString(slReturnCode).c_str());
            }
        }

        // Submit logger message.
        LOG_INFO(logging::g_qSharedLogger, "{} ZED stereo camera with serial number {} has been successfully opened.", m_szCameraModelCached, m_unCameraSerialNumber);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Unable to open ZED stereo camera {} ({})! sl::ERROR_CODE is: {}",
                  sl::toString(m_slCameraModel).get(),
                  m_unCameraSerialNumber,
                  sl::toString(slReturnCode).get());
    }

    // Set max FPS of the ThreadedContinuousCode method.
    this->SetMainThreadIPSLimit(nPropFramesPerSecond);

    // Publish an initial status snapshot. The producer thread is not running yet, so this is the
    // only thread touching the SDK and the call is safe here. Without it every status accessor
    // (notably GetCameraModel(), which other components read while being constructed) would see a
    // null snapshot and report defaults until the producer's first iteration.
    this->PublishStatus();
}

/******************************************************************************
 * @brief Destroy the Zed Cam:: Zed Cam object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
ZEDCam::~ZEDCam()
{
    // Stop threaded code.
    this->RequestStop();
    this->Join();

    // Shut down the command queue so any command posted after the producer thread stopped
    // is cancelled (its future resolves with an error) rather than left stranded.
    m_cmdQueue.Shutdown();

    // Close the ZEDCam. Safe here: the producer thread is joined, so no other thread touches the SDK.
    m_slCamera.close();

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "ZED stereo camera with serial number {} has been successfully closed.", m_unCameraSerialNumber);
}

/******************************************************************************
 * @brief The code inside this private method runs in a separate thread, but still
 *      has access to this*. Each iteration drains the command queue (so every SDK
 *      mutation happens on this thread), calls the grab() function of the ZEDSDK,
 *      then retrieves only the data types that currently have subscribers and
 *      publishes a deep-copied snapshot of each. Consumers read those snapshots on
 *      their own schedule, so this loop never waits on them.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-01
 ******************************************************************************/
void ZEDCam::ThreadedContinuousCode()
{
    // 1. Control channel in. Every SDK-mutating command posted by a foreign thread runs
    //    here, on this thread, so all SDK access is single threaded by construction.
    m_cmdQueue.DrainAll();

    // 2. Handle not-open / reconnect. This thread NEVER stops itself for a missing camera: it
    //    idles and keeps retrying, so a camera that is absent at startup or unplugged at runtime
    //    is recovered automatically and startup ordering never matters. Only RequestStop() from
    //    the owner ends this thread.
    if (!m_slCamera.isOpened())
    {
        // Log the open -> closed transition exactly once instead of every iteration.
        if (m_bLastKnownOpenState)
        {
            // Remember the new state so we do not log again until it changes back.
            m_bLastKnownOpenState = false;
            // Submit logger message.
            LOG_CRITICAL(logging::g_qSharedLogger,
                         "ZED stereo camera with serial number {} is not open. Retrying every {} ms until it connects; this thread will keep running.",
                         m_unCameraSerialNumber,
                         constants::CAMERA_RECONNECT_RETRY_INTERVAL.count());
        }

        // Rate limit reopen attempts on a monotonic deadline.
        if (m_tmReconnectTimer.Ready())
        {
            // Attempt to reopen camera.
            sl::ERROR_CODE slReturnCode = m_slCamera.open(m_slCameraParams);

            // Check if camera was reopened.
            if (slReturnCode == sl::ERROR_CODE::SUCCESS)
            {
                // Record the closed -> open transition so the recovery is logged once.
                m_bLastKnownOpenState = true;
                // Refresh the cached model string (owning thread) in case this is the first successful open.
                m_slCameraModel       = m_slCamera.getCameraInformation().camera_model;
                m_szCameraModelCached = sl::toString(m_slCameraModel).get();

                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "ZED stereo camera with serial number {} has been reconnected and reopened!", m_unCameraSerialNumber);

                // Check if positional tracking was enabled. Call the owning-thread Impl directly.
                if (m_bEnablePositionalTrackingFlag)
                {
                    slReturnCode = this->ImplEnablePositionalTracking(m_fExpectedCameraHeightFromFloorTolerance);

                    // Check if positional tracking was re-enabled successfully.
                    if (slReturnCode != sl::ERROR_CODE::SUCCESS)
                    {
                        // Submit logger message.
                        LOG_ERROR(logging::g_qSharedLogger,
                                  "After reopening ZED stereo camera with serial number {}, positional tracking failed to reinitialize. sl::ERROR_CODE is: {}",
                                  m_unCameraSerialNumber,
                                  sl::toString(slReturnCode).get());
                    }
                }
                // Check if spatial mapping was enabled.
                if (m_bEnableSpatialMappingFlag)
                {
                    slReturnCode = this->ImplEnableSpatialMapping();
                    // Check if spatial mapping was re-enabled successfully.
                    if (slReturnCode != sl::ERROR_CODE::SUCCESS)
                    {
                        // Submit logger message.
                        LOG_ERROR(logging::g_qSharedLogger,
                                  "After reopening ZED stereo camera with serial number {}, spatial mapping failed to reinitialize. sl::ERROR_CODE is: {}",
                                  m_unCameraSerialNumber,
                                  sl::toString(slReturnCode).get());
                    }
                }
                // Check if object detection was enabled.
                if (m_bEnableObjectDetectionFlag)
                {
                    slReturnCode = this->ImplEnableObjectDetection(m_slObjectDetectionBatchParams.enable);

                    // Check if object detection was re-enabled successfully.
                    if (slReturnCode != sl::ERROR_CODE::SUCCESS)
                    {
                        // Submit logger message.
                        LOG_ERROR(logging::g_qSharedLogger,
                                  "After reopening ZED stereo camera with serial number {}, object detection failed to reinitialize. sl::ERROR_CODE is: {}",
                                  m_unCameraSerialNumber,
                                  sl::toString(slReturnCode).get());
                    }
                }
            }
            else
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Attempt to reopen ZED stereo camera with serial number {} has failed! Trying again in {} ms...",
                            m_unCameraSerialNumber,
                            constants::CAMERA_RECONNECT_RETRY_INTERVAL.count());
            }
        }

        // Camera is not open: release any in-flight spatial-map extraction so its caller is not stranded,
        // publish the updated status, and produce no data this iteration.
        if (m_pPendingSpatialMapPromise != nullptr)
        {
            // Fulfill with an empty mesh since the camera is unavailable.
            m_pPendingSpatialMapPromise->set_value(sl::Mesh());
            m_pPendingSpatialMapPromise.reset();
        }
        this->PublishStatus();
        return;
    }

    // 3. One grab.
    sl::ERROR_CODE slReturnCode = m_slCamera.grab(m_slRuntimeParams);
    if (slReturnCode != sl::ERROR_CODE::SUCCESS)
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Unable to update stereo camera {} ({}) frames, measurements, and sensors! sl::ERROR_CODE is: {}. Closing camera...",
                  m_szCameraModelCached,
                  m_unCameraSerialNumber,
                  sl::toString(slReturnCode).get());

        // Release camera resources. Publish NO data; the last good snapshot stays valid and
        // consumers can detect staleness via its sequence number and publish time.
        m_slCamera.close();
        // Release any in-flight spatial-map extraction so its caller is not stranded.
        if (m_pPendingSpatialMapPromise != nullptr)
        {
            // Fulfill with an empty mesh since the camera just closed.
            m_pPendingSpatialMapPromise->set_value(sl::Mesh());
            m_pPendingSpatialMapPromise.reset();
        }
        // Publish the updated (closed) status.
        this->PublishStatus();
        return;
    }

    // 4. Data channel out. Retrieve only what someone is subscribed to, deep copy once, publish.
    this->RetrieveAndPublishData();

    // 5. Advance any in-flight async spatial-map extraction (all SDK access stays on this thread).
    this->PollPendingSpatialMap();

    // 6. Status out.
    this->PublishStatus();

    // 7. Periodically surface snapshot pool health.
    this->LogSnapshotPoolDiagnostics();
}

/******************************************************************************
 * @brief Periodically log the snapshot pool miss counts for every publisher. After
 *      warmup these should be flat: a steadily rising miss count means a consumer is
 *      holding snapshots longer than expected, and a breached growth ceiling almost
 *      always means a consumer is leaking them outright.
 *
 *      The interval is counted in producer iterations rather than wall-clock seconds
 *      so it fires exactly once per interval regardless of frame rate or scheduling.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void ZEDCam::LogSnapshotPoolDiagnostics()
{
    // Count this iteration and bail out unless we have reached the logging interval.
    if (++m_ullIterationCounter % constants::ZED_POOL_DIAGNOSTICS_INTERVAL != 0)
    {
        // Not time to log yet.
        return;
    }

    // Sum misses across every channel so one number answers "is the pooling healthy?".
    const size_t siTotalMisses = m_pubFrameCPU.GetPoolMisses() + m_pubFrameGPU.GetPoolMisses() + m_pubDepthMeasureCPU.GetPoolMisses() +
                                 m_pubDepthMeasureGPU.GetPoolMisses() + m_pubDepthImageCPU.GetPoolMisses() + m_pubDepthImageGPU.GetPoolMisses() +
                                 m_pubPointCloudCPU.GetPoolMisses() + m_pubPointCloudGPU.GetPoolMisses() + m_pubPose.GetPoolMisses() + m_pubFloorPlane.GetPoolMisses() +
                                 m_pubSensors.GetPoolMisses() + m_pubObjects.GetPoolMisses() + m_pubBatchedObjects.GetPoolMisses() + m_pubStatus.GetPoolMisses();

    // A breached ceiling on any channel is a much stronger signal than a few misses.
    const bool bAnyCeilingBreached = m_pubFrameCPU.GetGrowthCeilingBreached() || m_pubFrameGPU.GetGrowthCeilingBreached() ||
                                     m_pubDepthMeasureCPU.GetGrowthCeilingBreached() || m_pubDepthMeasureGPU.GetGrowthCeilingBreached() ||
                                     m_pubDepthImageCPU.GetGrowthCeilingBreached() || m_pubDepthImageGPU.GetGrowthCeilingBreached() ||
                                     m_pubPointCloudCPU.GetGrowthCeilingBreached() || m_pubPointCloudGPU.GetGrowthCeilingBreached() ||
                                     m_pubPose.GetGrowthCeilingBreached() || m_pubFloorPlane.GetGrowthCeilingBreached() || m_pubSensors.GetGrowthCeilingBreached() ||
                                     m_pubObjects.GetGrowthCeilingBreached() || m_pubBatchedObjects.GetGrowthCeilingBreached() || m_pubStatus.GetGrowthCeilingBreached();

    // Escalate to an error if a pool has grown past its ceiling, otherwise log at debug level.
    if (bAnyCeilingBreached)
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Stereo camera {} ({}) snapshot pool grew past its ceiling (total misses: {}). A consumer is most likely holding or leaking snapshots.",
                  m_szCameraModelCached,
                  m_unCameraSerialNumber,
                  siTotalMisses);
    }
    else
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger,
                  "Stereo camera {} ({}) snapshot pool total misses: {} (flat after warmup is healthy).",
                  m_szCameraModelCached,
                  m_unCameraSerialNumber,
                  siTotalMisses);
    }
}

/******************************************************************************
 * @brief Retrieve every data type that currently has a subscriber, deep copy each
 *      into a pooled snapshot, and publish it. Runs only on the owning thread after
 *      a successful grab. Nothing is retrieved for a type with no subscribers, and
 *      no aliasing view of an SDK buffer is ever published (fixes B1).
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void ZEDCam::RetrieveAndPublishData()
{
    // Whether the CPU or GPU channel for a given data type currently has demand.
    auto AnySub = [this](pubsub::Publisher<cv::Mat>& pubCPU, pubsub::Publisher<cv::cuda::GpuMat>& pubGPU)
    {
        return (m_slMemoryType == sl::MEM::CPU) ? pubCPU.HasSubscribers() : pubGPU.HasSubscribers();
    };
    // Deep copy the source sl::Mat into a pooled snapshot on the active memory channel and publish it.
    auto PublishMat = [this](pubsub::Publisher<cv::Mat>& pubCPU, pubsub::Publisher<cv::cuda::GpuMat>& pubGPU, sl::Mat& slSource)
    {
        // Publish to whichever memory channel this camera is configured for.
        if (m_slMemoryType == sl::MEM::CPU)
        {
            // Acquire a pooled slot and DEEP COPY the wrapped SDK buffer into it (never publish the alias).
            std::shared_ptr<pubsub::Snapshot<cv::Mat>> pSlot = pubCPU.Acquire();
            imgops::ConvertSLMatToCVMat(slSource).copyTo(pSlot->tData);
            pubCPU.Publish(std::move(pSlot));
        }
        else
        {
            // Acquire a pooled device slot and DEEP COPY on the GPU (device-to-device copyTo).
            std::shared_ptr<pubsub::Snapshot<cv::cuda::GpuMat>> pSlot = pubGPU.Acquire();
            imgops::ConvertSLMatToGPUMat(slSource).copyTo(pSlot->tData);
            pubGPU.Publish(std::move(pSlot));
        }
    };

    // Prebuild the retrieval resolution once.
    const sl::Resolution slResolution(m_nPropResolutionX, m_nPropResolutionY);
    sl::ERROR_CODE slReturnCode;

    // ---- Normal BGRA frame ----
    if (AnySub(m_pubFrameCPU, m_pubFrameGPU))
    {
        // Retrieve the image into the producer-local sl::Mat.
        slReturnCode = m_slCamera.retrieveImage(m_slFrame, constants::ZED_RETRIEVE_VIEW, m_slMemoryType, slResolution);
        if (slReturnCode == sl::ERROR_CODE::SUCCESS)
        {
            // Deep copy and publish.
            PublishMat(m_pubFrameCPU, m_pubFrameGPU, m_slFrame);
        }
        else
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger,
                        "Unable to retrieve new frame image for stereo camera {} ({})! sl::ERROR_CODE is: {}",
                        m_szCameraModelCached,
                        m_unCameraSerialNumber,
                        sl::toString(slReturnCode).get());
        }
    }

    // ---- Depth measure ----
    if (AnySub(m_pubDepthMeasureCPU, m_pubDepthMeasureGPU))
    {
        // Retrieve the depth measure into the producer-local sl::Mat.
        slReturnCode = m_slCamera.retrieveMeasure(m_slDepthMeasure, m_slDepthMeasureType, m_slMemoryType, slResolution);
        if (slReturnCode == sl::ERROR_CODE::SUCCESS)
        {
            // Deep copy and publish.
            PublishMat(m_pubDepthMeasureCPU, m_pubDepthMeasureGPU, m_slDepthMeasure);
        }
        else
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger,
                        "Unable to retrieve new depth measure for stereo camera {} ({})! sl::ERROR_CODE is: {}",
                        m_szCameraModelCached,
                        m_unCameraSerialNumber,
                        sl::toString(slReturnCode).get());
        }
    }

    // ---- Depth grayscale image ----
    if (AnySub(m_pubDepthImageCPU, m_pubDepthImageGPU))
    {
        // Retrieve the depth image into the producer-local sl::Mat.
        slReturnCode = m_slCamera.retrieveImage(m_slDepthImage, sl::VIEW::DEPTH, m_slMemoryType, slResolution);
        if (slReturnCode == sl::ERROR_CODE::SUCCESS)
        {
            // Deep copy and publish.
            PublishMat(m_pubDepthImageCPU, m_pubDepthImageGPU, m_slDepthImage);
        }
        else
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger,
                        "Unable to retrieve new depth image for stereo camera {} ({})! sl::ERROR_CODE is: {}",
                        m_szCameraModelCached,
                        m_unCameraSerialNumber,
                        sl::toString(slReturnCode).get());
        }
    }

    // ---- Point cloud ----
    if (AnySub(m_pubPointCloudCPU, m_pubPointCloudGPU))
    {
        // Retrieve the point cloud into the producer-local sl::Mat.
        slReturnCode = m_slCamera.retrieveMeasure(m_slPointCloud, sl::MEASURE::XYZBGRA, m_slMemoryType, slResolution);
        if (slReturnCode == sl::ERROR_CODE::SUCCESS)
        {
            // Deep copy and publish.
            PublishMat(m_pubPointCloudCPU, m_pubPointCloudGPU, m_slPointCloud);
        }
        else
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger,
                        "Unable to retrieve new point cloud for stereo camera {} ({})! sl::ERROR_CODE is: {}",
                        m_szCameraModelCached,
                        m_unCameraSerialNumber,
                        sl::toString(slReturnCode).get());
        }
    }

    // ---- Pose and floor plane (require positional tracking) ----
    if (m_slCamera.isPositionalTrackingEnabled())
    {
        // ---- Pose ----
        if (m_pubPose.HasSubscribers())
        {
            // Get the world-frame pose from the camera.
            sl::POSITIONAL_TRACKING_STATE slPoseTrackReturnCode = m_slCamera.getPosition(m_slCameraPose, sl::REFERENCE_FRAME::WORLD);
            if (slPoseTrackReturnCode == sl::POSITIONAL_TRACKING_STATE::OK)
            {
                // Rotate the ZED position coordinate frame to realign with the UTM global coordinate frame.
                std::vector<numops::CoordinatePoint<double>> vPointCloud;
                vPointCloud.emplace_back(m_slCameraPose.getTranslation().x, m_slCameraPose.getTranslation().y, m_slCameraPose.getTranslation().z);
                // Get angle realignments.
                double dNewXO = numops::InputAngleModulus<double>(m_slCameraPose.getEulerAngles(false).x + m_dPoseOffsetXO, 0.0, 360.0);
                double dNewYO = numops::InputAngleModulus<double>(m_slCameraPose.getEulerAngles(false).y + m_dPoseOffsetYO, 0.0, 360.0);
                double dNewZO = numops::InputAngleModulus<double>(m_slCameraPose.getEulerAngles(false).z + m_dPoseOffsetZO, 0.0, 360.0);
                // Rotate coordinate frame.
                numops::CoordinateFrameRotate3D(vPointCloud, m_dPoseOffsetXO, m_dPoseOffsetYO, m_dPoseOffsetZO);
                // Repack values into pose.
                Pose stPose(vPointCloud[0].tX + m_dPoseOffsetX, vPointCloud[0].tY + m_dPoseOffsetY, vPointCloud[0].tZ + m_dPoseOffsetZ, dNewXO, dNewYO, dNewZO);

                // ISSUE NOTE: Might be in the future if we ever change our coordinate system on the ZED. This can be used to fix the directions of the Pose's coordinate
                // system.
                // // Check ZED coordinate system.
                // switch (m_slCameraParams.coordinate_system)
                // {
                //     case sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP:
                //     {
                //         // Realign based in the signedness of this coordinate system. Z is backwards.
                //         stPose.stTranslation.dZ *= -1;
                //         break;
                //     }
                //     default:
                //     {
                //         // No need to flip signs for other coordinate systems.
                //         break;
                //     }
                // }

                // Acquire a pooled slot, store the realigned pose, and publish.
                std::shared_ptr<pubsub::Snapshot<Pose>> pSlot = m_pubPose.Acquire();
                pSlot->tData                                  = stPose;
                m_pubPose.Publish(std::move(pSlot));
            }
            else
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Unable to retrieve new positional tracking pose for stereo camera {} ({})! sl::POSITIONAL_TRACKING_STATE is: {}",
                            m_szCameraModelCached,
                            m_unCameraSerialNumber,
                            sl::toString(slPoseTrackReturnCode).get());
            }
        }

        // ---- Floor plane ----
        if (m_pubFloorPlane.HasSubscribers())
        {
            // Find the current floor plane relative to the camera pose.
            slReturnCode = m_slCamera.findFloorPlane(m_slFloorPlane,
                                                     m_slFloorTrackingTransform,
                                                     m_slCameraPose.getTranslation().y,
                                                     m_slCameraPose.getRotationMatrix(),
                                                     m_fExpectedCameraHeightFromFloorTolerance);
            if (slReturnCode == sl::ERROR_CODE::SUCCESS)
            {
                // Acquire a pooled slot, deep copy the plane, and publish.
                std::shared_ptr<pubsub::Snapshot<sl::Plane>> pSlot = m_pubFloorPlane.Acquire();
                pSlot->tData                                       = sl::Plane(m_slFloorPlane);
                m_pubFloorPlane.Publish(std::move(pSlot));
            }
            else
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Unable to retrieve new floor plane for stereo camera {} ({})! sl::ERROR_CODE is: {}",
                            m_szCameraModelCached,
                            m_unCameraSerialNumber,
                            sl::toString(slReturnCode).get());
            }
        }
    }

    // ---- Sensors ----
    if (m_pubSensors.HasSubscribers())
    {
        // Get the IMU, barometer, magnetometer, and temperature sensor info from the camera.
        slReturnCode = m_slCamera.getSensorsData(m_slSensorsData, sl::TIME_REFERENCE::CURRENT);
        if (slReturnCode == sl::ERROR_CODE::SUCCESS)
        {
            // Acquire a pooled slot, deep copy the sensor data, and publish.
            std::shared_ptr<pubsub::Snapshot<sl::SensorsData>> pSlot = m_pubSensors.Acquire();
            pSlot->tData                                             = sl::SensorsData(m_slSensorsData);
            m_pubSensors.Publish(std::move(pSlot));
        }
        else
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger,
                        "Unable to retrieve sensor data for stereo camera {} ({})! sl::ERROR_CODE is: {}",
                        m_szCameraModelCached,
                        m_unCameraSerialNumber,
                        sl::toString(slReturnCode).get());
        }
    }

    // ---- Objects and batched objects (require object detection) ----
    if (m_slCamera.isObjectDetectionEnabled())
    {
        // ---- Objects ----
        if (m_pubObjects.HasSubscribers())
        {
            // Get updated objects from camera.
            slReturnCode = m_slCamera.retrieveObjects(m_slDetectedObjects);
            if (slReturnCode == sl::ERROR_CODE::SUCCESS)
            {
                // Acquire a pooled slot for the object list.
                std::shared_ptr<pubsub::Snapshot<std::vector<sl::ObjectData>>> pSlot = m_pubObjects.Acquire();
                // Destroy whatever this recycled slot still holds first. Its sl::Mat masks own
                // their memory (see below) and sl::Mat's assignment operator is a SHALLOW copy, so
                // assigning over them would overwrite the owning handles and leak their buffers.
                // Clearing runs ~Mat(), which frees the owned memory properly.
                pSlot->tData.clear();
                // Copy the object list. Every mask in the copy is now a shallow, non-owning
                // reference to the ZED SDK's internal buffer.
                pSlot->tData = m_slDetectedObjects.object_list;
                // Replace each shared mask with a deep copy that this snapshot owns, so the next
                // retrieveObjects() cannot overwrite pixels a consumer is still reading. This is
                // the same aliasing hazard the image channels avoid with copyTo().
                for (sl::ObjectData& slObject : pSlot->tData)
                {
                    // Only masks that actually hold memory need cloning.
                    if (slObject.mask.isInit())
                    {
                        // Deep copy into an owning Mat, then hand that ownership to the snapshot.
                        // sl::Mat::clone() copies the pixels and marks the destination as the
                        // memory owner; move() transfers those attributes without another copy.
                        sl::Mat slOwnedMask;
                        slOwnedMask.clone(slObject.mask);
                        slOwnedMask.move(slObject.mask);
                    }
                }
                m_pubObjects.Publish(std::move(pSlot));
            }
            else
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Unable to retrieve new object data for stereo camera {} ({})! sl::ERROR_CODE is: {}",
                            m_szCameraModelCached,
                            m_unCameraSerialNumber,
                            sl::toString(slReturnCode).get());
            }
        }

        // ---- Batched objects ----
        if (m_slObjectDetectionBatchParams.enable && m_pubBatchedObjects.HasSubscribers())
        {
            // Get updated batched objects from camera.
            slReturnCode = m_slCamera.getObjectsBatch(m_slDetectedObjectsBatched);
            if (slReturnCode == sl::ERROR_CODE::SUCCESS)
            {
                // Acquire a pooled slot, deep copy the batched objects, and publish.
                std::shared_ptr<pubsub::Snapshot<std::vector<sl::ObjectsBatch>>> pSlot = m_pubBatchedObjects.Acquire();
                pSlot->tData                                                           = m_slDetectedObjectsBatched;
                m_pubBatchedObjects.Publish(std::move(pSlot));
            }
            else
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Unable to retrieve new batched object data for stereo camera {} ({})! sl::ERROR_CODE is: {}",
                            m_szCameraModelCached,
                            m_unCameraSerialNumber,
                            sl::toString(slReturnCode).get());
            }
        }
    }
}

/******************************************************************************
 * @brief Build the frequently-polled camera status and publish it once per
 *      iteration. Runs on the owning thread, so the status accessors become
 *      lock-free reads of the newest snapshot.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void ZEDCam::PublishStatus()
{
    // Build the status from the current SDK state (legal here, on the owning thread).
    CameraStatus stStatus;
    stStatus.bCameraIsOpen = m_slCamera.isOpened();
    if (stStatus.bCameraIsOpen)
    {
        // Capture the full positional tracking status once.
        stStatus.stPositionalTrackingStatus = m_slCamera.getPositionalTrackingStatus();
        stStatus.bPositionalTrackingEnabled = m_slCamera.isPositionalTrackingEnabled() && stStatus.stPositionalTrackingStatus.odometry_status == sl::ODOMETRY_STATUS::OK;
        stStatus.bObjectDetectionEnabled    = m_slCamera.isObjectDetectionEnabled();
        stStatus.eSpatialMappingState       = m_slCamera.getSpatialMappingState();
        // Carry the cached model string so GetCameraModel() is a lock-free snapshot read.
        stStatus.szCameraModel = m_szCameraModelCached;
    }

    // Publish the status snapshot.
    std::shared_ptr<pubsub::Snapshot<CameraStatus>> pSlot = m_pubStatus.Acquire();
    pSlot->tData                                          = stStatus;
    m_pubStatus.Publish(std::move(pSlot));
}

/******************************************************************************
 * @brief Advance any in-flight asynchronous spatial-map extraction. The
 *      ExtractSpatialMapAsync() command kicks off an SDK request and stores a
 *      promise; this method, called each iteration on the owning thread, polls the
 *      SDK for completion and fulfills the promise. Keeping the poll on the owning
 *      thread fixes the old unlocked std::async race.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void ZEDCam::PollPendingSpatialMap()
{
    // Nothing to do if no extraction is pending.
    if (m_pPendingSpatialMapPromise == nullptr)
    {
        // No pending map.
        return;
    }

    // Check the async request status. FAILURE means "still processing" in the ZED SDK.
    sl::ERROR_CODE slStatus = m_slCamera.getSpatialMapRequestStatusAsync();
    if (slStatus == sl::ERROR_CODE::SUCCESS)
    {
        // The map is ready: retrieve it and fulfill the promise.
        sl::Mesh slSpatialMap;
        m_slCamera.retrieveSpatialMapAsync(slSpatialMap);
        m_pPendingSpatialMapPromise->set_value(slSpatialMap);
        m_pPendingSpatialMapPromise.reset();
    }
    else if (slStatus != sl::ERROR_CODE::FAILURE)
    {
        // A real error (not "still processing"): give up and fulfill with an empty mesh.
        LOG_ERROR(logging::g_qSharedLogger, "Failed to extract ZED spatial map. sl::ERROR_CODE is: {}", sl::toString(slStatus).get());
        m_pPendingSpatialMapPromise->set_value(sl::Mesh());
        m_pPendingSpatialMapPromise.reset();
    }
    // else: still processing; leave pending for the next iteration.
}

/******************************************************************************
 * @brief Not used. Frame distribution is handled by the publish-latest mechanism
 *      in ThreadedContinuousCode(); no per-consumer fan-out work remains.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-08
 ******************************************************************************/
void ZEDCam::PooledLinearCode() {}

/******************************************************************************
 * @brief Resets the cameras X,Y,Z translation and Roll,Pitch,Yaw orientation back
 *      to 0. THINK CAREFULLY! Do you actually want to reset this? It will also realign
 *      the coordinate system to whichever way the camera happens to be facing.
 *
 * @return sl::ERROR_CODE - Status of the positional tracking reset.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::ResetPositionalTracking()
{
    // Post to the owning thread and block for the result so all SDK access stays single threaded.
    return this->RunOnOwningThread<sl::ERROR_CODE>([this]() { return this->ImplResetPositionalTracking(); }, sl::ERROR_CODE::FAILURE, "ResetPositionalTracking");
}

/******************************************************************************
 * @brief Owning-thread implementation of ResetPositionalTracking(). Runs when the
 *      producer drains the command queue, so it needs no lock.
 *
 * @return sl::ERROR_CODE - Status of the positional tracking reset.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::ImplResetPositionalTracking()
{
    // Create new translation to set position back to user given values.
    sl::Translation slZeroTranslation(0, 0, 0);
    // Update offset member variables.
    m_dPoseOffsetX = 0.0;
    m_dPoseOffsetY = 0.0;
    m_dPoseOffsetZ = 0.0;
    // This will reset position and coordinate frame.
    sl::Rotation slZeroRotation;
    slZeroRotation.setEulerAngles(sl::float3(0.0, 0.0, 0.0), false);

    // Store new translation and rotation in a transform object.
    sl::Transform slZeroTransform(slZeroRotation, slZeroTranslation);

    // Submit logger message.
    LOG_NOTICE(logging::g_qSharedLogger, "Resetting positional tracking for camera {} ({})!", m_szCameraModelCached, m_unCameraSerialNumber);

    // Reset the positional tracking location of the camera.
    return m_slCamera.resetPositionalTracking(slZeroTransform);
}

/******************************************************************************
 * @brief A vector containing CustomBoxObjectData objects. These objects simply store
 *      information about your detected objects from an external object detection model.
 *      You will need to take your inference results and package them into a sl::CustomBoxObjectData
 *      so the the ZEDSDK can properly interpret your detections.
 *
 *      Giving the bounding boxes of your detected objects to the ZEDSDK will enable positional
 *      tracking and velocity estimation for each object. Even when not in view. The IDs of objects
 *      will also become persistent.
 *
 * @param vCustomObjects - A vector of sl::CustomBoxObjectData objects.
 * @return sl::ERROR_CODE - The return status of ingestion.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::TrackCustomBoxObjects(std::vector<ZedObjectData>& vCustomObjects)
{
    // Create instance variables.
    std::vector<sl::CustomBoxObjectData> vCustomBoxData;

    // Repack detection data into sl specific object.
    for (ZedObjectData stObjectData : vCustomObjects)
    {
        // Create new sl CustomBoxObjectData struct.
        sl::CustomBoxObjectData slCustomBox;
        std::vector<sl::uint2> vCorners;

        // Assign simple attributes.
        slCustomBox.unique_object_id = sl::String(stObjectData.GetObjectUUID().c_str());
        slCustomBox.label            = stObjectData.nClassNumber;
        slCustomBox.probability      = stObjectData.fConfidence;
        slCustomBox.is_grounded      = stObjectData.bObjectRemainsOnFloorPlane;
        // Repackage object corner data.
        vCorners.emplace_back(sl::uint2(stObjectData.cvBoundingBox.x, stObjectData.cvBoundingBox.y));                                        // Top-left corner
        vCorners.emplace_back(sl::uint2(stObjectData.cvBoundingBox.x + stObjectData.cvBoundingBox.width, stObjectData.cvBoundingBox.y));     // Top-right corner
        vCorners.emplace_back(sl::uint2(stObjectData.cvBoundingBox.x, stObjectData.cvBoundingBox.y + stObjectData.cvBoundingBox.height));    // Bottom-left corner
        vCorners.emplace_back(sl::uint2(stObjectData.cvBoundingBox.x + stObjectData.cvBoundingBox.width,
                                        stObjectData.cvBoundingBox.y + stObjectData.cvBoundingBox.height));                                  // Bottom-right corner
        slCustomBox.bounding_box_2d = vCorners;

        // Append repackaged object to vector.
        vCustomBoxData.emplace_back(slCustomBox);
    }

    // Give the packaged data to the owning thread to ingest, and block for the result.
    return this->RunOnOwningThread<sl::ERROR_CODE>([this, vCustomBoxData]() { return this->ImplTrackCustomBoxObjects(vCustomBoxData); },
                                                   sl::ERROR_CODE::FAILURE,
                                                   "TrackCustomBoxObjects");
}

/******************************************************************************
 * @brief Owning-thread implementation of TrackCustomBoxObjects(). Runs when the
 *      producer drains the command queue, so it needs no lock.
 *
 * @param vCustomBoxData - The already-packaged sl::CustomBoxObjectData to ingest.
 * @return sl::ERROR_CODE - The return status of ingestion.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::ImplTrackCustomBoxObjects(const std::vector<sl::CustomBoxObjectData>& vCustomBoxData)
{
    // Give the custom box data to the zed api.
    sl::ERROR_CODE slReturnCode = m_slCamera.ingestCustomBoxObjects(vCustomBoxData);

    // Check if ingestion FAILED (the old code had this success check inverted).
    if (slReturnCode != sl::ERROR_CODE::SUCCESS)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger,
                    "Failed to ingest new objects for camera {} ({})! sl::ERROR_CODE is: {}",
                    m_szCameraModelCached,
                    m_unCameraSerialNumber,
                    sl::toString(slReturnCode).get());
    }

    // Return error code.
    return slReturnCode;
}

/******************************************************************************
 * @brief Performs a hardware reset of the ZED2 or ZED2i camera.
 *
 * @return sl::ERROR_CODE - Whether or not the camera reboot was successful.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::RebootCamera()
{
    // Post to the owning thread and block for the result so all SDK access stays single threaded.
    return this->RunOnOwningThread<sl::ERROR_CODE>([this]() { return this->ImplRebootCamera(); }, sl::ERROR_CODE::FAILURE, "RebootCamera");
}

/******************************************************************************
 * @brief Owning-thread implementation of RebootCamera().
 *
 * @return sl::ERROR_CODE - Whether or not the camera reboot was successful.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::ImplRebootCamera()
{
    // Reboot this camera and return the status code.
    return sl::Camera::reboot(m_unCameraSerialNumber);
}

/******************************************************************************
 * @brief Enable the positional tracking functionality of the camera.
 *
 * @param fExpectedCameraHeightFromFloorTolerance - The expected height of the camera from the floor.
 *              This aids with floor plane detection.
 * @return sl::ERROR_CODE - Whether or not positional tracking was successfully enabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-22
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::EnablePositionalTracking(const float fExpectedCameraHeightFromFloorTolerance)
{
    // Post to the owning thread and block for the result so all SDK access stays single threaded.
    return this->RunOnOwningThread<sl::ERROR_CODE>([this, fExpectedCameraHeightFromFloorTolerance]()
                                                   { return this->ImplEnablePositionalTracking(fExpectedCameraHeightFromFloorTolerance); },
                                                   sl::ERROR_CODE::FAILURE,
                                                   "EnablePositionalTracking");
}

/******************************************************************************
 * @brief Owning-thread implementation of EnablePositionalTracking(). Also called
 *      directly by the reconnect path (already on the owning thread).
 *
 * @param fExpectedCameraHeightFromFloorTolerance - Expected camera height from floor.
 * @return sl::ERROR_CODE - Whether or not positional tracking was successfully enabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::ImplEnablePositionalTracking(const float fExpectedCameraHeightFromFloorTolerance)
{
    // Assign member variable.
    m_fExpectedCameraHeightFromFloorTolerance = fExpectedCameraHeightFromFloorTolerance;

    // Enable pose tracking and store return code.
    sl::ERROR_CODE slReturnCode = m_slCamera.enablePositionalTracking(m_slPoseTrackingParams);

    // Check if positional tracking was enabled properly.
    if (slReturnCode != sl::ERROR_CODE::SUCCESS)
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Failed to enable positional tracking for camera {} ({})! sl::ERROR_CODE is: {}",
                  m_szCameraModelCached,
                  m_unCameraSerialNumber,
                  sl::toString(slReturnCode).get());
    }

    // Set flag.
    m_bEnablePositionalTrackingFlag = true;

    // Return error code.
    return slReturnCode;
}

/******************************************************************************
 * @brief Disable to positional tracking functionality of the camera.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
void ZEDCam::DisablePositionalTracking()
{
    // Post to the owning thread and block until done so all SDK access stays single threaded.
    this->RunOnOwningThreadVoid([this]() { this->ImplDisablePositionalTracking(); }, "DisablePositionalTracking");
}

/******************************************************************************
 * @brief Owning-thread implementation of DisablePositionalTracking().
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void ZEDCam::ImplDisablePositionalTracking()
{
    // Disable pose tracking.
    m_slCamera.disablePositionalTracking();
    // Set flag.
    m_bEnablePositionalTrackingFlag = false;
}

/******************************************************************************
 * @brief Sets the pose of the positional tracking of the camera. XYZ will point
 *      in their respective directions according to ZED_COORD_SYSTEM defined in
 *      AutonomyConstants.h.
 *
 *      Warning: This method is slow and should not be called in a loop. Setting the pose
 *              will temporarily block the entire camera from grabbed or copying frames to
 *              new threads. This method should only be called occasionally when absolutely needed.
 *
 * @param dX - The X position of the camera in ZED_MEASURE_UNITS.
 * @param dY - The Y position of the camera in ZED_MEASURE_UNITS.
 * @param dZ - The Z position of the camera in ZED_MEASURE_UNITS.
 * @param dXO - The tilt of the camera around the X axis in degrees. (0-360)
 * @param dYO - The tilt of the camera around the Y axis in degrees. (0-360)
 * @param dZO - The tilt of the camera around the Z axis in degrees. (0-360)
 * @return sl::ERROR_CODE - Whether or not the pose was set successfully.
 *
 * @bug The ZEDSDK currently cannot handle resetting the positional pose with large translational (dX, dY, dZ) values without breaking positional
 *      tracking. This is because the values are floats and not doubles. To fix this I (claytonraycowen@gmail.com), have decided to just handle the translation offsets
 *      internally. So when SetPositionalPose() is called is assigns the dX, dY, and dZ values to private member variables of this class, then the offsets are added the
 *      the pose in the PooledLinearCode() under the pose requests section. If StereoLabs fixes this in the future, I will go back to using the sl::Translation to reset
 *      positional tracking.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
void ZEDCam::SetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO)
{
    // Post to the owning thread and block until done. This reads m_slCameraPose, which the producer
    // thread writes, so it must run on the owning thread to avoid racing it.
    this->RunOnOwningThreadVoid([this, dX, dY, dZ, dXO, dYO, dZO]() { this->ImplSetPositionalPose(dX, dY, dZ, dXO, dYO, dZO); }, "SetPositionalPose");
}

/******************************************************************************
 * @brief Owning-thread implementation of SetPositionalPose().
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void ZEDCam::ImplSetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO)
{
    // Update offset member variables.
    m_dPoseOffsetX = dX - m_slCameraPose.getTranslation().x;
    m_dPoseOffsetY = dY - m_slCameraPose.getTranslation().y;
    m_dPoseOffsetZ = dZ - m_slCameraPose.getTranslation().z;
    // Find the angular distance from current and desired pose angles. This is complicated because zed uses different angle ranges.
    m_dPoseOffsetXO =
        numops::InputAngleModulus(numops::AngularDifference(numops::InputAngleModulus<double>(m_slCameraPose.getEulerAngles(false).x, 0.0, 360.0), dXO), 0.0, 360.0);
    m_dPoseOffsetYO =
        numops::InputAngleModulus(numops::AngularDifference(numops::InputAngleModulus<double>(m_slCameraPose.getEulerAngles(false).y, 0.0, 360.0), dYO), 0.0, 360.0);
    m_dPoseOffsetZO =
        numops::InputAngleModulus(numops::AngularDifference(numops::InputAngleModulus<double>(m_slCameraPose.getEulerAngles(false).z, 0.0, 360.0), dZO), 0.0, 360.0);
}

/******************************************************************************
 * @brief Enabled the spatial mapping feature of the camera. Pose tracking will be
 *      enabled if it is not already.
 *
 * @return sl::ERROR_CODE - Whether or not spatial mapping was successfully enabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::EnableSpatialMapping()
{
    // Post to the owning thread and block for the result so all SDK access stays single threaded.
    return this->RunOnOwningThread<sl::ERROR_CODE>([this]() { return this->ImplEnableSpatialMapping(); }, sl::ERROR_CODE::FAILURE, "EnableSpatialMapping");
}

/******************************************************************************
 * @brief Owning-thread implementation of EnableSpatialMapping(). Also called
 *      directly by the reconnect path (already on the owning thread).
 *
 * @return sl::ERROR_CODE - Whether or not spatial mapping was successfully enabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::ImplEnableSpatialMapping()
{
    // Create instance variables.
    sl::ERROR_CODE slReturnCode = sl::ERROR_CODE::SUCCESS;

    // Check if positional tracking is enabled; enable it if not.
    if (!m_slCamera.isPositionalTrackingEnabled())
    {
        // Enable positional tracking (owning-thread implementation).
        slReturnCode = this->ImplEnablePositionalTracking(m_fExpectedCameraHeightFromFloorTolerance);
    }

    // Check if positional tracking is or was enabled successfully.
    if (slReturnCode == sl::ERROR_CODE::SUCCESS)
    {
        // Call camera grab function once to ensure the camera is initialized with data.
        m_slCamera.grab(m_slRuntimeParams);
        // Enable spatial mapping.
        slReturnCode = m_slCamera.enableSpatialMapping(m_slSpatialMappingParams);

        // Check if spatial mapping was enabled properly.
        if (slReturnCode != sl::ERROR_CODE::SUCCESS)
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger,
                      "Failed to enabled spatial mapping for camera {} ({})! sl::ERROR_CODE is: {}",
                      m_szCameraModelCached,
                      m_unCameraSerialNumber,
                      sl::toString(slReturnCode).get());
        }
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Failed to enabled spatial mapping for camera {} ({}) because positional tracking could not be enabled! sl::ERROR_CODE is: {}",
                  m_szCameraModelCached,
                  m_unCameraSerialNumber,
                  sl::toString(slReturnCode).get());
    }

    // Set flag.
    m_bEnableSpatialMappingFlag = true;

    // Return error code.
    return slReturnCode;
}

/******************************************************************************
 * @brief Disabled the spatial mapping feature of the camera.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
void ZEDCam::DisableSpatialMapping()
{
    // Post to the owning thread and block until done so all SDK access stays single threaded.
    this->RunOnOwningThreadVoid([this]() { this->ImplDisableSpatialMapping(); }, "DisableSpatialMapping");
}

/******************************************************************************
 * @brief Owning-thread implementation of DisableSpatialMapping().
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void ZEDCam::ImplDisableSpatialMapping()
{
    // Disable spatial mapping.
    m_slCamera.disableSpatialMapping();
    // Set flag.
    m_bEnableSpatialMappingFlag = false;
}

/******************************************************************************
 * @brief Enables the object detection and tracking feature of the camera.
 *
 * @return sl::ERROR_CODE - Whether or not object detection/tracking was successfully enabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::EnableObjectDetection(const bool bEnableBatching)
{
    // Post to the owning thread and block for the result so all SDK access stays single threaded.
    return this->RunOnOwningThread<sl::ERROR_CODE>([this, bEnableBatching]() { return this->ImplEnableObjectDetection(bEnableBatching); },
                                                   sl::ERROR_CODE::FAILURE,
                                                   "EnableObjectDetection");
}

/******************************************************************************
 * @brief Owning-thread implementation of EnableObjectDetection(). Also called
 *      directly by the reconnect path (already on the owning thread).
 *
 * @param bEnableBatching - Whether or not to enable batching.
 * @return sl::ERROR_CODE - Whether or not object detection was successfully enabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::ImplEnableObjectDetection(const bool bEnableBatching)
{
    // Check if batching should be turned on.
    bEnableBatching ? m_slObjectDetectionBatchParams.enable = true : m_slObjectDetectionBatchParams.enable = false;
    // Give batch params to detection params.
    m_slObjectDetectionParams.batch_parameters = m_slObjectDetectionBatchParams;

    // Enable object detection.
    sl::ERROR_CODE slReturnCode = m_slCamera.enableObjectDetection(m_slObjectDetectionParams);

    // Check if object detection was enabled properly.
    if (slReturnCode != sl::ERROR_CODE::SUCCESS)
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Failed to enabled object detection for camera {} ({})! sl::ERROR_CODE is: {}",
                  m_szCameraModelCached,
                  m_unCameraSerialNumber,
                  sl::toString(slReturnCode).get());
    }

    // Set flag.
    m_bEnableObjectDetectionFlag = true;

    // Return error code.
    return slReturnCode;
}

/******************************************************************************
 * @brief Disables the object detection and tracking feature of the camera.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
void ZEDCam::DisableObjectDetection()
{
    // Post to the owning thread and block until done so all SDK access stays single threaded.
    this->RunOnOwningThreadVoid([this]() { this->ImplDisableObjectDetection(); }, "DisableObjectDetection");
}

/******************************************************************************
 * @brief Owning-thread implementation of DisableObjectDetection().
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
void ZEDCam::ImplDisableObjectDetection()
{
    // Disable object detection and tracking.
    m_slCamera.disableObjectDetection();
    // Set flag.
    m_bEnableObjectDetectionFlag = false;
}

/******************************************************************************
 * @brief Accessor for the current status of the camera.
 *
 * @return true - Camera is currently opened and functional.
 * @return false - Camera is not opened and/or connected.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
bool ZEDCam::GetCameraIsOpen()
{
    // Lock-free read of the newest published status snapshot.
    pubsub::Publisher<CameraStatus>::SharedSnapshot pStatus = m_pubStatus.Get();
    return this->GetThreadState() == AutonomyThreadState::eRunning && pStatus != nullptr && pStatus->tData.bCameraIsOpen;
}

/******************************************************************************
 * @brief Accessor for if this ZED is storing it's frames in GPU memory.
 *
 * @return true - Using GPU memory for mats.
 * @return false - Using CPU memory for mats.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-09
 ******************************************************************************/
bool ZEDCam::GetUsingGPUMem() const
{
    // Check if we are using GPU memory.
    return m_slMemoryType == sl::MEM::GPU;
}

/******************************************************************************
 * @brief Accessor for the model enum from the ZEDSDK and represents the camera model as a string.
 *
 * @return std::string - The model of the zed camera.
 *      Possible values: ZED, ZED_MINI, ZED_2, ZED_2i, ZED_X, ZED_X_MINI, UNDEFINED_UNKNOWN
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
std::string ZEDCam::GetCameraModel()
{
    // Lock-free read of the newest published status snapshot; the model travels inside it.
    pubsub::Publisher<CameraStatus>::SharedSnapshot pStatus = m_pubStatus.Get();
    return (pStatus != nullptr) ? pStatus->tData.szCameraModel : std::string("NOT_OPENED");
}

/******************************************************************************
 * @brief Accessor for the camera's serial number.
 *
 * @return unsigned int - The serial number of the camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
unsigned int ZEDCam::GetCameraSerial()
{
    // Return the model string to show camera isn't opened.
    return m_unCameraSerialNumber;
}

/******************************************************************************
 * @brief Accessor for if the positional tracking functionality of the camera has been enabled
 *      and functioning.
 *
 * @return true - Positional tracking is enabled.
 * @return false - Positional tracking is not enabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
bool ZEDCam::GetPositionalTrackingEnabled()
{
    // Lock-free read of the newest published status snapshot.
    pubsub::Publisher<CameraStatus>::SharedSnapshot pStatus = m_pubStatus.Get();
    return pStatus != nullptr && pStatus->tData.bPositionalTrackingEnabled;
}

/******************************************************************************
 * @brief Accessor for the current positional tracking status of the camera.
 *
 * @return sl::PositionalTrackingStatus - The sl::PositionalTrackingStatus struct storing
 *      information about the current VIO positional tracking state.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-04-20
 ******************************************************************************/
sl::PositionalTrackingStatus ZEDCam::GetPositionalTrackingState()
{
    // Lock-free read of the newest published status snapshot.
    pubsub::Publisher<CameraStatus>::SharedSnapshot pStatus = m_pubStatus.Get();
    return (pStatus != nullptr) ? pStatus->tData.stPositionalTrackingStatus : sl::PositionalTrackingStatus();
}

/******************************************************************************
 * @brief Accessor for the current state of the camera's spatial mapping feature.
 *
 * @return sl::SPATIAL_MAPPING_STATE - The enum value of the spatial mapping state.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
sl::SPATIAL_MAPPING_STATE ZEDCam::GetSpatialMappingState()
{
    // Lock-free read of the newest published status snapshot.
    pubsub::Publisher<CameraStatus>::SharedSnapshot pStatus = m_pubStatus.Get();
    return (pStatus != nullptr) ? pStatus->tData.eSpatialMappingState : sl::SPATIAL_MAPPING_STATE::NOT_ENABLED;
}

/******************************************************************************
 * @brief Retrieve the built spatial map from the camera. Spatial mapping must be enabled.
 *  This method takes in an std::future<sl::FusedPointCloud> to eventually store the map in.
 *  It returns a enum code representing the successful scheduling of building the map.
 *  Any code other than SPATIAL_MAPPING_STATE::OK means the future will never be filled.
 *
 * @param std::future<sl::Mesh> - The future to eventually store the map in.
 * @return sl::SPATIAL_MAPPING_STATE - Whether or not the building of the map was successfully scheduled.
 *          Anything other than OK means the future will never be filled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
sl::SPATIAL_MAPPING_STATE ZEDCam::ExtractSpatialMapAsync(std::future<sl::Mesh>& fuMeshFuture)
{
    // Create a promise for the mesh and hand its future to the caller now. The producer loop
    // (PollPendingSpatialMap) fulfills it once the SDK finishes, so all SDK access stays on the
    // owning thread instead of the old unlocked std::async lambda.
    std::shared_ptr<std::promise<sl::Mesh>> pMeshPromise = std::make_shared<std::promise<sl::Mesh>>();
    fuMeshFuture                                         = pMeshPromise->get_future();

    // Post the request to the owning thread and block for the resulting mapping state. If the
    // camera thread is not running the command is cancelled, and the catch below fulfills the
    // caller's mesh future so it is never left waiting on a mesh that will never arrive.
    const sl::SPATIAL_MAPPING_STATE slReturnState = this->RunOnOwningThread<sl::SPATIAL_MAPPING_STATE>(
        [this, pMeshPromise]() -> sl::SPATIAL_MAPPING_STATE
        {
            // Get the current state of spatial mapping.
            sl::SPATIAL_MAPPING_STATE slMappingState = m_slCamera.getSpatialMappingState();
            if (slMappingState == sl::SPATIAL_MAPPING_STATE::OK)
            {
                // Request that the ZEDSDK begin processing the spatial map for export.
                m_slCamera.requestSpatialMapAsync();
                // If a previous extraction is still pending, release its caller with an empty mesh.
                if (m_pPendingSpatialMapPromise != nullptr)
                {
                    // Fulfill the superseded request.
                    m_pPendingSpatialMapPromise->set_value(sl::Mesh());
                }
                // Store this promise for the producer loop to complete.
                m_pPendingSpatialMapPromise = pMeshPromise;
            }
            else
            {
                // Mapping is not ready; fulfill the caller immediately with an empty mesh.
                LOG_WARNING(logging::g_qSharedLogger, "ZED spatial mapping was never enabled, can't extract spatial map!");
                pMeshPromise->set_value(sl::Mesh());
            }
            // Return current spatial mapping state.
            return slMappingState;
        },
        sl::SPATIAL_MAPPING_STATE::NOT_ENABLED,
        "ExtractSpatialMapAsync");

    // If the command was cancelled (camera thread not running) nothing fulfilled the mesh
    // promise, and the caller would block forever on the future we already handed them. Fulfill
    // it here. If it was already fulfilled above, set_value throws and we simply ignore it.
    if (slReturnState != sl::SPATIAL_MAPPING_STATE::OK)
    {
        try
        {
            // Release the caller with an empty mesh.
            pMeshPromise->set_value(sl::Mesh());
        }
        catch (const std::future_error&)
        {
            // Already fulfilled by the command itself; nothing to do.
        }
    }

    // Return current spatial mapping state.
    return slReturnState;
}

/******************************************************************************
 * @brief Accessor for if the cameras object detection and tracking feature is enabled.
 *
 * @return true - Object detection and tracking is enabled.
 * @return false - Object detection and tracking is not enabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
bool ZEDCam::GetObjectDetectionEnabled()
{
    // Lock-free read of the newest published status snapshot.
    pubsub::Publisher<CameraStatus>::SharedSnapshot pStatus = m_pubStatus.Get();
    return pStatus != nullptr && pStatus->tData.bObjectDetectionEnabled;
}
