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
#include "../../util/vision/ImageOperations.hpp"

/******************************************************************************
 * @brief This struct is part of the ZEDCam class and is used as a container for all
 *      bounding box data that is going to be passed to the zed api via the ZEDCam's
 *      TrackCustomBoxObjects() method.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-29
 ******************************************************************************/
struct ZEDCam::ZedObjectData

{
    private:
        // Declare and define private struct member variables.
        std::string szObjectUUID = sl::generate_unique_id().get();    // This will automatically generate a guaranteed unique id so the object is traceable.

        // Declare a private struct for holding point data.
        /******************************************************************************
         * @brief This struct is internal to the ZedObjectData struct is used to store
         *      an X and Y value for the corners of a bounding box.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-08-29
         ******************************************************************************/
        struct Corner
        {
            public:
                // Declare public struct member variables.
                unsigned int nX;
                unsigned int nY;
        };

    public:
        // Declare and define public struct member variables.
        Corner CornerTL;      // The top left corner of the bounding box.
        Corner CornerTR;      // The top right corner of the bounding box.
        Corner CornerBL;      // The bottom left corner of the bounding box.
        Corner CornerBR;      // The bottom right corner of bounding box.
        int nClassNumber;     // This info is passed through from your detection algorithm and will improve tracking be ensure the type of object remains the
        float fConfidence;    // This info is passed through from your detection algorithm and will help improve tracking by throwing out bad detections.
        // Whether of not this object remains on the floor plane. This parameter can't be changed for a given object tracking ID, it's advised to set it by class
        // to avoid issues.
        bool bObjectRemainsOnFloorPlane = false;

        // Declare and define public struct getters.
        std::string GetObjectUUID() { return szObjectUUID; };
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * @brief Construct a new Zed Cam:: Zed Cam object.
 *
 * @param nPropResolutionX - X res of camera. Must be smaller than ZED_BASE_RESOLUTION.
 * @param nPropResolutionY - Y res of camera. Must be smaller than ZED_BASE_RESOLUTION.
 * @param nPropFramesPerSecond - FPS camera is running at.
 * @param dPropHorizontalFOV - The horizontal field of view.
 * @param dPropVerticalFOV - The vertical field of view.
 * @param fMinSenseDistance - The minimum distance to include in depth measures.
 * @param fMaxSenseDistance - The maximum distance to include in depth measures.
 * @param bMemTypeGPU - Whether or not to use the GPU memory for operations.
 * @param unCameraSerialNumber - The serial number of the camera to open.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
ZEDCam::ZEDCam(const int nPropResolutionX,
               const int nPropResolutionY,
               const int nPropFramesPerSecond,
               const double dPropHorizontalFOV,
               const double dPropVerticalFOV,
               const float fMinSenseDistance,
               const float fMaxSenseDistance,
               const bool bMemTypeGPU,
               const bool bUseHalfDepthPrecision,
               const int nNumFrameRetrievalThreads,
               const unsigned int unCameraSerialNumber) :
    Camera(nPropResolutionX, nPropResolutionY, nPropFramesPerSecond, PIXEL_FORMATS::eZED, dPropHorizontalFOV, dPropVerticalFOV)
{
    // Assign member variables.
    bMemTypeGPU ? m_slMemoryType = sl::MEM::GPU : m_slMemoryType = sl::MEM::CPU;
    bUseHalfDepthPrecision ? m_slDepthMeasureType = sl::MEASURE::DEPTH_U16_MM : m_slDepthMeasureType = sl::MEASURE::DEPTH;
    m_nNumFrameRetrievalThreads = nNumFrameRetrievalThreads;
    m_unCameraSerialNumber      = unCameraSerialNumber;

    // Setup camera params.
    m_slCameraParams.camera_resolution      = constants::ZED_BASE_RESOLUTION;
    m_slCameraParams.camera_fps             = nPropFramesPerSecond;
    m_slCameraParams.coordinate_units       = constants::ZED_MEASURE_UNITS;
    m_slCameraParams.coordinate_system      = constants::ZED_COORD_SYSTEM;
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
    if (constants::ZED_MAPPING_RANGE_METER >= 0)
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
    m_slObjectDetectionParams.image_sync           = constants::ZED_OBJDETECTION_IMG_SYNC;
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
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger,
                  "{} stereo camera with serial number {} has been successfully opened.",
                  this->GetCameraModel(),
                  m_slCamera.getCameraInformation().serial_number);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Unable to open stereo camera {} ({})! sl::ERROR_CODE is: {}",
                  sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                  m_slCamera.getCameraInformation().serial_number,
                  sl::toString(slReturnCode).get());
    }
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

    // Close the ZEDCam.
    m_slCamera.close();

    // Submit logger message.
    LOG_DEBUG(logging::g_qSharedLogger, "ZED stereo camera with serial number {} has been successfully closed.", m_unCameraSerialNumber);
}

/******************************************************************************
 * @brief The code inside this private method runs in a separate thread, but still
 *      has access to this*. This method continuously calls the grab() function of
 *      the ZEDSDK, which updates all frames (RGB, depth, cloud) and all other data
 *      such as positional and spatial mapping. Then a thread pool is started and joined
 *      once per iteration to mass copy the frames and/or measure to any other thread
 *      waiting in the queues.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-01
 ******************************************************************************/
void ZEDCam::ThreadedContinuousCode()
{
    // Check if camera is opened.
    if (!m_slCamera.isOpened())
    {
        // Shutdown threads for this ZEDCam.
        this->RequestStop();
        // Submit logger message.
        LOG_CRITICAL(logging::g_qSharedLogger,
                     "Camera start was attempted for camera at {}, but camera never properly opened or it has been closed/rebooted!",
                     m_unCameraSerialNumber);
    }
    else
    {
        // Acquire write lock for camera object.
        std::unique_lock<std::shared_mutex> lkSharedCameraLock(m_muCameraMutex);
        // Call generalized update method of zed api.
        sl::ERROR_CODE slReturnCode = m_slCamera.grab(m_slRuntimeParams);
        // Check if new frame was computed successfully.
        if (slReturnCode == sl::ERROR_CODE::SUCCESS)
        {
            // Grab regular image and store it in member variable.
            slReturnCode = m_slCamera.retrieveImage(m_slFrame, constants::ZED_RETRIEVE_VIEW, m_slMemoryType, sl::Resolution(m_nPropResolutionX, m_nPropResolutionY));
            // Check that the regular frame was retrieved successfully.
            if (slReturnCode != sl::ERROR_CODE::SUCCESS)
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Unable to retrieve new frame image for stereo camera {} ({})! sl::ERROR_CODE is: {}",
                            sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                            m_slCamera.getCameraInformation().serial_number,
                            sl::toString(slReturnCode).get());
            }

            // Grab depth measure and store it in member variable.
            slReturnCode = m_slCamera.retrieveMeasure(m_slDepthMeasure, m_slDepthMeasureType, m_slMemoryType, sl::Resolution(m_nPropResolutionX, m_nPropResolutionY));
            // Check that the regular frame was retrieved successfully.
            if (slReturnCode != sl::ERROR_CODE::SUCCESS)
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Unable to retrieve new depth measure for stereo camera {} ({})! sl::ERROR_CODE is: {}",
                            sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                            m_slCamera.getCameraInformation().serial_number,
                            sl::toString(slReturnCode).get());
            }

            // Grab depth grayscale image and store it in member variable.
            slReturnCode = m_slCamera.retrieveImage(m_slDepthImage, sl::VIEW::DEPTH, m_slMemoryType, sl::Resolution(m_nPropResolutionX, m_nPropResolutionY));
            // Check that the regular frame was retrieved successfully.
            if (slReturnCode != sl::ERROR_CODE::SUCCESS)
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Unable to retrieve new depth image for stereo camera {} ({})! sl::ERROR_CODE is: {}",
                            sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                            m_slCamera.getCameraInformation().serial_number,
                            sl::toString(slReturnCode).get());
            }

            // Grab regular resized image and store it in member variable.
            slReturnCode = m_slCamera.retrieveMeasure(m_slPointCloud, sl::MEASURE::XYZBGRA, m_slMemoryType, sl::Resolution(m_nPropResolutionX, m_nPropResolutionY));
            // Check that the regular frame was retrieved successfully.
            if (slReturnCode != sl::ERROR_CODE::SUCCESS)
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Unable to retrieve new point cloud for stereo camera {} ({})! sl::ERROR_CODE is: {}",
                            sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                            m_slCamera.getCameraInformation().serial_number,
                            sl::toString(slReturnCode).get());
            }

            // Check if positional tracking is enabled.
            if (m_slCamera.isPositionalTrackingEnabled())
            {
                // Get the current pose of the camera.
                sl::POSITIONAL_TRACKING_STATE slPoseTrackReturnCode = m_slCamera.getPosition(m_slCameraPose, sl::REFERENCE_FRAME::WORLD);
                // Check that the regular frame was retrieved successfully.
                if (slPoseTrackReturnCode != sl::POSITIONAL_TRACKING_STATE::OK)
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger,
                                "Unable to retrieve new positional tracking pose for stereo camera {} ({})! sl::POSITIONAL_TRACKING_STATE is: {}",
                                sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                                m_slCamera.getCameraInformation().serial_number,
                                sl::toString(slPoseTrackReturnCode).get());
                }
            }
            // Check if object detection is enabled.
            if (m_slCamera.isObjectDetectionEnabled())
            {
                // Get updated objects from camera.
                slReturnCode = m_slCamera.retrieveObjects(m_slDetectedObjects);
                // Check that the regular frame was retrieved successfully.
                if (slReturnCode != sl::ERROR_CODE::SUCCESS)
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger,
                                "Unable to retrieve new object data for stereo camera {} ({})! sl::ERROR_CODE is: {}",
                                sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                                m_slCamera.getCameraInformation().serial_number,
                                sl::toString(slReturnCode).get());
                }

                // Check if batched object data is enabled.
                if (m_slObjectDetectionBatchParams.enable)
                {
                    // Get updated batched objects from camera.
                    slReturnCode = m_slCamera.getObjectsBatch(m_slDetectedObjectsBatched);
                    // Check that the regular frame was retrieved successfully.
                    if (slReturnCode != sl::ERROR_CODE::SUCCESS)
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger,
                                    "Unable to retrieve new batched object data for stereo camera {} ({})! sl::ERROR_CODE is: {}",
                                    sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                                    m_slCamera.getCameraInformation().serial_number,
                                    sl::toString(slReturnCode).get());
                    }
                }
            }
            // Release camera lock.
            lkSharedCameraLock.unlock();

            // Call FPS tick.
            m_IPS.Tick();
        }
        else
        {
            // Release camera lock.
            lkSharedCameraLock.unlock();

            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger,
                      "Unable to update stereo camera {} ({}) frames, measurements, and sensors! sl::ERROR_CODE is: {}",
                      sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                      m_slCamera.getCameraInformation().serial_number,
                      sl::toString(slReturnCode).get());
        }

        // Acquire a shared_lock on the frame copy queue.
        std::shared_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
        // Check if the frame copy queue is empty.
        if (!m_qFrameCopySchedule.empty() || !m_qGPUFrameCopySchedule.empty() || !m_qCustomBoxIngestSchedule.empty() || !m_qPoseCopySchedule.empty() ||
            !m_qIMUDataCopySchedule.empty() || !m_qObjectDataCopySchedule.empty() || !m_qObjectBatchedDataCopySchedule.empty())
        {
            // Find the queue with the longest length.
            size_t siMaxQueueLength = std::max({m_qFrameCopySchedule.size(),
                                                m_qGPUFrameCopySchedule.size(),
                                                m_qCustomBoxIngestSchedule.size(),
                                                m_qPoseCopySchedule.size(),
                                                m_qIMUDataCopySchedule.size(),
                                                m_qObjectDataCopySchedule.size(),
                                                m_qObjectBatchedDataCopySchedule.size()});

            // Start the thread pool to store multiple copies of the sl::Mat into the given cv::Mats.
            this->RunDetachedPool(siMaxQueueLength, m_nNumFrameRetrievalThreads);
            // Wait for thread pool to finish.
            this->JoinPool();
            // Release lock on frame copy queue.
            lkSchedulers.unlock();
        }
    }
}

/******************************************************************************
 * @brief This method holds the code that is ran in the thread pool started by
 *      the ThreadedLinearCode() method. It copies the data from the different
 *      data objects to references of the same type stored in a queue filled by the
 *      Request methods.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-08
 ******************************************************************************/
void ZEDCam::PooledLinearCode()
{
    /////////////////////////////
    //  Frame queue.
    /////////////////////////////
    // Check if we are using CPU or GPU mats.
    if (m_slMemoryType == sl::MEM::CPU)
    {
        // Acquire mutex for getting frames out of the queue.
        std::unique_lock<std::mutex> lkFrameQueue(m_muFrameCopyMutex);
        // Check if the queue is empty.
        if (!m_qFrameCopySchedule.empty())
        {
            // Get frame container out of queue.
            containers::FrameFetchContainer<cv::Mat> stContainer = m_qFrameCopySchedule.front();
            // Pop out of queue.
            m_qFrameCopySchedule.pop();
            // Release lock.
            lkFrameQueue.unlock();

            // Determine which frame should be copied.
            switch (stContainer.eFrameType)
            {
                case eBGRA: *(stContainer.pFrame) = imgops::ConvertSLMatToCVMat(m_slFrame); break;
                case eDepthMeasure: *(stContainer.pFrame) = imgops::ConvertSLMatToCVMat(m_slDepthMeasure); break;
                case eDepthImage: *(stContainer.pFrame) = imgops::ConvertSLMatToCVMat(m_slDepthImage); break;
                case eXYZBGRA: *(stContainer.pFrame) = imgops::ConvertSLMatToCVMat(m_slPointCloud); break;
                default: *(stContainer.pFrame) = imgops::ConvertSLMatToCVMat(m_slFrame); break;
            }

            // Signal future that the frame has been successfully retrieved.
            stContainer.pCopiedFrameStatus->set_value(true);
        }

        // Check if anything has been added to the GPU queue.
        if (!m_qGPUFrameCopySchedule.empty())
        {
            // Submit logger error.
            LOG_ERROR(logging::g_qSharedLogger,
                      "ZEDCam ({}) is in CPU sl::Mat mode but a GPU mat has been added to the copy queue! Whichever thread queued the frame will now appear frozen if "
                      "future.get() is called. Either switch the camera to GPU Mat mode in AutonomyConstants.h or stop queueing frames of type cv::Mat.",
                      m_unCameraSerialNumber);
        }
    }
    // Use GPU mat.
    else
    {
        // Acquire mutex for getting frames out of the queue.
        std::unique_lock<std::mutex> lkFrameQueue(m_muFrameCopyMutex);
        // Check if the queue is empty.
        if (!m_qGPUFrameCopySchedule.empty())
        {
            // Get frame container out of queue.
            containers::FrameFetchContainer<cv::cuda::GpuMat> stContainer = m_qGPUFrameCopySchedule.front();
            // Pop out of queue.
            m_qGPUFrameCopySchedule.pop();
            // Release lock.
            lkFrameQueue.unlock();

            // Determine which frame should be copied.
            switch (stContainer.eFrameType)
            {
                case eBGRA: *(stContainer.pFrame) = imgops::ConvertSLMatToGPUMat(m_slFrame); break;
                case eDepthMeasure: *(stContainer.pFrame) = imgops::ConvertSLMatToGPUMat(m_slDepthMeasure); break;
                case eDepthImage: *(stContainer.pFrame) = imgops::ConvertSLMatToGPUMat(m_slDepthImage); break;
                case eXYZBGRA: *(stContainer.pFrame) = imgops::ConvertSLMatToGPUMat(m_slPointCloud); break;
                default: *(stContainer.pFrame) = imgops::ConvertSLMatToGPUMat(m_slFrame); break;
            }

            // Signal future that the frame has been successfully retrieved.
            stContainer.pCopiedFrameStatus->set_value(true);
        }

        // Check if anything has been added to the GPU queue.
        if (!m_qFrameCopySchedule.empty())
        {
            // Submit logger error.
            LOG_ERROR(logging::g_qSharedLogger,
                      "ZEDCam ({}) is in GPU sl::Mat mode but a CPU mat has been added to the copy queue! Whichever thread queued the frame will now appear frozen if "
                      "future.get() is called. Either switch the camera to GPU Mat mode in AutonomyConstants.h or stop queueing frames of type cv::cuda::GpuMat.",
                      m_unCameraSerialNumber);
        }
    }

    /////////////////////////////
    //  Pose queue.
    /////////////////////////////
    // Acquire mutex for getting frames out of the pose queue.
    std::unique_lock<std::mutex> lkPoseQueue(m_muPoseCopyMutex);
    // Check if the queue is empty.
    if (!m_qPoseCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::DataFetchContainer<sl::Pose> stContainer = m_qPoseCopySchedule.front();
        // Pop out of queue.
        m_qPoseCopySchedule.pop();
        // Release lock.
        lkPoseQueue.unlock();

        // Copy pose.
        *(stContainer.pData) = sl::Pose(m_slCameraPose);

        // Signal future that the data has been successfully retrieved.
        stContainer.pCopiedDataStatus->set_value(true);
    }

    /////////////////////////////
    //  ObjectData queue.
    /////////////////////////////
    // Acquire mutex for getting frames out of the pose queue.
    std::unique_lock<std::mutex> lkObjectDataQueue(m_muObjectDataCopyMutex);
    // Check if the queue is empty.
    if (!m_qObjectDataCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::DataFetchContainer<std::vector<sl::ObjectData>> stContainer = m_qObjectDataCopySchedule.front();
        // Pop out of queue.
        m_qObjectDataCopySchedule.pop();
        // Release lock.
        lkObjectDataQueue.unlock();

        // Make copy of object vector. (Apparently the assignment operator actually does a deep copy)
        *(stContainer.pData) = m_slDetectedObjects.object_list;

        // Signal future that the data has been successfully retrieved.
        stContainer.pCopiedDataStatus->set_value(true);
    }

    /////////////////////////////
    //  ObjectData Batched queue.
    /////////////////////////////
    // Acquire mutex for getting frames out of the pose queue.
    std::unique_lock<std::mutex> lkObjectBatchedDataQueue(m_muObjectBatchedDataCopyMutex);
    // Check if the queue is empty.
    if (!m_qObjectBatchedDataCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::DataFetchContainer<std::vector<sl::ObjectsBatch>> stContainer = m_qObjectBatchedDataCopySchedule.front();
        // Pop out of queue.
        m_qObjectBatchedDataCopySchedule.pop();
        // Release lock.
        lkObjectBatchedDataQueue.unlock();

        // Make copy of object vector. (Apparently the assignment operator actually does a deep copy)
        *(stContainer.pData) = m_slDetectedObjectsBatched;

        // Signal future that the data has been successfully retrieved.
        stContainer.pCopiedDataStatus->set_value(true);
    }
}

/******************************************************************************
 * @brief Requests a regular BGRA image from the LEFT eye of the zed camera.
 *      Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
 *      Remember this code will be ran in whatever class/thread calls it.
 *
 * @param cvFrame - A reference to the cv::Mat to copy the normal frame to.
 * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-09
 ******************************************************************************/
std::future<bool> ZEDCam::RequestFrameCopy(cv::Mat& cvFrame)
{
    // Assemble the FrameFetchContainer.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvFrame, eBGRA);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkSchedulers.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Grabs a regular BGRA image from the LEFT eye of the zed camera and stores it in a GPU mat.
 *      Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
 *      Remember this code will be ran in whatever class/thread calls it.
 *
 * @param cvGPUFrame - A reference to the cv::Mat to copy the normal frame to.
 * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-09
 ******************************************************************************/
std::future<bool> ZEDCam::RequestFrameCopy(cv::cuda::GpuMat& cvGPUFrame)
{
    // Assemble the FrameFetchContainer.
    containers::FrameFetchContainer<cv::cuda::GpuMat> stContainer(cvGPUFrame, eBGRA);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qGPUFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkSchedulers.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Requests a depth measure or image from the camera.
 *      Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
 *      This image has the same shape as a grayscale image, but the values represent the depth in
 *      MILLIMETERS. The ZEDSDK will always return this measure in MILLIMETERS.
 *
 * @param cvDepth - A reference to the cv::Mat to copy the depth frame to.
 * @param bRetrieveMeasure - False to get depth IMAGE instead of MEASURE. Do not use the 8-bit grayscale depth image
 *                  purposes other than displaying depth.
 * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
std::future<bool> ZEDCam::RequestDepthCopy(cv::Mat& cvDepth, const bool bRetrieveMeasure)
{
    // Create instance variables.
    PIXEL_FORMATS eFrameType;

    // Check if the container should be set to retrieve an image or a measure.
    bRetrieveMeasure ? eFrameType = eDepthMeasure : eFrameType = eDepthImage;
    // Assemble container.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvDepth, eFrameType);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkSchedulers.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Requests a depth measure or image from the camera.
 *      Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
 *      This image has the same shape as a grayscale image, but the values represent the depth in
 *      MILLIMETERS. The ZEDSDK will always return this measure in MILLIMETERS.
 *
 * @param cvGPUDepth - A reference to the cv::Mat to copy the depth frame to.
 * @param bRetrieveMeasure - False to get depth IMAGE instead of MEASURE. Do not use the 8-bit grayscale depth image
 *                  purposes other than displaying depth.
 * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
std::future<bool> ZEDCam::RequestDepthCopy(cv::cuda::GpuMat& cvGPUDepth, const bool bRetrieveMeasure)
{
    // Create instance variables.
    PIXEL_FORMATS eFrameType;

    // Check if the container should be set to retrieve an image or a measure.
    bRetrieveMeasure ? eFrameType = eDepthMeasure : eFrameType = eDepthImage;
    // Assemble container.
    containers::FrameFetchContainer<cv::cuda::GpuMat> stContainer(cvGPUDepth, eFrameType);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qGPUFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkSchedulers.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Requests a point cloud image from the camera. This image has the same resolution as a normal
 *      image but with three XYZ values replacing the old color values in the 3rd dimension.
 *      The units and sign of the XYZ values are determined by ZED_MEASURE_UNITS and ZED_COORD_SYSTEM
 *      constants set in AutonomyConstants.h.
 *
 *      A 4th value in the 3rd dimension exists as a float32 storing the BGRA values. Each color value
 *      is 8-bits and is in this order:
 *                          00000000 00000000 00000000 00000000 = 32 bits (float32)
 *                              B       G         R       A
 *
 *      Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
 *
 * @param cvPointCloud - A reference to the cv::Mat to copy the point cloud frame to.
 * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
std::future<bool> ZEDCam::RequestPointCloudCopy(cv::Mat& cvPointCloud)
{
    // Assemble the FrameFetchContainer.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvPointCloud, eXYZBGRA);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkSchedulers.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Grabs a point cloud image from the camera. This image has the same resolution as a normal
 *      image but with three XYZ values replacing the old color values in the 3rd dimension.
 *      The units and sign of the XYZ values are determined by ZED_MEASURE_UNITS and ZED_COORD_SYSTEM
 *      constants set in AutonomyConstants.h.
 *
 *      A 4th value in the 3rd dimension exists as a float32 storing the BGRA values. Each color value
 *      is 8-bits and is in this order:
 *                          00000000 00000000 00000000 00000000 = 32 bits (float32)
 *                              B       G         R       A
 *
 *      Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
 *
 * @param cvGPUPointCloud - A reference to the cv::Mat to copy the point cloud frame to.
 * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
std::future<bool> ZEDCam::RequestPointCloudCopy(cv::cuda::GpuMat& cvGPUPointCloud)
{
    // Assemble the FrameFetchContainer.
    containers::FrameFetchContainer<cv::cuda::GpuMat> stContainer(cvGPUPointCloud, eXYZBGRA);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qGPUFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkSchedulers.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

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
    // Create new translation to set position back to zero.
    sl::Translation slZeroTranslation(0.0, 0.0, 0.0);
    // This will reset position and coordinate frame.
    sl::Rotation slZeroRotation;
    slZeroRotation.setEulerAngles(sl::float3(0.0, 0.0, 0.0), false);

    // Store new translation and rotation in a transform object.
    sl::Transform slZeroTransform(slZeroRotation, slZeroTranslation);

    // Acquire write lock.
    std::unique_lock<std::shared_mutex> lkSharedLock(m_muCameraMutex);
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
        vCorners.emplace_back(sl::uint2(stObjectData.CornerTL.nX, stObjectData.CornerTL.nY));
        vCorners.emplace_back(sl::uint2(stObjectData.CornerTR.nX, stObjectData.CornerTR.nY));
        vCorners.emplace_back(sl::uint2(stObjectData.CornerBL.nX, stObjectData.CornerBL.nY));
        vCorners.emplace_back(sl::uint2(stObjectData.CornerBR.nX, stObjectData.CornerBR.nY));
        slCustomBox.bounding_box_2d = vCorners;

        // Append repackaged object to vector.
        vCustomBoxData.emplace_back(slCustomBox);
    }

    // Acquire write lock.
    std::unique_lock<std::shared_mutex> lkSharedLock(m_muCameraMutex);
    // Give the custom box data to the zed api.
    sl::ERROR_CODE slReturnCode = m_slCamera.ingestCustomBoxObjects(vCustomBoxData);
    // Release lock.
    lkSharedLock.unlock();

    // Check if successful.
    if (slReturnCode == sl::ERROR_CODE::SUCCESS)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger,
                    "Failed to ingest new objects for camera {} ({})! sl::ERROR_CODE is: {}",
                    sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                    m_slCamera.getCameraInformation().serial_number,
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
    // Acquire write lock.
    std::unique_lock<std::shared_mutex> lkSharedLock(m_muCameraMutex);
    // Reboot this camera and return the status code.
    return sl::Camera::reboot(m_slCamera.getCameraInformation().serial_number);
}

/******************************************************************************
 * @brief Enable the positional tracking functionality of the camera.
 *
 * @return sl::ERROR_CODE - Whether or not positional tracking was successfully enabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::EnablePositionalTracking()
{
    // Acquire write lock.
    std::unique_lock<std::shared_mutex> lkSharedLock(m_muCameraMutex);
    // Enable pose tracking and store return code.
    sl::ERROR_CODE slReturnCode = m_slCamera.enablePositionalTracking(m_slPoseTrackingParams);
    // Release lock.
    lkSharedLock.unlock();

    // Check if positional tracking was enabled properly.
    if (slReturnCode != sl::ERROR_CODE::SUCCESS)
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Failed to enabled positional tracking for camera {} ({})! sl::ERROR_CODE is: {}",
                  sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                  m_slCamera.getCameraInformation().serial_number,
                  sl::toString(slReturnCode).get());
    }

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
    // Acquire write lock.
    std::unique_lock<std::shared_mutex> lkSharedLock(m_muCameraMutex);
    // Disable pose tracking.
    m_slCamera.disablePositionalTracking();
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
 * @param dXO - The tilt of the camera around the X axis in degrees.
 * @param dYO - The tilt of the camera around the Y axis in degrees.
 * @param dZO - The tilt of the camera around the Z axis in degrees.
 * @return sl::ERROR_CODE - Whether or not the pose was set successfully.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::SetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO)
{
    // Create new translation to set position back to user given values.
    sl::Translation slZeroTranslation(dX, dY, dZ);
    // This will reset position and coordinate frame.
    sl::Rotation slZeroRotation;
    slZeroRotation.setEulerAngles(sl::float3(dXO, dYO, dZO), false);

    // Store new translation and rotation in a transform object.
    sl::Transform slZeroTransform(slZeroRotation, slZeroTranslation);

    // Acquire write lock.
    std::unique_lock<std::shared_mutex> lkSharedLock(m_muCameraMutex);
    // Reset the positional tracking location of the camera.
    return m_slCamera.resetPositionalTracking(slZeroTransform);
}

/******************************************************************************
 * @brief Enabled the spatial mapping feature of the camera. Pose tracking will be
 *      enabled if it is not already.
 *
 * @param fTimeoutSeconds - The timeout used to wait for pose tracking to be on the OK state. Default is 10 seconds.
 * @return sl::ERROR_CODE - Whether or not spatial mapping was successfully enabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::EnableSpatialMapping(const int nTimeoutSeconds)
{
    // Create instance variables.
    auto tmStartTime = std::chrono::steady_clock::now();
    sl::Pose slCameraPose;
    sl::ERROR_CODE slReturnCode;

    // Check if positional tracking is enabled.
    if (!m_slCamera.isPositionalTrackingEnabled())
    {
        // Enable positional tracking.
        this->EnablePositionalTracking();
    }

    // Wait for positional tracking state to be OK. Default Timeout of 10 seconds.
    while (m_slCamera.getPosition(slCameraPose) != sl::POSITIONAL_TRACKING_STATE::OK &&
           std::chrono::steady_clock::now() - tmStartTime <= std::chrono::seconds(nTimeoutSeconds))
    {
        // Sleep for one millisecond.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Final check if positional tracking was successfully enabled.
    if (m_slCamera.getPosition(slCameraPose) == sl::POSITIONAL_TRACKING_STATE::OK)
    {
        // Acquire write lock.
        std::unique_lock<std::shared_mutex> lkSharedLock(m_muCameraMutex);
        // Enable spatial mapping.
        slReturnCode = m_slCamera.enableSpatialMapping(m_slSpatialMappingParams);
        // Release lock.
        lkSharedLock.unlock();

        // Check if positional tracking was enabled properly.
        if (slReturnCode != sl::ERROR_CODE::SUCCESS)
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger,
                      "Failed to enabled spatial mapping for camera {} ({})! sl::ERROR_CODE is: {}",
                      sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                      m_slCamera.getCameraInformation().serial_number,
                      sl::toString(slReturnCode).get());
        }
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Failed to enabled spatial mapping for camera {} ({}) because positional tracking could not be enabled! sl::ERROR_CODE is: {}",
                  sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                  m_slCamera.getCameraInformation().serial_number,
                  sl::toString(slReturnCode).get());
    }

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
    // Acquire write lock.
    std::unique_lock<std::shared_mutex> lkSharedLock(m_muCameraMutex);
    // Disable spatial mapping.
    m_slCamera.disableSpatialMapping();
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
    // Check if batching should be turned on.
    bEnableBatching ? m_slObjectDetectionBatchParams.enable = true : m_slObjectDetectionBatchParams.enable = false;
    // Give batch params to detection params.
    m_slObjectDetectionParams.batch_parameters = m_slObjectDetectionBatchParams;

    // Acquire write lock.
    std::unique_lock<std::shared_mutex> lkSharedLock(m_muCameraMutex);
    // Enable object detection.
    sl::ERROR_CODE slReturnCode = m_slCamera.enableObjectDetection(m_slObjectDetectionParams);
    // Release lock.
    lkSharedLock.unlock();

    // Check if positional tracking was enabled properly.
    if (slReturnCode != sl::ERROR_CODE::SUCCESS)
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger,
                  "Failed to enabled object detection for camera {} ({})! sl::ERROR_CODE is: {}",
                  sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                  m_slCamera.getCameraInformation().serial_number,
                  sl::toString(slReturnCode).get());
    }

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
    // Acquire write lock.
    std::unique_lock<std::shared_mutex> lkSharedLock(m_muCameraMutex);
    // Disable object detection and tracking.
    m_slCamera.disableObjectDetection();
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
    return m_slCamera.isOpened();
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
    if (m_slMemoryType == sl::MEM::GPU)
    {
        // Using GPU memory.
        return true;
    }
    else
    {
        // Not using GPU memory.
        return false;
    }
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
    // Declare instance variables.
    std::string szCameraModel;

    // Check if the camera is opened.
    if (m_slCamera.isOpened())
    {
        // Convert camera model to a string.
        szCameraModel = sl::toString(m_slCamera.getCameraInformation().camera_model).get();
    }
    else
    {
        // Set the model string to show camera isn't opened.
        szCameraModel = "NOT_OPENED";
    }

    // Return model of camera represented as string.
    return szCameraModel;
}

/******************************************************************************
 * @brief Accessor for the camera's serial number.
 *
 * @return unsigned int -
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
unsigned int ZEDCam::GetCameraSerial()
{
    // Return connected camera serial number.
    return m_slCamera.getCameraInformation().serial_number;
}

/******************************************************************************
 * @brief Requests the current pose of the camera relative to it's start pose or the origin of the set pose.
 *      Puts a Pose pointer into a queue so a copy of a frame from the camera can be written to it.
 *      If positional tracking is not enabled, this method will return false and the sl::Pose may be uninitialized.
 *
 * @param slPose - A reference to the sl::Pose object to copy the current camera pose to.
 * @return std::future<bool> - A future that should be waited on before the passed in sl::Pose is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
std::future<bool> ZEDCam::RequestPositionalPoseCopy(sl::Pose& slPose)
{
    // Check if positional tracking has been enabled.
    if (m_slCamera.isPositionalTrackingEnabled())
    {
        // Assemble the data container.
        containers::DataFetchContainer<sl::Pose> stContainer(slPose);

        // Acquire lock on frame copy queue.
        std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
        // Append frame fetch container to the schedule queue.
        m_qPoseCopySchedule.push(stContainer);
        // Release lock on the frame schedule queue.
        lkSchedulers.unlock();

        // Return the future from the promise stored in the container.
        return stContainer.pCopiedDataStatus->get_future();
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Attempted to get ZED positional pose but positional tracking is not enabled!");

        // Create dummy promise to return the future.
        std::promise<bool> pmDummyPromise;
        std::future<bool> fuDummyFuture = pmDummyPromise.get_future();
        // Set future value.
        pmDummyPromise.set_value(false);

        // Return unsuccessful.
        return fuDummyFuture;
    }
}

/******************************************************************************
 * @brief Accessor for if the positional tracking functionality of the camera has been enabled.
 *
 * @return true - Positional tracking is enabled.
 * @return false - Positional tracking is not enabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
bool ZEDCam::GetPositionalTrackingEnabled()
{
    return m_slCamera.isPositionalTrackingEnabled();
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
    // Return the current spatial mapping state of the camera.
    return m_slCamera.getSpatialMappingState();
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
    // Get and store current state of spatial mapping.
    sl::SPATIAL_MAPPING_STATE slReturnState = m_slCamera.getSpatialMappingState();

    // Check if spatial mapping has been enabled and ready
    if (slReturnState == sl::SPATIAL_MAPPING_STATE::OK)
    {
        // Request that the ZEDSDK begin processing the spatial map for export.
        m_slCamera.requestSpatialMapAsync();

        // Start an async thread to wait for spatial map processing to finish. Return resultant future object.
        fuMeshFuture = std::async(std::launch::async,
                                  [this]()
                                  {
                                      // Create instance variables.
                                      sl::Mesh slSpatialMap;

                                      // Loop until map is finished generating.
                                      while (m_slCamera.getSpatialMapRequestStatusAsync() == sl::ERROR_CODE::FAILURE)
                                      {
                                          // Sleep for 10ms.
                                          std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                      }

                                      // Check if the spatial map was exported successfully.
                                      if (m_slCamera.getSpatialMapRequestStatusAsync() == sl::ERROR_CODE::SUCCESS)
                                      {
                                          // Get and store the spatial map.
                                          m_slCamera.retrieveSpatialMapAsync(slSpatialMap);

                                          // Return spatial map.
                                          return slSpatialMap;
                                      }
                                      else
                                      {
                                          // Submit logger message.
                                          LOG_ERROR(logging::g_qSharedLogger,
                                                    "Failed to extract ZED spatial map. sl::ERROR_CODE is: {}",
                                                    sl::toString(m_slCamera.getSpatialMapRequestStatusAsync()).get());

                                          // Return empty point cloud.
                                          return sl::Mesh();
                                      }
                                  });
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "ZED spatial mapping was never enabled, can't extract spatial map!");
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
    return m_slCamera.isObjectDetectionEnabled();
}

/******************************************************************************
 * @brief Requests a current copy of the tracked objects from the camera.
 *      Puts a pointer to a vector containing sl::ObjectData into a queue so a copy of a frame from the camera can be written to it.
 *
 * @param vObjectData - A vector that will have data copied to it containing sl::ObjectData objects.
 * @return std::future<bool> - A future that should be waited on before the passed in vector is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
std::future<bool> ZEDCam::RequestObjectsCopy(std::vector<sl::ObjectData>& vObjectData)
{
    // Check if object detection has been enabled.
    if (m_slCamera.isObjectDetectionEnabled())
    {
        // Assemble the data container.
        containers::DataFetchContainer<std::vector<sl::ObjectData>> stContainer(vObjectData);

        // Acquire lock on frame copy queue.
        std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
        // Append frame fetch container to the schedule queue.
        m_qObjectDataCopySchedule.push(stContainer);
        // Release lock on the frame schedule queue.
        lkSchedulers.unlock();

        // Return the future from the promise stored in the container.
        return stContainer.pCopiedDataStatus->get_future();
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Attempted to get ZED object data but object detection/tracking is not enabled!");

        // Create dummy promise to return the future.
        std::promise<bool> pmDummyPromise;
        std::future<bool> fuDummyFuture = pmDummyPromise.get_future();
        // Set future value.
        pmDummyPromise.set_value(false);

        // Return unsuccessful.
        return fuDummyFuture;
    }
}

/******************************************************************************
 * @brief If batching is enabled, this requests the normal objects and passes them to
 *  the the internal batching queue of the zed api. This performs short-term re-identification
 *  with deep learning and trajectories filtering. Batching must have been set to enabled when
 *  EnableObjectDetection() was called. Most of the time the vector will be empty and will be
 *  filled every ZED_OBJDETECTION_BATCH_LATENCY.
 *
 * @param vBatchedObjectData - A vector containing objects of sl::ObjectsBatch object that will
 *                              have object data copied to.
 * @return std::future<bool> - A future that should be waited on before the passed in vector is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-30
 ******************************************************************************/
std::future<bool> ZEDCam::RequestBatchedObjectsCopy(std::vector<sl::ObjectsBatch>& vBatchedObjectData)
{
    // Check if object detection and batching has been enabled.
    if (m_slCamera.isObjectDetectionEnabled() && m_slObjectDetectionBatchParams.enable)
    {
        // Assemble the data container.
        containers::DataFetchContainer<std::vector<sl::ObjectsBatch>> stContainer(vBatchedObjectData);

        // Acquire lock on frame copy queue.
        std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
        // Append frame fetch container to the schedule queue.
        m_qObjectBatchedDataCopySchedule.push(stContainer);
        // Release lock on the frame schedule queue.
        lkSchedulers.unlock();

        // Return the future from the promise stored in the container.
        return stContainer.pCopiedDataStatus->get_future();
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Attempted to get ZED batched object data but object detection/tracking is not enabled!");

        // Create dummy promise to return the future.
        std::promise<bool> pmDummyPromise;
        std::future<bool> fuDummyFuture = pmDummyPromise.get_future();
        // Set future value.
        pmDummyPromise.set_value(false);

        // Return unsuccessful.
        return fuDummyFuture;
    }
}
