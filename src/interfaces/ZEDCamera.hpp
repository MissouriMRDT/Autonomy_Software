/******************************************************************************
 * @brief Defines and implements the ZEDCamera interface class.
 *
 * @file ZEDCamera.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-22
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef ZEDCAMERA_HPP
#define ZEDCAMERA_HPP

#include "../AutonomyLogging.h"
#include "../util/GeospatialOperations.hpp"
#include "../util/threading/CommandQueue.hpp"
#include "../util/threading/Publisher.hpp"
#include "Camera.hpp"

/// \cond
#include <sl/Camera.hpp>
#include <sl/Fusion.hpp>

/// \endcond

/******************************************************************************
 * @brief This class serves as a middle inheritor between the Camera interface
 *      and the ZEDCam class. ZEDCam and SIMZEDCam will inherit from this class.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-25
 ******************************************************************************/
class ZEDCamera : public Camera<cv::Mat>
{
    public:
        /////////////////////////////////////////
        // Declare public structs that are specific to and used within this class.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief This struct is part of the ZEDCam class and is used as a container for all
         *      bounding box data that is going to be passed to the zed api via the ZEDCam's
         *      TrackCustomBoxObjects() method.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-08-29
         ******************************************************************************/
        struct ZedObjectData
        {
            private:
                // Declare and define private struct member variables.
                std::string szObjectUUID = sl::generate_unique_id().get();    // This will automatically generate a guaranteed unique id so the object is traceable.

            public:
                // Declare and define public struct member variables.
                cv::Rect2d cvBoundingBox;    // The bounding box of the object in the image.
                int nClassNumber;     // This info is passed through from your detection algorithm and will improve tracking be ensure the type of object remains the
                float fConfidence;    // This info is passed through from your detection algorithm and will help improve tracking by throwing out bad detections.
                // Whether of not this object remains on the floor plane. This parameter can't be changed for a given object tracking ID, it's advised to set it by class
                // to avoid issues.
                bool bObjectRemainsOnFloorPlane = false;

                // Declare and define public struct getters.
                std::string GetObjectUUID() { return szObjectUUID; };
        };

        /******************************************************************************
         * @brief This struct is used within the ZEDCam class to store the camera pose with high precision.
         *      The sl::Pose object from the ZEDSDK stores everything as float which is not precise enough for storing
         *      relative UTM values. This struct replaces that.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-04-17
         ******************************************************************************/
        struct Pose
        {
            private:
                // Declare struct for storing translation values.
                struct Translation
                {
                    public:
                        double dX;    // Translation in ZED_MEASURE_UNITS on the x-axis.
                        double dY;    // Translation in ZED_MEASURE_UNITS on the y-axis.
                        double dZ;    // Translation in ZED_MEASURE_UNITS on the z-axis.
                };

                // Declare struct for storing rotation values.
                struct EulerAngles
                {
                    public:
                        double dXO;    // Rotation in degrees around the x-axis.
                        double dYO;    // Rotation in degrees around the y-axis.
                        double dZO;    // Rotation in degrees around the z-axis.
                };

            public:
                // Declare struct public member variables.
                Translation stTranslation;
                EulerAngles stEulerAngles;

                /******************************************************************************
                 * @brief Construct a new Pose object.
                 *
                 * @param dX - The X position of the camera in ZED_MEASURE_UNITS.
                 * @param dY - The Y position of the camera in ZED_MEASURE_UNITS.
                 * @param dZ - The Z position of the camera in ZED_MEASURE_UNITS.
                 * @param dXO - The tilt of the camera around the X axis in degrees.
                 * @param dYO - The tilt of the camera around the Y axis in degrees.
                 * @param dZO - The tilt of the camera around the Z axis in degrees.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2024-04-17
                 ******************************************************************************/
                Pose(const double dX = 0.0, const double dY = 0.0, const double dZ = 0.0, const double dXO = 0.0, const double dYO = 0.0, const double dZO = 0.0)
                {
                    // Initialize member variables.
                    stTranslation.dX  = dX;
                    stTranslation.dY  = dY;
                    stTranslation.dZ  = dZ;
                    stEulerAngles.dXO = dXO;
                    stEulerAngles.dYO = dYO;
                    stEulerAngles.dZO = dZO;
                }
        };

        /******************************************************************************
         * @brief A lightweight snapshot of the camera's frequently-polled status. It is
         *      built and published once per producer iteration (on the owning thread,
         *      where all SDK access is legal) so status accessors become lock-free reads
         *      instead of foreign-thread SDK calls.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        struct CameraStatus
        {
            public:
                bool bCameraIsOpen                  = false;    // Whether the camera is currently open.
                bool bPositionalTrackingEnabled     = false;    // Whether positional tracking is enabled and healthy.
                bool bObjectDetectionEnabled        = false;    // Whether object detection is enabled.
                sl::SPATIAL_MAPPING_STATE eSpatialMappingState = sl::SPATIAL_MAPPING_STATE::NOT_ENABLED;    // Current spatial mapping state.
                sl::PositionalTrackingStatus stPositionalTrackingStatus;    // Full VIO positional tracking status.
                std::string szCameraModel = "NOT_OPENED";    // Camera model string ("NOT_OPENED" while closed).
        };

        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Construct a new ZEDCamera object.
         *
         * @param nPropResolutionX - The X resolution of the camera.
         * @param nPropResolutionY - The Y resolution of the camera.
         * @param nPropFramesPerSecond - The FPS of the camera.
         * @param dPropHorizontalFOV - The horizontal field of view.
         * @param dPropVerticalFOV - The vertical field of view.
         * @param bEnableRecordingFlag - Whether or not this camera should be recorded.
         * @param fMinSenseDistance - The minimum distance the camera can sense.
         * @param fMaxSenseDistance - The maximum distance the camera can sense.
         * @param bMemTypeGPU - Whether or not to use GPU memory.
         * @param bUseHalfDepthPrecision - Whether or not to use half depth precision.
         * @param nNumFrameRetrievalThreads - The number of threads to use for frame queueing and copying.
         * @param unCameraSerialNumber - The serial number of the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        ZEDCamera(const int nPropResolutionX,
                  const int nPropResolutionY,
                  const int nPropFramesPerSecond,
                  const double dPropHorizontalFOV,
                  const double dPropVerticalFOV,
                  const bool bEnableRecordingFlag,
                  const bool bMemTypeGPU,
                  const bool bUseHalfDepthPrecision,
                  const int nNumFrameRetrievalThreads,
                  const unsigned int unCameraSerialNumber) :
            Camera(nPropResolutionX,
                   nPropResolutionY,
                   nPropFramesPerSecond,
                   PIXEL_FORMATS::eZED,
                   dPropHorizontalFOV,
                   dPropVerticalFOV,
                   bEnableRecordingFlag,
                   nNumFrameRetrievalThreads)
        {
            // Initialize member variables. Some parameters are not used.
            (void) bMemTypeGPU;
            (void) bUseHalfDepthPrecision;
            m_unCameraSerialNumber = unCameraSerialNumber;

            // Teach the command queue how to tell whether this camera's thread is still able to
            // drain it. Without this, a command posted after the camera thread has stopped (for
            // example when the camera was never present and ThreadedContinuousCode() self-stopped)
            // would sit in the queue forever and block its caller forever.
            m_cmdQueue.SetDrainerLivenessCheck(
                [this]()
                {
                    // Only a starting or running thread will reach DrainAll() again.
                    const AutonomyThreadState eThreadState = this->GetThreadState();
                    return eThreadState == AutonomyThreadState::eStarting || eThreadState == AutonomyThreadState::eRunning;
                });
        }

        /******************************************************************************
         * @brief Destroy the ZEDCamera object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        virtual ~ZEDCamera() = default;

        // NOTE: All data delivery (frames, depth, point clouds, pose, floor plane, sensors and
        // detected objects) is handled by this class's publish-latest channels. Consumers hold a
        // Subscription to express demand and Get() the newest immutable snapshot, so there are no
        // per-consumer request methods on this interface. See the publisher accessors below.

        /******************************************************************************
         * @brief Resets the positional tracking of the camera.
         *
         * @return sl::ERROR_CODE - The error code returned by the ZED SDK.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual sl::ERROR_CODE ResetPositionalTracking() = 0;

        /******************************************************************************
         * @brief Tracks custom bounding boxes in the camera's field of view.
         *
         * @param vCustomBoxes - A vector of ZedObjectData structs representing the custom bounding boxes to track.
         * @return sl::ERROR_CODE - The error code returned by the ZED SDK.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual sl::ERROR_CODE TrackCustomBoxObjects(std::vector<ZedObjectData>& vCustomObjects)
        {
            // Initialize instance variables.
            (void) vCustomObjects;

            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "ZEDCamera::TrackCustomBoxObjects(const std::vector<ZedObjectData>& vCustomObjects) not implemented.");

            return sl::ERROR_CODE::FAILURE;
        }

        /******************************************************************************
         * @brief Reboots the camera.
         *
         * @return sl::ERROR_CODE - The error code returned by the ZED SDK.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual sl::ERROR_CODE RebootCamera() = 0;

        /////////////////////////////////////////
        // Setters for class member variables.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Enables the position tracking of the camera.
         *
         * @param fExpectedCameraHeightFromFloorTolerance - The expected camera height from the floor tolerance.
         * @return sl::ERROR_CODE - The error code returned by the ZED SDK.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual sl::ERROR_CODE EnablePositionalTracking(const float fExpectedCameraHeightFromFloorTolerance = constants::ZED_DEFAULT_FLOOR_PLANE_ERROR) = 0;

        /******************************************************************************
         * @brief Disables the position tracking of the camera.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual void DisablePositionalTracking() = 0;

        /******************************************************************************
         * @brief Mutator for the Positional Pose private member
         *
         * @param dX - The X position of the camera in ZED_MEASURE_UNITS.
         * @param dY - The Y position of the camera in ZED_MEASURE_UNITS.
         * @param dZ - The Z position of the camera in ZED_MEASURE_UNITS.
         * @param dXO - The tilt of the camera around the X axis in degrees.
         * @param dYO - The tilt of the camera around the Y axis in degrees.
         * @param dZO - The tilt of the camera around the Z axis in degrees.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual void SetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO) = 0;

        /******************************************************************************
         * @brief Enables spatial mapping.
         *
         * @return sl::ERROR_CODE - The error code returned by the ZED SDK.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual sl::ERROR_CODE EnableSpatialMapping()
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "ZEDCamera::EnableSpatialMapping() not implemented.");

            return sl::ERROR_CODE::FAILURE;
        }

        /******************************************************************************
         * @brief Disables spatial mapping.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual void DisableSpatialMapping()
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "ZEDCamera::DisableSpatialMapping() not implemented.");
        }

        /******************************************************************************
         * @brief Enables object detection.
         *
         * @param bEnableBatching - Whether or not to enable batching.
         * @return sl::ERROR_CODE - The error code returned by the ZED SDK.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual sl::ERROR_CODE EnableObjectDetection(const bool bEnableBatching = false)
        {
            // Initialize instance variables.
            (void) bEnableBatching;

            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "ZEDCamera::EnableObjectDetection(const bool bEnableBatching = false) not implemented.");

            return sl::ERROR_CODE::FAILURE;
        }

        /******************************************************************************
         * @brief Disables object detection.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual void DisableObjectDetection()
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "ZEDCamera::DisableObjectDetection() not implemented.");
        }

        /////////////////////////////////////////
        // Accessors for class member variables.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Accessor for the Using G P U Memory private member.
         *
         * @return true - We are using GPU memory.
         * @return false - We are not using GPU memory.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual bool GetUsingGPUMem() const { return false; }

        /******************************************************************************
         * @brief Accessor for the Camera Model private member.
         *
         * @return std::string - The model of the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual std::string GetCameraModel() = 0;

        /******************************************************************************
         * @brief Accessor for the Camera Serial private member.
         *
         * @return unsigned int - The serial number of the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual unsigned int GetCameraSerial() { return m_unCameraSerialNumber; };

        /******************************************************************************
         * @brief Accessor for the Positional Tracking Enabled private member.
         *
         * @return true - Positional tracking is enabled.
         * @return false - Positional tracking is not enabled.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual bool GetPositionalTrackingEnabled() = 0;

        /******************************************************************************
         * @brief Accessor for the Positional Tracking State private member.
         *
         * @return sl::PositionalTrackingStatus - The positional tracking state.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual sl::PositionalTrackingStatus GetPositionalTrackingState()
        {
            // Initialize instance variable.
            sl::PositionalTrackingStatus stStatus;

            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "ZEDCamera::GetPositionalTrackingState() not implemented.");

            return stStatus;
        }

        /******************************************************************************
         * @brief Accessor for the Spatial Mapping State private member.
         *
         * @return sl::SPATIAL_MAPPING_STATE - The spatial mapping state.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual sl::SPATIAL_MAPPING_STATE GetSpatialMappingState() { return sl::SPATIAL_MAPPING_STATE::NOT_ENABLED; }

        /******************************************************************************
         * @brief Copies the newest published spatial mapping mesh snapshot into the given destination.
         *
         * @param fuMeshFuture - A future that should be waited on before the passed in Mesh is used.
         * @return sl::SPATIAL_MAPPING_STATE - The spatial mapping state.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual sl::SPATIAL_MAPPING_STATE ExtractSpatialMapAsync(std::future<sl::Mesh>& fuMeshFuture)
        {
            // Initialize instance variables.
            (void) fuMeshFuture;

            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "ZEDCamera::ExtractSpatialMapAsync(std::future<sl::Mesh>& fuMeshFuture) not implemented.");

            return sl::SPATIAL_MAPPING_STATE::NOT_ENABLED;
        }

        /******************************************************************************
         * @brief Accessor for the Object Detection Enabled private member.
         *
         * @return true - Object detection is enabled.
         * @return false - Object detection is not enabled.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual bool GetObjectDetectionEnabled() { return false; }

        /////////////////////////////////////////
        // Publish-latest data channels. Consumers Subscribe() to express demand and
        // Get() the newest immutable snapshot without blocking the producer. Both the
        // real ZEDCam and the simulated SIMZEDCam publish through these same channels,
        // so consumers stay drop-in interchangeable. A camera in CPU memory mode
        // publishes the CPU channels; a camera in GPU mode publishes the GPU channels.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Accessor for the BGRA frame publisher (CPU memory).
         * @return pubsub::Publisher<cv::Mat>& - The CPU frame channel.
         ******************************************************************************/
        pubsub::Publisher<cv::Mat>& GetFrameCPUPublisher() { return m_pubFrameCPU; }

        /******************************************************************************
         * @brief Accessor for the BGRA frame publisher (GPU memory).
         * @return pubsub::Publisher<cv::cuda::GpuMat>& - The GPU frame channel.
         ******************************************************************************/
        pubsub::Publisher<cv::cuda::GpuMat>& GetFrameGPUPublisher() { return m_pubFrameGPU; }

        /******************************************************************************
         * @brief Accessor for the depth measure publisher (CPU memory).
         * @return pubsub::Publisher<cv::Mat>& - The CPU depth measure channel.
         ******************************************************************************/
        pubsub::Publisher<cv::Mat>& GetDepthMeasureCPUPublisher() { return m_pubDepthMeasureCPU; }

        /******************************************************************************
         * @brief Accessor for the depth measure publisher (GPU memory).
         * @return pubsub::Publisher<cv::cuda::GpuMat>& - The GPU depth measure channel.
         ******************************************************************************/
        pubsub::Publisher<cv::cuda::GpuMat>& GetDepthMeasureGPUPublisher() { return m_pubDepthMeasureGPU; }

        /******************************************************************************
         * @brief Accessor for the depth image (grayscale) publisher (CPU memory).
         * @return pubsub::Publisher<cv::Mat>& - The CPU depth image channel.
         ******************************************************************************/
        pubsub::Publisher<cv::Mat>& GetDepthImageCPUPublisher() { return m_pubDepthImageCPU; }

        /******************************************************************************
         * @brief Accessor for the depth image (grayscale) publisher (GPU memory).
         * @return pubsub::Publisher<cv::cuda::GpuMat>& - The GPU depth image channel.
         ******************************************************************************/
        pubsub::Publisher<cv::cuda::GpuMat>& GetDepthImageGPUPublisher() { return m_pubDepthImageGPU; }

        /******************************************************************************
         * @brief Accessor for the point cloud publisher (CPU memory).
         * @return pubsub::Publisher<cv::Mat>& - The CPU point cloud channel.
         ******************************************************************************/
        pubsub::Publisher<cv::Mat>& GetPointCloudCPUPublisher() { return m_pubPointCloudCPU; }

        /******************************************************************************
         * @brief Accessor for the point cloud publisher (GPU memory).
         * @return pubsub::Publisher<cv::cuda::GpuMat>& - The GPU point cloud channel.
         ******************************************************************************/
        pubsub::Publisher<cv::cuda::GpuMat>& GetPointCloudGPUPublisher() { return m_pubPointCloudGPU; }

        /******************************************************************************
         * @brief Accessor for the realigned positional pose publisher.
         * @return pubsub::Publisher<Pose>& - The pose channel.
         ******************************************************************************/
        pubsub::Publisher<Pose>& GetPosePublisher() { return m_pubPose; }

        /******************************************************************************
         * @brief Accessor for the floor plane publisher.
         * @return pubsub::Publisher<sl::Plane>& - The floor plane channel.
         ******************************************************************************/
        pubsub::Publisher<sl::Plane>& GetFloorPlanePublisher() { return m_pubFloorPlane; }

        /******************************************************************************
         * @brief Accessor for the sensors data publisher.
         * @return pubsub::Publisher<sl::SensorsData>& - The sensors channel.
         ******************************************************************************/
        pubsub::Publisher<sl::SensorsData>& GetSensorsPublisher() { return m_pubSensors; }

        /******************************************************************************
         * @brief Accessor for the detected objects publisher.
         * @return pubsub::Publisher<std::vector<sl::ObjectData>>& - The objects channel.
         ******************************************************************************/
        pubsub::Publisher<std::vector<sl::ObjectData>>& GetObjectsPublisher() { return m_pubObjects; }

        /******************************************************************************
         * @brief Accessor for the batched detected objects publisher.
         * @return pubsub::Publisher<std::vector<sl::ObjectsBatch>>& - The batched objects channel.
         ******************************************************************************/
        pubsub::Publisher<std::vector<sl::ObjectsBatch>>& GetBatchedObjectsPublisher() { return m_pubBatchedObjects; }

        /******************************************************************************
         * @brief Accessor for the camera status publisher (lock-free status reads).
         * @return pubsub::Publisher<CameraStatus>& - The status channel.
         ******************************************************************************/
        pubsub::Publisher<CameraStatus>& GetStatusPublisher() { return m_pubStatus; }

    protected:
        /////////////////////////////////////////
        // Declare protected methods.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Check whether an SDK command may safely be executed inline, on the calling
         *      thread, instead of being posted to the producer thread.
         *
         *      This is true exactly when no producer thread exists: either it has never been
         *      started (the camera is still being constructed/configured) or it has been
         *      joined by Stop(). In both cases nothing will ever drain the command queue, so
         *      posting would cancel the command and silently discard the caller's intent -
         *      and because no producer thread is running, the caller is the only thread that
         *      can reach the SDK, so running inline preserves single-threaded access.
         *
         * @return true - No producer thread exists; run the command inline.
         * @return false - The producer thread is starting or running; post the command to it.
         *
         * @note This assumes configuration calls are not made concurrently with Start(). A
         *      camera must be fully configured before Start(), or after Stop() has joined.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-26
         ******************************************************************************/
        bool CanRunCommandInline() const
        {
            // A shut down queue means the object is being destroyed; do not run anything new.
            if (m_cmdQueue.IsShutdown())
            {
                // Let the normal (cancelling) path handle it.
                return false;
            }

            // Only a fully stopped thread guarantees there is no concurrent SDK access.
            return this->GetThreadState() == AutonomyThreadState::eStopped;
        }

        /******************************************************************************
         * @brief Run a value-returning command on the camera's owning thread and wait for
         *      its result. If the owning thread has stopped (or stops while we wait) the
         *      command can never run, so the caller is released with the supplied fallback
         *      instead of being blocked forever.
         *
         * @tparam R - The return type of the command.
         * @param fnCommand - The command to run on the owning thread.
         * @param tFallbackOnCancel - The value to return if the command could not be run.
         * @param szCommandName - Human readable command name, used only for logging.
         * @return R - The command's result, or tFallbackOnCancel if it was cancelled.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        template<typename R>
        R RunOnOwningThread(std::function<R()> fnCommand, const R& tFallbackOnCancel, const std::string& szCommandName)
        {
            // If no producer thread exists there is nothing to drain the queue, so a posted command
            // would be cancelled and the caller silently handed the fallback. Run it inline instead:
            // with the producer thread stopped (or never started) this thread is the only one that
            // can touch the SDK, so the single-threaded-access invariant still holds.
            if (this->CanRunCommandInline())
            {
                // Execute directly on the caller's thread.
                return fnCommand();
            }

            try
            {
                // Post the command and wait for the owning thread to run it.
                return m_cmdQueue.PostAndWait<R>(std::move(fnCommand)).get();
            }
            catch (const std::exception& stdError)
            {
                // The camera thread is not running, so this command will never execute.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Camera command '{}' was not executed because the camera thread is not running: {}",
                            szCommandName,
                            stdError.what());
                // Release the caller with the fallback rather than leaving it blocked.
                return tFallbackOnCancel;
            }
        }

        /******************************************************************************
         * @brief Run a void command on the camera's owning thread and wait for it to
         *      finish. Behaves like RunOnOwningThread() but has no result to fall back to.
         *
         * @param fnCommand - The command to run on the owning thread.
         * @param szCommandName - Human readable command name, used only for logging.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        void RunOnOwningThreadVoid(std::function<void()> fnCommand, const std::string& szCommandName)
        {
            // See RunOnOwningThread(): with no producer thread to drain the queue, run inline so the
            // command actually takes effect instead of being cancelled.
            if (this->CanRunCommandInline())
            {
                // Execute directly on the caller's thread.
                fnCommand();
                return;
            }

            try
            {
                // Post the command and wait for the owning thread to run it.
                m_cmdQueue.PostAndWait<void>(std::move(fnCommand)).get();
            }
            catch (const std::exception& stdError)
            {
                // The camera thread is not running, so this command will never execute.
                LOG_WARNING(logging::g_qSharedLogger,
                            "Camera command '{}' was not executed because the camera thread is not running: {}",
                            szCommandName,
                            stdError.what());
            }
        }

        /////////////////////////////////////////
        // Declare protected member variables.
        /////////////////////////////////////////

        // ZED Camera specific.
        unsigned int m_unCameraSerialNumber;

        // Control channel in: foreign threads Post() SDK-mutating commands here and the
        // owning producer thread DrainAll()s them on itself, so all SDK access is single
        // threaded by construction.
        CommandQueue m_cmdQueue;

        // Publish-latest data channels out (see accessors above). Every channel is given an
        // explicit preallocation and growth ceiling so steady state allocates nothing and a
        // consumer that leaks snapshots trips the ceiling and is logged as an error.
        pubsub::Publisher<cv::Mat> m_pubFrameCPU{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<cv::cuda::GpuMat> m_pubFrameGPU{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<cv::Mat> m_pubDepthMeasureCPU{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<cv::cuda::GpuMat> m_pubDepthMeasureGPU{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<cv::Mat> m_pubDepthImageCPU{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<cv::cuda::GpuMat> m_pubDepthImageGPU{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<cv::Mat> m_pubPointCloudCPU{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<cv::cuda::GpuMat> m_pubPointCloudGPU{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<Pose> m_pubPose{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<sl::Plane> m_pubFloorPlane{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<sl::SensorsData> m_pubSensors{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<std::vector<sl::ObjectData>> m_pubObjects{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<std::vector<sl::ObjectsBatch>> m_pubBatchedObjects{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<CameraStatus> m_pubStatus{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
};

#endif
