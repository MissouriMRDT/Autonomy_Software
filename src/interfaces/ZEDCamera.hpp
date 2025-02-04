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
         * @param bEnableFusionMaster - Whether or not to enable the fusion master.
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
                  const bool bEnableFusionMaster,
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
            (void) bEnableFusionMaster;
            m_bCameraIsFusionMaster = bEnableFusionMaster;
            m_unCameraSerialNumber  = unCameraSerialNumber;
        }

        /******************************************************************************
         * @brief Destroy the ZEDCamera object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        virtual ~ZEDCamera() = default;

        /******************************************************************************
         * Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
         *      Remember, this code will be ran in whatever, class/thread calls it.
         *
         * @param cvFrame - A reference to the cv::Mat to store the frame in.
         * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
         *                          Value will be true if frame was successfully retrieved.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        std::future<bool> RequestFrameCopy(cv::Mat& cvFrame) override = 0;

        /******************************************************************************
         * @brief Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
         *
         * @param cvGPUFrame - A reference to the cv::cuda::GpuMat to store the frame in.
         * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        virtual std::future<bool> RequestFrameCopy(cv::cuda::GpuMat& cvGPUFrame)
        {
            // Create instance variables.
            (void) cvGPUFrame;
            std::promise<bool> pmPromise;
            std::future<bool> fuFuture = pmPromise.get_future();

            // Immediately set the promise to false.
            pmPromise.set_value(false);

            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "ZEDCamera::RequestFrameCopy(cv::cuda::GpuMat& cvGPUFrame) not implemented. If SIM_MODE use cv::Mat version instead.");

            return fuFuture;
        }

        /******************************************************************************
         * @brief Puts a frame pointer into a queue so a copy of a depth frame from the camera can be written to it.
         *
         * @param cvDepth - A reference to the cv::Mat to store the depth frame in.
         * @param bRetrieveMeasure - Whether or not to retrieve the depth measure or just the image.
         * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        virtual std::future<bool> RequestDepthCopy(cv::Mat& cvDepth, const bool bRetrieveMeasure = true) = 0;

        /******************************************************************************
         * @brief Puts a frame pointer into a queue so a copy of a depth frame from the camera can be written to it.
         *
         * @param cvGPUDepth - A reference to the cv::cuda::GpuMat to store the depth frame in.
         * @param bRetrieveMeasure - Whether or not to retrieve the depth measure or just the image.
         * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        virtual std::future<bool> RequestDepthCopy(cv::cuda::GpuMat& cvGPUDepth, const bool bRetrieveMeasure = true)
        {
            // Initialize instance variables.
            (void) cvGPUDepth;
            (void) bRetrieveMeasure;
            std::promise<bool> pmPromise;

            // Immediately set the promise to false.
            pmPromise.set_value(false);

            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger,
                      "ZEDCamera::RequestDepthCopy(cv::cuda::GpuMat& cvGPUDepth, const bool bRetrieveMeasure = true) not implemented. If SIM_MODE use cv::Mat version "
                      "instead.");

            return pmPromise.get_future();
        }

        /******************************************************************************
         * @brief Puts a frame pointer into a queue so a copy of a point cloud from the camera can be written to it.
         *
         * @param cvPointCloud - A reference to the cv::Mat to store the point cloud in.
         * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        virtual std::future<bool> RequestPointCloudCopy(cv::Mat& cvPointCloud) = 0;

        /******************************************************************************
         * @brief Puts a frame pointer into a queue so a copy of a point cloud from the camera can be written to it.
         *
         * @param cvGPUPointCloud - A reference to the cv::cuda::GpuMat to store the point cloud in.
         * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual std::future<bool> RequestPointCloudCopy(cv::cuda::GpuMat& cvGPUPointCloud)
        {
            // Initialize instance variables.
            (void) cvGPUPointCloud;
            std::promise<bool> pmPromise;

            // Immediately set the promise to false.
            pmPromise.set_value(false);

            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger,
                      "ZEDCamera::RequestPointCloudCopy(cv::cuda::GpuMat& cvGPUPointCloud) not implemented. If SIM_MODE use cv::Mat version instead.");

            return pmPromise.get_future();
        }

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

        /******************************************************************************
         * @brief Subscribes the fusion object to the camera with the given UUID.
         *
         * @param slCameraUUID - The UUID of the camera to subscribe to.
         * @return sl::FUSION_ERROR_CODE - The error code returned by the ZED SDK.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual sl::FUSION_ERROR_CODE SubscribeFusionToCameraUUID(sl::CameraIdentifier& slCameraUUID) = 0;

        /******************************************************************************
         * @brief Publishes the camera to the fusion object.
         *
         * @return sl::CameraIdentifier - The identifier of the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual sl::CameraIdentifier PublishCameraToFusion() = 0;

        /******************************************************************************
         * @brief Ingests GPS data into the fusion object.
         *
         * @param stNewGPSLocation - The new GPS location to ingest.
         * @return sl::FUSION_ERROR_CODE - The error code returned by the ZED SDK.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual sl::FUSION_ERROR_CODE IngestGPSDataToFusion(geoops::GPSCoordinate stNewGPSLocation)
        {
            // Initialize instance variables.
            (void) stNewGPSLocation;

            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "ZEDCamera::IngestGPSDataToFusion(geoops::GPSCoordinate stNewGPSLocation) not implemented.");

            return sl::FUSION_ERROR_CODE::FAILURE;
        }

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
         * @brief Accessor for if this ZED is running a fusion instance.
         *
         * @return true - This ZEDCam is a fusion master and is running an sl::Fusion instance.
         * @return false - This ZEDCam is not a fusion master and is not running a sl::Fusion instance.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-01-26
         ******************************************************************************/
        virtual bool GetIsFusionMaster() const { return m_bCameraIsFusionMaster; }

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
         * @brief Puts a Pose pointer into a queue so a copy of a Pose from the camera can be written to it.
         *
         * @param stPose - A reference to the Pose to store the Pose in.
         * @return std::future<bool> - A future that should be waited on before the passed in Pose is used.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual std::future<bool> RequestPositionalPoseCopy(Pose& stPose) = 0;

        /******************************************************************************
         * @brief Puts a GeoPose pointer into a queue so a copy of a GeoPose from the camera can be written to it.
         *
         * @param slGeoPose - A reference to the sl::GeoPose to store the GeoPose in.
         * @return std::future<bool> - A future that should be waited on before the passed in GeoPose is used.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual std::future<bool> RequestFusionGeoPoseCopy(sl::GeoPose& slGeoPose) = 0;

        /******************************************************************************
         * @brief Puts a FloorPlane pointer into a queue so a copy of a FloorPlane from the camera can be written to it.
         *
         * @param slFloorPlane - A reference to the sl::Plane to store the FloorPlane in.
         * @return std::future<bool> - A future that should be waited on before the passed in FloorPlane is used.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual std::future<bool> RequestFloorPlaneCopy(sl::Plane& slFloorPlane)
        {
            // Initialize instance variables.
            (void) slFloorPlane;
            std::promise<bool> pmPromise;

            // Immediately set the promise to false.
            pmPromise.set_value(false);

            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "ZEDCamera::RequestFloorPlaneCopy(sl::Plane& slFloorPlane) not implemented.");

            return pmPromise.get_future();
        }

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
         * @brief Accessor for the Fused Positional Tracking State private member.
         *
         * @return sl::FusedPositionalTrackingStatus - The fused positional tracking state.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual sl::FusedPositionalTrackingStatus GetFusedPositionalTrackingState()
        {
            // Initialize instance variable.
            sl::FusedPositionalTrackingStatus stStatus;

            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "ZEDCamera::GetFusedPositionalTrackingState() not implemented.");

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
         * @brief Puts a Mesh pointer into a queue so a copy of a spatial mapping mesh from the camera can be written to it.
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

        /******************************************************************************
         * @brief Puts a vector of ObjectData pointers into a queue so a copy of a vector of ObjectData from the camera can be written to it.
         *
         * @param vObjectData - A reference to the vector of sl::ObjectData to store the ObjectData in.
         * @return std::future<bool> - A future that should be waited on before the passed in ObjectData is used.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual std::future<bool> RequestObjectsCopy(std::vector<sl::ObjectData>& vObjectData)
        {
            // Initialize instance variables.
            (void) vObjectData;
            std::promise<bool> pmPromise;

            // Immediately set the promise to false.
            pmPromise.set_value(false);

            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "ZEDCamera::RequestObjectsCopy(std::vector<sl::ObjectData>& vObjectData) not implemented.");

            return pmPromise.get_future();
        }

        /******************************************************************************
         * @brief Puts a vector of ObjectsBatch pointers into a queue so a copy of a vector of ObjectsBatch from the camera can be written to it.
         *
         * @param vBatchedObjectData - A reference to the vector of sl::ObjectsBatch to store the ObjectsBatch in.
         * @return std::future<bool> - A future that should be waited on before the passed in ObjectsBatch is used.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual std::future<bool> RequestBatchedObjectsCopy(std::vector<sl::ObjectsBatch>& vBatchedObjectData)
        {
            // Initialize instance variables.
            (void) vBatchedObjectData;
            std::promise<bool> pmPromise;

            // Immediately set the promise to false.
            pmPromise.set_value(false);

            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "ZEDCamera::RequestBatchedObjectsCopy(std::vector<sl::ObjectsBatch>& vBatchedObjectData) not implemented.");

            return pmPromise.get_future();
        }

    protected:
        /////////////////////////////////////////
        // Declare class constants.
        /////////////////////////////////////////

        const std::memory_order ATOMIC_MEMORY_ORDER_METHOD = std::memory_order_relaxed;

        /////////////////////////////////////////
        // Declare protected member variables.
        /////////////////////////////////////////

        // ZED Camera specific.
        unsigned int m_unCameraSerialNumber;
        bool m_bCameraIsFusionMaster;

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
};

#endif
