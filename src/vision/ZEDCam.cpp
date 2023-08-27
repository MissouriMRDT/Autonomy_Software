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
#include "../AutonomyLogging.h"

/******************************************************************************
 * @brief Construct a new Zed Cam:: Zed Cam object.
 *
 * @param unCameraSerialNumber - The serial number of the camera to open.
 * @param nPropResolutionX - X res of camera.
 * @param nPropResolutionY - Y res of camera.
 * @param nPropFramesPerSecond - FPS camera is running at.
 * @param dPropHorizontalFOV - The horizontal field of view.
 * @param dPropVerticalFOV - The vertical field of view.
 * @param bMemTypeGPU - Whether or not to use the GPU memory for operations.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
ZEDCam::ZEDCam(const unsigned int unCameraSerialNumber,
               const int nPropResolutionX,
               const int nPropResolutionY,
               const int nPropFramesPerSecond,
               const double dPropHorizontalFOV,
               const double dPropVerticalFOV,
               const float fMinSenseDistance,
               const float fMaxSenseDistance,
               const bool bMemTypeGPU) :
    Camera(nPropResolutionX, nPropResolutionY, nPropFramesPerSecond, PIXEL_FORMATS::eZED, dPropHorizontalFOV, dPropVerticalFOV)
{
    // Assign member variables.
    bMemTypeGPU ? m_slMemoryType = sl::MEM::GPU : m_slMemoryType = sl::MEM::CPU;

    // Setup camera params.
    m_slCameraParams.input.setFromSerialNumber(unCameraSerialNumber);
    m_slCameraParams.camera_resolution      = constants::ZED_BASE_RESOLUTION;
    m_slCameraParams.camera_fps             = nPropFramesPerSecond;
    m_slCameraParams.coordinate_units       = constants::ZED_MEASURE_UNITS;
    m_slCameraParams.coordinate_system      = constants::ZED_COORD_SYSTEM;
    m_slCameraParams.depth_mode             = constants::ZED_DEPTH_MODE;
    m_slCameraParams.depth_minimum_distance = fMinSenseDistance;
    m_slCameraParams.depth_maximum_distance = fMaxSenseDistance;
    m_slCameraParams.depth_stabilization    = constants::ZED_DEPTH_STABILIZATION;

    // Setup camera runtime params.
    m_slRuntimeParams.enable_fill_mode = constants::ZED_SENSING_FILL;

    // Attempt to open camera.
    m_slCamera.open(m_slCameraParams);
    // Check if the camera was successfully opened.
    if (m_slCamera.isOpened())
    {
        // Submit logger message.
        LOG_DEBUG(g_qSharedLogger,
                  "{} stereo camera with serial number {} has been succsessfully opened.",
                  this->GetCameraModel(),
                  m_slCamera.getCameraInformation().serial_number);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(g_qSharedLogger, "Unable to open stereo camera with serial number {}", unCameraSerialNumber);
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
    // Delete dynamically allocated memory.
    delete m_pIPSDepth;
    delete m_pIPSPointCloud;

    // Set dangling pointers to point to null.
    m_pIPSDepth      = nullptr;
    m_pIPSPointCloud = nullptr;

    // Close the ZEDCam.
    m_slCamera.close();

    // Free all mats and other sl namespace objects.
    m_slFrame.free();
    m_slDepth.free();
    m_slPointCloud.free();
}

/******************************************************************************
 * @brief Grabs a regular BGRA image from the LEFT eye of the zed camera.
 *
 * @param bGrabRaw - Whether or not to apply class properties to image. (resize, colorspace change, etc.)
 *              If bGrabRaw is set to true, then the ZED_BASE_RESOLUTION that is set in AutonomyContants.h
 *              will be used.
 * @return sl::Mat - The result image stored in an sl::Mat object.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
sl::Mat ZEDCam::GrabFrame(const bool bGrabRaw)
{
    // Call generalized update method of zed api.
    sl::ERROR_CODE slReturnCode = m_slCamera.grab(m_slRuntimeParams);

    // Update zed api.
    if (slReturnCode == sl::ERROR_CODE::SUCCESS)
    {
        // Check if we should resize the grabbed image.
        if (bGrabRaw)
        {
            // Grab regular image and store it in member variable.
            slReturnCode = m_slCamera.retrieveImage(m_slFrame, constants::ZED_RETRIEVE_VIEW, m_slMemoryType);
        }
        else
        {
            // Grab regular resized image and store it in member variable.
            slReturnCode = m_slCamera.retrieveImage(m_slFrame, constants::ZED_RETRIEVE_VIEW, m_slMemoryType, sl::Resolution(m_nPropResolutionX, m_nPropResolutionY));
        }

        // Check if a new image was successfully retrieved.
        if (slReturnCode == sl::ERROR_CODE::SUCCESS)
        {
            // Call FPS tick.
            m_pIPS->Tick();
        }
        else
        {
            // Submit logger message.
            LOG_WARNING(g_qSharedLogger,
                        "Failed to retrieve image for stereo camera {} ({}).",
                        sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                        m_slCamera.getCameraInformation().serial_number);
        }
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(g_qSharedLogger,
                    "Unable to update stereo camera {} ({}) images and sensors!",
                    sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                    m_slCamera.getCameraInformation().serial_number);
    }

    // Return a copy of the frame member variable.
    return m_slFrame;
}

/******************************************************************************
 * @brief Grabs a depth measure image from the camera. This image has the same shape as
 *      a grayscale image, but the values represent the depth in ZED_MEASURE_UNITS that is set in
 *      AutonomyConstants.h.
 *
 * @param bGrabRaw - Whether or not to apply class properties to image. (resize)
 *              If bGrabRaw is set to true, then the ZED_BASE_RESOLUTION that is set in AutonomyContants.h
 *              will be used.
 * @param bHalfPrecision - The accuracy to use for the depth measurement. Full = float32, Half = unsigned long16.
 * @return sl::Mat - The result depth image stored in an sl::Mat object.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
sl::Mat ZEDCam::GrabDepth(const bool bGrabRaw, const bool bHalfPrecision)
{
    // Declare instance variables.
    sl::MEASURE slMeasureType;

    // Determine if we are using float32 or unsigned long16.
    bHalfPrecision ? slMeasureType = sl::MEASURE::DEPTH_U16_MM : slMeasureType = sl::MEASURE::DEPTH;
    // Call generalized update method of zed api.
    sl::ERROR_CODE slReturnCode = m_slCamera.grab(m_slRuntimeParams);

    // Update zed api.
    if (slReturnCode == sl::ERROR_CODE::SUCCESS)
    {
        // Check if we should resize the grabbed image.
        if (bGrabRaw)
        {
            // Grab regular image and store it in member variable.
            slReturnCode = m_slCamera.retrieveMeasure(m_slDepth, slMeasureType, m_slMemoryType);
        }
        else
        {
            // Grab regular resized image and store it in member variable.
            slReturnCode = m_slCamera.retrieveMeasure(m_slDepth, slMeasureType, m_slMemoryType, sl::Resolution(m_nPropResolutionX, m_nPropResolutionY));
        }

        // Check if a new image was successfully retrieved.
        if (slReturnCode == sl::ERROR_CODE::SUCCESS)
        {
            // Call FPS tick.
            m_pIPSDepth->Tick();
        }
        else
        {
            // Submit logger message.
            LOG_WARNING(g_qSharedLogger,
                        "Failed to retrieve depth measure for stereo camera {} ({}).",
                        sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                        m_slCamera.getCameraInformation().serial_number);
        }
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(g_qSharedLogger,
                    "Unable to update stereo camera {} ({}) measurements and sensors!",
                    sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                    m_slCamera.getCameraInformation().serial_number);
    }

    // Return a copy of the frame member variable.
    return m_slDepth;
}

/******************************************************************************
 * @brief Grabs a point cloud image from the camera. This image has the same resolution as a normal
 *      image but with three XYZ values replacing the old color values in the 3rd dimension. A 4th value can be
 *      added to the 3rd dimension to get color (X, Y, Z, color(BGRA)), but this is slower.
 *      The units and sign of the XYZ values are determined by ZED_MEASURE_UNITS and ZED_COORD_SYSTEM
 *      constants set in AutonomyConstants.h.
 *
 * @param bGrabRaw - Whether or not to apply class properties to image. (resize)
 *              If bGrabRaw is set to true, then the ZED_BASE_RESOLUTION that is set in AutonomyContants.h
 *              will be used.
 * @param bIncludeColor - Whether or not a unsigned char[4] should be appended to the 3rd dimension after the XYZ values.
 * @return sl::Mat - The result point cloud image with pixel colors and real-world locations.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
sl::Mat ZEDCam::GrabPointCloud(const bool bGrabRaw, const bool bIncludeColor)
{
    // Declare instance variables.
    sl::MEASURE slMeasureType;

    // Determine if we are using float32 or unsigned long16.
    bIncludeColor ? slMeasureType = sl::MEASURE::XYZBGRA : slMeasureType = sl::MEASURE::XYZ;
    // Call generalized update method of zed api.
    sl::ERROR_CODE slReturnCode = m_slCamera.grab(m_slRuntimeParams);

    // Update zed api.
    if (slReturnCode == sl::ERROR_CODE::SUCCESS)
    {
        // Check if we should resize the grabbed image.
        if (bGrabRaw)
        {
            // Grab regular image and store it in member variable.
            slReturnCode = m_slCamera.retrieveMeasure(m_slPointCloud, slMeasureType, m_slMemoryType);
        }
        else
        {
            // Grab regular resized image and store it in member variable.
            slReturnCode = m_slCamera.retrieveMeasure(m_slPointCloud, slMeasureType, m_slMemoryType, sl::Resolution(m_nPropResolutionX, m_nPropResolutionY));
        }

        // Check if a new image was successfully retrieved.
        if (slReturnCode == sl::ERROR_CODE::SUCCESS)
        {
            // Call FPS tick.
            m_pIPSPointCloud->Tick();
        }
        else
        {
            // Submit logger message.
            LOG_WARNING(g_qSharedLogger,
                        "Failed to retrieve point cloud for stereo camera {} ({}).",
                        sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                        m_slCamera.getCameraInformation().serial_number);
        }
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(g_qSharedLogger,
                    "Unable to update stereo camera {} ({}) measurements and sensors!",
                    sl::toString(m_slCamera.getCameraInformation().camera_model).get(),
                    m_slCamera.getCameraInformation().serial_number);
    }

    // Return a copy of the frame member variable.
    return m_slPointCloud;
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
    // Reset the positional location of the camera. Maintain the orientation.
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
sl::ERROR_CODE ZEDCam::TrackCustomBoxObjects(std::vector<sl::CustomBoxObjectData> vCustomObjects)
{
    // Pass objects to ZEDSDK.
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
    // Reboot the this camera and return the status code.
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
    //
}

/******************************************************************************
 * @brief Disable to positional tracking funcionality of the camera.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
void ZEDCam::DisablePositionalTracking() {}

/******************************************************************************
 * @brief Sets the pose of the positional tracking of the camera. XYZ will point
 *      in their respective directions according to ZED_COORD_SYSTEM defined in
 *      AutonomyConstants.h.
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
sl::ERROR_CODE ZEDCam::SetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO) {}

/******************************************************************************
 * @brief Enabled the spatial mapping feature of the camera.
 *
 * @return sl::ERROR_CODE - Whether or not spatial mapping was successfully enabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::EnableSpatialMapping() {}

/******************************************************************************
 * @brief Disabled the spatial mapping feature of the camera.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
void ZEDCam::DisableSpatialMapping() {}

/******************************************************************************
 * @brief Enables the object detection and tracking feature of the camera.
 *
 * @return sl::ERROR_CODE - Whether or not object detection/tracking was successfully enabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
sl::ERROR_CODE ZEDCam::EnableObjectTracking() {}

/******************************************************************************
 * @brief Disables the object detection and tracking feature of the camera.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
void ZEDCam::DisableObjectTracking() {}

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
    // Return if the ZED camera is currently opened.
    return m_slCamera.isOpened();
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
 * @brief Returns the current pose of the camera. If positional tracking is not
 *      enabled, this method will return an defualt initialized pose object.
 *
 * @param slPositionReference - An sl::REFERENCE_FRAME enum that specifies whether to return the pose relative
 *                          to the last camera position or the camera's start position.
 * @return sl::Pose - The current pose of the camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
sl::Pose ZEDCam::GetPositionalPose(const sl::REFERENCE_FRAME slPositionReference)
{
    // Check if positional tracking has been enabled.
    if (m_slCamera.isPositionalTrackingEnabled())
    {
        // Get the current pose of the camera.
        sl::POSITIONAL_TRACKING_STATE slReturnCode = m_slCamera.getPosition(m_slCameraPose, slPositionReference);

        // Check if the tracking state is anything other than OK.
        if (slReturnCode != sl::POSITIONAL_TRACKING_STATE::OK)
        {
            // Submit logger message.
            LOG_WARNING(g_qSharedLogger, "Getting ZED positional pose returned non-OK status! sl::POSITIONAL_TRACKING_STATE is: {}", sl::toString(slReturnCode).get());
        }
    }

    // Return the pose object of the camera with respect to either its last position or its start.
    return m_slCameraPose;
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
    // Return is positional tracking is enabled.
    return m_slCamera.isPositionalTrackingEnabled();
}

/******************************************************************************
 * @brief Gets the IMU data from the ZED camera. If getting the data fails, the
 *      last successfully retrieved value is returned.
 *
 * @return std::vector<double> - A 1x6 vector containing X_deg, X_deg, X_deg, X_liner_accel, Y_liner_accel, Z_liner_accel.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
std::vector<double> ZEDCam::GetIMUData()
{
    // Create instance variables.
    std::vector<double> vIMUAnglesAndAccel;

    // Get and store the SensorData object from the camera. Get data from the most recent image grab.
    // Using TIME_REFERENCE::CURRENT requires high rate polling and can introduce error as the most recent
    // IMU data could be in the future of the camera image.
    sl::ERROR_CODE slReturnCode = m_slCamera.getSensorsData(m_slSensorData, sl::TIME_REFERENCE::IMAGE);

    // Check if the sensor data was retrieved correctly.
    if (slReturnCode == sl::ERROR_CODE::SUCCESS)
    {
        // Get IMU orientation in degrees.
        sl::float3 slAngles = m_slSensorData.imu.pose.getEulerAngles(false);
        // Get IMU linear acceleration.
        sl::float3 slLinearAccels = m_slSensorData.imu.linear_acceleration;

        // Repackage angles and accels into vector.
        vIMUAnglesAndAccel.emplace_back(slAngles.x);
        vIMUAnglesAndAccel.emplace_back(slAngles.y);
        vIMUAnglesAndAccel.emplace_back(slAngles.z);
        vIMUAnglesAndAccel.emplace_back(slLinearAccels.x);
        vIMUAnglesAndAccel.emplace_back(slLinearAccels.y);
        vIMUAnglesAndAccel.emplace_back(slLinearAccels.z);
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(g_qSharedLogger, "Failed to get data from ZED sensors package. sl::ERROR_CODE number: {}", sl::toString(slReturnCode).get());
    }

    // Return sensors data.
    return vIMUAnglesAndAccel;
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
 * @brief Retrieve the
 *
 * @return std::future<sl::FusedPointCloud> -
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
std::future<sl::FusedPointCloud> ZEDCam::ExtractSpatialMapAsync()
{
    // Check if spatial mapping has been enabled and ready
    if (m_slCamera.getSpatialMappingState() == sl::SPATIAL_MAPPING_STATE::OK)
    {
        // Request that the ZEDSDK begin processing the spatial map for export.
        m_slCamera.requestSpatialMapAsync();

        // Start an async thread to wait for spatial map processing to finish. Return resultant future object.
        return std::async(std::launch::async,
                          [this]()
                          {
                              // Create instance variables.
                              sl::FusedPointCloud slSpatialMap;

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
                                  LOG_ERROR(g_qSharedLogger,
                                            "Failed to extract ZED spatial map. sl::ERROR_CODE is: {}",
                                            sl::toString(m_slCamera.getSpatialMapRequestStatusAsync()).get());

                                  // Return empty point cloud.
                                  return sl::FusedPointCloud();
                              }
                          });
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(g_qSharedLogger, "ZED spatial mapping was never enabled, can't extract spatial map!");
    }
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
bool ZEDCam::GetObjectTrackingEnabled()
{
    // Return is the object tracking feature of camera is enabled.
    return m_slCamera.isObjectDetectionEnabled();
}

/******************************************************************************
 * @brief Accessor for getting the tracked objects from the camera.
 *
 * @return std::vector<sl::ObjectData> - A vector containing the data for each object stored in an
 *                                  sl::ObjectData object.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-27
 ******************************************************************************/
std::vector<sl::ObjectData> ZEDCam::GetTrackedObjects()
{
    // Check if object detection has been enabled.
    if (m_slCamera.isObjectDetectionEnabled())
    {
        // Get updated image from camera.
        sl::ERROR_CODE slReturnCode = m_slCamera.retrieveObjects(m_slTrackedObjects);

        // Check if objects were successfully retrieved.
        if (slReturnCode == sl::ERROR_CODE::SUCCESS)
        {
            // Return the tracked object data.
            return m_slTrackedObjects.object_list;
        }
        else
        {
            // Submit logger message.
            LOG_WARNING(g_qSharedLogger, "Failed to retrieve ZED tracked objects. sl::ERROR_CODE is : {}", sl::toString(slReturnCode).get());

            // Return previously tracked object list.
            return m_slTrackedObjects.object_list;
        }
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(g_qSharedLogger, "ZED object tracking was never enabled!");

        // Return empty vector.
        return std::vector<sl::ObjectData>();
    }
}
