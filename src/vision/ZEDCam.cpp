/******************************************************************************
 * @brief Implements the ZedCam class.
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
 * @param nPropResolutionX - X res of camera.
 * @param nPropResolutionY - Y res of camera.
 * @param nPropFramesPerSecond - FPS camera is running at.
 * @param dPropHorizontalFOV - The horizontal field of view.
 * @param dPropVerticalFOV - The vertical field of view.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
ZedCam::ZedCam(const int nPropResolutionX, const int nPropResolutionY, const int nPropFramesPerSecond, const double dPropHorizontalFOV, const double dPropVerticalFOV) :
    Camera(nPropResolutionX, nPropResolutionY, nPropFramesPerSecond, PIXEL_FORMATS::eZED, dPropHorizontalFOV, dPropVerticalFOV)
{
    // Assign member variables.

    // Attempt to open camera.
}

/******************************************************************************
 * @brief Destroy the Zed Cam:: Zed Cam object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
ZedCam::~ZedCam()
{
    // Close the ZedCam.
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
sl::Mat ZedCam::GrabFrame(const bool bGrabRaw)
{
    // Grab regular image and store it in member variable.
}

/******************************************************************************
 * @brief Grabs a depth measure image from the camera. This image has the same shape as
 *      a grayscale image, but the values represent the depth in ZED_MEASURE_UNITS that is set in
 *      AutonomyConstants.h.
 *
 * @param bGrabRaw - Whether or not to apply class properties to image. (resize)
 *              If bGrabRaw is set to true, then the ZED_BASE_RESOLUTION that is set in AutonomyContants.h
 *              will be used.
 * @return sl::Mat - The result depth image stored in an sl::Mat object.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
sl::Mat ZedCam::GrabDepth(const bool bGrabRaw)
{
    // Grab the depth image and store it in member variable.
}

/******************************************************************************
 * @brief Grabs a point cloud image from the camera. This image has the same shape as a normal
 *      BGRA image put with three extra XYZ values attached to the 3rd dimension. (BGRAXYZ)
 *      The units and sign of the XYZ values are determined by ZED_MEASURE_UNITS and ZED_COORD_SYSTEM
 *      constants set in AutonomyConstants.h.
 *
 * @param bGrabRaw - Whether or not to apply class properties to image. (resize)
 *              If bGrabRaw is set to true, then the ZED_BASE_RESOLUTION that is set in AutonomyContants.h
 *              will be used.
 * @return sl::Mat - The result point cloud image with pixel colors and real-world locations.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
sl::Mat ZedCam::GrabPointCloud(const bool bGrabRaw)
{
    // Grab the point cloud and store it in member variable.
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
sl::ERROR_CODE ZedCam::ResetPositionalTracking()
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
sl::ERROR_CODE ZedCam::IngestCustomBoxObjects(std::vector<sl::CustomBoxObjectData> vCustomObjects)
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
sl::ERROR_CODE ZedCam::RebootCamera()
{
    // Reboot the camera and return the status code.
    return m_slCamera.reboot();
}
