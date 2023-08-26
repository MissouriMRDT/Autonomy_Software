/******************************************************************************
 * @brief Implements the BasicCam class.
 *
 * @file BasicCam.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-19
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "BasicCam.h"
#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Construct a new Basic Cam:: Basic Cam object.
 *
 * @param szCameraPath - The file path to the camera hardware.
 * @param nPropResolutionX - X res of camera.
 * @param nPropResolutionY - Y res of camera.
 * @param nPropFramesPerSecond - FPS camera is running at.
 * @param ePropPixelFormat - The pixel layout/format of the image.
 * @param dPropHorizontalFOV - The horizontal field of view.
 * @param dPropVerticalFOV - The vertical field of view.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
BasicCam::BasicCam(const std::string szCameraPath,
                   const int nPropResolutionX,
                   const int nPropResolutionY,
                   const int nPropFramesPerSecond,
                   const PIXEL_FORMATS ePropPixelFormat,
                   const double dPropHorizontalFOV,
                   const double dPropVerticalFOV) :
    Camera(nPropResolutionX, nPropResolutionY, nPropFramesPerSecond, ePropPixelFormat, dPropHorizontalFOV, dPropVerticalFOV)
{
    // Assign member variables.
    m_szCameraPath = szCameraPath;
    m_nCameraIndex = -1;

    // Set flag specifying that the camera is located at a dev/video index.
    m_bCameraIsConnectedOnVideoIndex = false;

    // Attempt to open camera with OpenCV's VideoCapture.
    m_cvCamera.open(szCameraPath);
    // Check if the camera was successfully opened.
    if (m_cvCamera.isOpened())
    {
        // Submit logger message.
        LOG_DEBUG(g_qSharedLogger, "Camera {} at path/URL {} has been successfully opened.", m_cvCamera.getBackendName(), m_szCameraPath);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(g_qSharedLogger, "Unable to open camera at path/URL {}", m_szCameraPath);
    }
}

/******************************************************************************
 * @brief Construct a new Basic Cam:: Basic Cam object. Overloaded for dev/video
 *      indexes.
 *
 * @param nCameraIndex - The video index that the camera is connected on.
 * @param nPropResolutionX - X res of camera.
 * @param nPropResolutionY - Y res of camera.
 * @param nPropFramesPerSecond - FPS camera is running at.
 * @param ePropPixelFormat - The pixel layout/format of the image.
 * @param dPropHorizontalFOV - The horizontal field of view.
 * @param dPropVerticalFOV - The vertical field of view.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
BasicCam::BasicCam(const int nCameraIndex,
                   const int nPropResolutionX,
                   const int nPropResolutionY,
                   const int nPropFramesPerSecond,
                   const PIXEL_FORMATS ePropPixelFormat,
                   const double dPropHorizontalFOV,
                   const double dPropVerticalFOV) :
    Camera(nPropResolutionX, nPropResolutionY, nPropFramesPerSecond, ePropPixelFormat, dPropHorizontalFOV, dPropVerticalFOV)
{
    // Assign member variables.
    m_nCameraIndex = nCameraIndex;
    m_szCameraPath = "";

    // Set flag specifying that the camera is located at a dev/video index.
    m_bCameraIsConnectedOnVideoIndex = true;

    // Attempt to open camera with OpenCV's VideoCapture.
    m_cvCamera.open(m_nCameraIndex);
    // Check if the camera was successfully opened.
    if (m_cvCamera.isOpened())
    {
        // Submit logger message.
        LOG_DEBUG(g_qSharedLogger, "Camera {} at video index {} has been successfully opened.", m_cvCamera.getBackendName(), m_nCameraIndex);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(g_qSharedLogger, "Unable to open camera at video index {}", m_nCameraIndex);
    }
}

/******************************************************************************
 * @brief Destroy the Basic Cam:: Basic Cam object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
BasicCam::~BasicCam()
{
    // Release camera capture object.
    m_cvCamera.release();
}

/******************************************************************************
 * @brief Retrieves a frame from the camera.
 *
 * @return cv::Mat - The returned image stored in an OpenCV MAT.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
cv::Mat& BasicCam::GrabFrame(const bool bGrabRaw)
{
    // Check if the camera is open.
    if (m_cvCamera.isOpened())
    {
        // Get frame from camera and store.
        m_cvCamera >> m_cvFrame;

        // Check if the retrieved image is empty.
        if (!m_cvFrame.empty())
        {
            // Check if we should resize to match member variables properties.
            if (!bGrabRaw)
            {
                // Resize frame.
                cv::resize(m_cvFrame, m_cvFrame, cv::Size(m_nPropResolutionX, m_nPropResolutionY));
            }
        }
        else
        {
            // Submit logger message.
            LOG_WARNING(g_qSharedLogger, "Image retrieved from camera source is empty!");
        }
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(g_qSharedLogger, "Tried to get frame from camera but it's not opened!");
    }

    // Call FPS tracker tick.
    m_pIPS->Tick();

    // Return retrieved image.
    return m_cvFrame;
}

/******************************************************************************
 * @brief Accessor for the camera open status.
 *
 * @return true - The camera has been successfully opened.
 * @return false - The camera has not been successfully opened.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
bool BasicCam::GetCameraIsOpen()
{
    // Get camera status from OpenCV.
    return m_cvCamera.isOpened();
}

/******************************************************************************
 * @brief Accessor for the cameras path or video index.
 *
 * @tparam T - Either an int or string depending on if the camera is located at a
 *          hardware path or a dev/video index.
 * @return T - The path or index of the camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
template<typename T>
T BasicCam::GetCameraLocation()
{
    // Check if camera location is a hardware path or video index.
    if (m_bCameraIsConnectedOnVideoIndex)
    {
        // If video index, return index integer.
        return m_nCameraIndex;
    }
    else
    {
        // If video path, return path string.
        return m_szCameraPath;
    }
}