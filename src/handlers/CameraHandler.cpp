/******************************************************************************
 * @brief Implements the CameraHandler class.
 *
 * @file CameraHandler.cpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "CameraHandler.h"
#include "../AutonomyConstants.h"

/******************************************************************************
 * @brief Construct a new Camera Handler Thread:: Camera Handler Thread object.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
CameraHandler::CameraHandler()
{
    // Initialize main ZED camera.
    m_pMainCam = new ZEDCam(constants::ZED_MAINCAM_RESOLUTIONX,
                            constants::ZED_MAINCAM_RESOLUTIONY,
                            constants::ZED_MAINCAM_FPS,
                            constants::ZED_MAINCAM_HORIZONTAL_FOV,
                            constants::ZED_MAINCAM_VERTICAL_FOV,
                            constants::ZED_DEFAULT_MINIMUM_DISTANCE,
                            constants::ZED_DEFAULT_MAXIMUM_DISTANCE,
                            constants::ZED_MAINCAM_USE_GPU_MAT,
                            constants::ZED_MAINCAM_USE_HALF_PRECISION_DEPTH,
                            constants::ZED_MAINCAM_FRAME_RETRIEVAL_THREADS,
                            constants::ZED_MAINCAN_SERIAL);

    // Initialize Left acruco eye.
    m_pLeftCam = new BasicCam(constants::BASICCAM_LEFTCAM_INDEX,
                              constants::BASICCAM_LEFTCAM_RESOLUTIONX,
                              constants::BASICCAM_LEFTCAM_RESOLUTIONY,
                              constants::BASICCAM_LEFTCAM_FPS,
                              constants::BASICCAM_LEFTCAM_PIXELTYPE,
                              constants::BASICCAM_LEFTCAM_HORIZONTAL_FOV,
                              constants::BASICCAM_LEFTCAM_VERTICAL_FOV,
                              constants::BASICCAM_LEFTCAM_FRAME_RETRIEVAL_THREADS);
}

/******************************************************************************
 * @brief Destroy the Camera Handler Thread:: Camera Handler Thread object.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
CameraHandler::~CameraHandler()
{
    // Signal and wait for cameras to stop.
    m_pMainCam->RequestStop();
    m_pLeftCam->RequestStop();
    m_pMainCam->Join();
    m_pLeftCam->Join();

    // Delete dynamic memory.
    delete m_pMainCam;
    delete m_pLeftCam;

    // Set dangling pointers to nullptr.
    m_pMainCam = nullptr;
    m_pLeftCam = nullptr;
}

/******************************************************************************
 * @brief Signals all cameras to start their threads.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-09
 ******************************************************************************/
void CameraHandler::StartAllCameras()
{
    // Start ZED cams.
    m_pMainCam->Start();

    // Start basic cams.
    m_pLeftCam->Start();
}

/******************************************************************************
 * @brief Signals all cameras to stop their threads.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-03
 ******************************************************************************/
void CameraHandler::StopAllCameras()
{
    // Stop ZED cams.
    m_pMainCam->RequestStop();
    m_pMainCam->Join();

    // Stop basic cams.
    m_pLeftCam->RequestStop();
    m_pLeftCam->Join();
}

/******************************************************************************
 * @brief Accessor for ZED cameras.
 *
 * @param eCameraName - The name of the camera to retrieve. An enum defined in and specific to this class.
 * @return ZEDCam* - A pointer to the zed camera pertaining to the given name.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-01
 ******************************************************************************/
ZEDCam* CameraHandler::GetZED(ZEDCamName eCameraName)
{
    // Determine which camera should be returned.
    switch (eCameraName)
    {
        case eHeadMainCam: return m_pMainCam;
        default: return m_pMainCam;
    }
}

/******************************************************************************
 * @brief Accessor for Basic cameras.
 *
 * @param eCameraName - The name of the camera to retrieve. An enum defined in and specific to this class.
 * @return BasicCam* - A pointer to the basic camera pertaining to the given name.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-01
 ******************************************************************************/
BasicCam* CameraHandler::GetBasicCam(BasicCamName eCameraName)
{
    // Determine which camera should be returned.
    switch (eCameraName)
    {
        case eHeadLeftArucoEye: return m_pLeftCam;     // Return the left fisheye cam in the autonomy head.
        case eHeadRightAcuroEye: return m_pLeftCam;    // No camera to return yet.
        default: return m_pLeftCam;
    }
}
