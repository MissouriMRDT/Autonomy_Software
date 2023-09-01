/******************************************************************************
 * @brief Implements the CameraHandlerThread class.
 *
 * @file CameraHandlerThread.cpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "CameraHandlerThread.h"

/******************************************************************************
 * @brief Construct a new Camera Handler Thread:: Camera Handler Thread object.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
CameraHandlerThread::CameraHandlerThread()
{
    // Initialize main ZED camera.
    m_pMainCam = new ZEDCam(1920, 1080, 60, 110, 80, 0.2f, 40.0f, true);
}

/******************************************************************************
 * @brief Destroy the Camera Handler Thread:: Camera Handler Thread object.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
CameraHandlerThread::~CameraHandlerThread()
{
    // Delete dynamic memory.
    delete m_pMainCam;

    // Set dangling pointers to nullptr.
    m_pMainCam = nullptr;
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
ZEDCam* CameraHandlerThread::GetZED(ZEDCamName eCameraName)
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
BasicCam* CameraHandlerThread::GetBasicCam(BasicCamName eCameraName)
{
    // Determine which camera should be returned.
    switch (eCameraName)
    {
        case eHeadLeftArucoEye: break;     // No camera to return yet.
        case eHeadRightAcuroEye: break;    // No camera to return yet.
        default: break;
    }
}
