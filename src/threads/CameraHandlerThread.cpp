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
#include "../AutonomyConstants.h"

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
    m_pMainCam = new ZEDCam(constants::ZED_MAINCAM_RESOLUTIONX,
                            constants::ZED_MAINCAM_RESOLUTIONY,
                            constants::ZED_MAINCAM_FPS,
                            constants::ZED_MAINCAM_HORIZONTAL_FOV,
                            constants::ZED_MAINCAM_VERTICAL_FOV,
                            constants::ZED_DEFAULT_MINIMUM_DISTANCE,
                            constants::ZED_DEFAULT_MAXIMUM_DISTANCE,
                            constants::ZED_MAINCAM_USE_GPU_MAT,
                            constants::ZED_MAINCAM_USE_HALF_PRECISION_DEPTH,
                            constants::ZED_MAINCAM_SERIAL);

    // Initialize main ZED camera 2.
    m_pMainCam2 = new ZEDCam(constants::ZEDTEST_MAINCAM_RESOLUTIONX,
                             constants::ZEDTEST_MAINCAM_RESOLUTIONY,
                             constants::ZEDTEST_MAINCAM_FPS,
                             constants::ZEDTEST_MAINCAM_HORIZONTAL_FOV,
                             constants::ZEDTEST_MAINCAM_VERTICAL_FOV,
                             constants::ZED_DEFAULT_MINIMUM_DISTANCE,
                             constants::ZED_DEFAULT_MAXIMUM_DISTANCE,
                             constants::ZEDTEST_MAINCAM_USE_GPU_MAT,
                             constants::ZEDTEST_MAINCAM_USE_HALF_PRECISION_DEPTH,
                             constants::ZEDTEST_MAINCAM_SERIAL);

    // Initialize Left acruco eye.
    // m_pLeftCam = new BasicCam(constants::BASIC_LEFTCAM_INDEX,
    //                           constants::BASIC_LEFTCAM_RESOLUTIONX,
    //                           constants::BASIC_LEFTCAM_RESOLUTIONY,
    //                           constants::BASIC_LEFTCAM_FPS,
    //                           constants::BASIC_LEFTCAM_PIXELTYPE,
    //                           constants::BASIC_LEFTCAM_HORIZONTAL_FOV,
    //                           constants::BASIC_LEFTCAM_VERTICAL_FOV);
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
    // Signal and wait for cameras to stop.
    m_pMainCam->RequestStop();
    m_pMainCam2->RequestStop();
    m_pLeftCam->RequestStop();
    m_pMainCam->Join();
    m_pMainCam2->Join();
    m_pLeftCam->Join();

    // Delete dynamic memory.
    delete m_pMainCam;
    delete m_pMainCam2;
    delete m_pLeftCam;

    // Set dangling pointers to nullptr.
    m_pMainCam  = nullptr;
    m_pMainCam2 = nullptr;
    m_pLeftCam  = nullptr;
}

/******************************************************************************
 * @brief Signals all cameras to start their threads.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-09
 ******************************************************************************/
void CameraHandlerThread::StartAllCameras()
{
    // Start ZED cams.
    m_pMainCam->Start();
    m_pMainCam2->Start();

    // Start basic cams.
    // m_pLeftCam->Start();
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
        case eHeadMainCam2: return m_pMainCam2;
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
        case eHeadLeftArucoEye: return m_pLeftCam;     // Return the left fisheye cam in the autonomy head.
        case eHeadRightAcuroEye: return m_pLeftCam;    // No camera to return yet.
        default: return m_pLeftCam;
    }
}
