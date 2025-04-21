/******************************************************************************
 * @brief Implements the CameraHandler class.
 *
 * @file CameraHandler.cpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "CameraHandler.h"
#include "../AutonomyConstants.h"
#include "../vision/cameras/BasicCam.h"
#include "../vision/cameras/ZEDCam.h"
#include "../vision/cameras/sim/SIMBasicCam.h"
#include "../vision/cameras/sim/SIMZEDCam.h"

/******************************************************************************
 * @brief Construct a new Camera Handler Thread:: Camera Handler Thread object.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
CameraHandler::CameraHandler()
{
    // Check if we are in simulation mode.
    if (!constants::MODE_SIM)
    {
        // Initialize main ZED camera.
        m_pMainCam = std::make_shared<ZEDCam>(constants::ZED_MAINCAM_RESOLUTIONX,
                                              constants::ZED_MAINCAM_RESOLUTIONY,
                                              constants::ZED_MAINCAM_FPS,
                                              constants::ZED_MAINCAM_HORIZONTAL_FOV,
                                              constants::ZED_MAINCAM_VERTICAL_FOV,
                                              constants::ZED_MAINCAM_ENABLE_RECORDING,
                                              constants::ZED_MAINCAM_EXPORT_SVO_RECORDING,
                                              constants::ZED_DEFAULT_MINIMUM_DISTANCE,
                                              constants::ZED_DEFAULT_MAXIMUM_DISTANCE,
                                              constants::ZED_MAINCAM_USE_GPU_MAT,
                                              constants::ZED_MAINCAM_USE_HALF_PRECISION_DEPTH,
                                              constants::ZED_MAINCAM_FUSION_MASTER,
                                              constants::ZED_MAINCAM_FRAME_RETRIEVAL_THREADS,
                                              constants::ZED_MAINCAM_SERIAL);

        // Always enable positional tracking.
        m_pMainCam->EnablePositionalTracking();

        // Additional setup for main ZED camera.
        if (constants::ZED_MAINCAM_EXPORT_SPATIAL_MAP)
        {
            m_pMainCam->EnableSpatialMapping();
        }
        if (constants::ZED_MAINCAM_EXPORT_SVO_RECORDING)
        {
            m_pMainCam->EnableSpatialMapping();
        }
    }
    else
    {
        m_pMainCam = std::make_shared<SIMZEDCam>("ws://" + constants::SIM_IP_ADDRESS + ":" + std::to_string(constants::SIM_WEBSOCKET_PORT),
                                                 constants::ZED_MAINCAM_RESOLUTIONX,
                                                 constants::ZED_MAINCAM_RESOLUTIONY,
                                                 constants::ZED_MAINCAM_FPS,
                                                 constants::ZED_MAINCAM_HORIZONTAL_FOV,
                                                 constants::ZED_MAINCAM_VERTICAL_FOV,
                                                 constants::ZED_MAINCAM_ENABLE_RECORDING,
                                                 constants::ZED_MAINCAM_FRAME_RETRIEVAL_THREADS,
                                                 constants::ZED_MAINCAM_SERIAL);
    }

    // Initialize ground eye.
    m_pGroundCam = std::make_shared<BasicCam>(constants::BASICCAM_GROUNDCAM_INDEX,
                                              constants::BASICCAM_GROUNDCAM_RESOLUTIONX,
                                              constants::BASICCAM_GROUNDCAM_RESOLUTIONY,
                                              constants::BASICCAM_GROUNDCAM_FPS,
                                              constants::BASICCAM_GROUNDCAM_PIXELTYPE,
                                              constants::BASICCAM_GROUNDCAM_HORIZONTAL_FOV,
                                              constants::BASICCAM_GROUNDCAM_VERTICAL_FOV,
                                              constants::BASICCAM_GROUNDCAM_ENABLE_RECORDING,
                                              constants::BASICCAM_GROUNDCAM_FRAME_RETRIEVAL_THREADS);

    // Initialize recording handler for cameras.
    m_pRecordingHandler = std::make_unique<RecordingHandler>(RecordingHandler::RecordingMode::eCameraHandler);
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
    this->StopAllCameras();
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
    m_pGroundCam->Start();
}

/******************************************************************************
 * @brief Signal the RecordingHandler to start recording video feeds from the CameraHandler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-31
 ******************************************************************************/
void CameraHandler::StartRecording()
{
    // Start recording handler.
    m_pRecordingHandler->Start();
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
    // Stop recording handler.
    m_pRecordingHandler->RequestStop();
    m_pRecordingHandler->Join();

    // Stop ZED cams.
    m_pMainCam->RequestStop();
    m_pMainCam->Join();

    // Stop basic cams.
    m_pGroundCam->RequestStop();
    m_pGroundCam->Join();
}

/******************************************************************************
 * @brief Signal the RecordingHandler to stop recording video feeds from the CameraHandler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-01
 ******************************************************************************/
void CameraHandler::StopRecording()
{
    // Stop recording handler.
    m_pRecordingHandler->RequestStop();
    m_pRecordingHandler->Join();
}

/******************************************************************************
 * @brief Accessor for ZED cameras.
 *
 * @param eCameraName - The name of the camera to retrieve. An enum defined in and specific to this class.
 * @return std::shared_ptr<ZEDCamera> - A pointer to the zed camera pertaining to the given name.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-01
 ******************************************************************************/
std::shared_ptr<ZEDCamera> CameraHandler::GetZED(ZEDCamName eCameraName)
{
    // Determine which camera should be returned.
    switch (eCameraName)
    {
        case ZEDCamName::eHeadMainCam: return m_pMainCam; break;    // Return the ZEDCam in the autonomy head.
        default: return m_pMainCam; break;
    }
}

/******************************************************************************
 * @brief Accessor for Basic cameras.
 *
 * @param eCameraName - The name of the camera to retrieve. An enum defined in and specific to this class.
 * @return std::shared_ptr<BasicCamera> - A pointer to the basic camera pertaining to the given name.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-01
 ******************************************************************************/
std::shared_ptr<BasicCamera> CameraHandler::GetBasicCam(BasicCamName eCameraName)
{
    // Determine which camera should be returned.
    switch (eCameraName)
    {
        case BasicCamName::eHeadGroundCam: return m_pGroundCam; break;    // Return the ground fisheye cam in the autonomy head.
        default: return m_pGroundCam; break;
    }
}
