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
 * @author ClayJay3 (claytonraycowen@gmail.com), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
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
                                              constants::ZED_MAINCAM_FRAME_RETRIEVAL_THREADS,
                                              constants::ZED_MAINCAM_SERIAL);

        // Additional setup for main ZED camera.
        if (constants::ZED_MAINCAM_EXPORT_SPATIAL_MAP)
        {
            m_pMainCam->EnableSpatialMapping();
        }

        // Initialize rear ZED camera.
        m_pRearCam = std::make_shared<ZEDCam>(constants::ZED_REARCAM_RESOLUTIONX,
                                              constants::ZED_REARCAM_RESOLUTIONY,
                                              constants::ZED_REARCAM_FPS,
                                              constants::ZED_REARCAM_HORIZONTAL_FOV,
                                              constants::ZED_REARCAM_VERTICAL_FOV,
                                              constants::ZED_REARCAM_ENABLE_RECORDING,
                                              constants::ZED_REARCAM_EXPORT_SVO_RECORDING,
                                              constants::ZED_DEFAULT_MINIMUM_DISTANCE,
                                              constants::ZED_DEFAULT_MAXIMUM_DISTANCE,
                                              constants::ZED_REARCAM_USE_GPU_MAT,
                                              constants::ZED_REARCAM_USE_HALF_PRECISION_DEPTH,
                                              constants::ZED_REARCAM_FRAME_RETRIEVAL_THREADS,
                                              constants::ZED_REARCAM_SERIAL);

        // Additional setup for rear ZED camera.
        if (constants::ZED_REARCAM_EXPORT_SPATIAL_MAP)
        {
            m_pRearCam->EnableSpatialMapping();
        }
    }
    else
    {
        m_pMainCam =
            std::make_shared<SIMZEDCam>("ws://" + constants::SIM_IP_ADDRESS + ":" + std::to_string(constants::SIM_WEBSOCKET_PORT) + "/" + constants::SIM_MAINCAM_NAME,
                                        constants::ZED_MAINCAM_RESOLUTIONX,
                                        constants::ZED_MAINCAM_RESOLUTIONY,
                                        constants::ZED_MAINCAM_FPS,
                                        constants::ZED_MAINCAM_HORIZONTAL_FOV,
                                        constants::ZED_MAINCAM_VERTICAL_FOV,
                                        constants::ZED_MAINCAM_ENABLE_RECORDING,
                                        constants::ZED_MAINCAM_FRAME_RETRIEVAL_THREADS,
                                        constants::ZED_MAINCAM_SERIAL);

        // Set the position offsets of the main camera.
        m_pMainCam->SetCameraPoseOffset(constants::ZED_MAINCAM_EASTING_OFFSET,
                                        constants::ZED_MAINCAM_NORTHING_OFFSET,
                                        constants::ZED_MAINCAM_ALTITUDE_OFFSET,
                                        constants::ZED_MAINCAM_QUATERNION_OFFSET_X,
                                        constants::ZED_MAINCAM_QUATERNION_OFFSET_Y,
                                        constants::ZED_MAINCAM_QUATERNION_OFFSET_Z,
                                        constants::ZED_MAINCAM_QUATERNION_OFFSET_W);

        m_pRearCam =
            std::make_shared<SIMZEDCam>("ws://" + constants::SIM_IP_ADDRESS + ":" + std::to_string(constants::SIM_WEBSOCKET_PORT) + "/" + constants::SIM_REARCAM_NAME,
                                        constants::ZED_REARCAM_RESOLUTIONX,
                                        constants::ZED_REARCAM_RESOLUTIONY,
                                        constants::ZED_REARCAM_FPS,
                                        constants::ZED_REARCAM_HORIZONTAL_FOV,
                                        constants::ZED_REARCAM_VERTICAL_FOV,
                                        constants::ZED_REARCAM_ENABLE_RECORDING,
                                        constants::ZED_REARCAM_FRAME_RETRIEVAL_THREADS,
                                        constants::ZED_REARCAM_SERIAL);

        // Set the position offsets of the rear camera.
        m_pRearCam->SetCameraPoseOffset(constants::ZED_REARCAM_EASTING_OFFSET,
                                        constants::ZED_REARCAM_NORTHING_OFFSET,
                                        constants::ZED_REARCAM_ALTITUDE_OFFSET,
                                        constants::ZED_REARCAM_QUATERNION_OFFSET_X,
                                        constants::ZED_REARCAM_QUATERNION_OFFSET_Y,
                                        constants::ZED_REARCAM_QUATERNION_OFFSET_Z,
                                        constants::ZED_REARCAM_QUATERNION_OFFSET_W);
    }

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
 * @author ClayJay3 (claytonraycowen@gmail.com), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2023-09-09
 ******************************************************************************/
void CameraHandler::StartAllCameras()
{
    // Start ZED cams.
    m_pMainCam->Start();
    m_pRearCam->Start();
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
 * @author ClayJay3 (claytonraycowen@gmail.com), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2023-10-03
 ******************************************************************************/
void CameraHandler::StopAllCameras()
{
    // Stop recording handler.
    m_pRecordingHandler->RequestStop();
    m_pRecordingHandler->Join();

    // Stop main ZED cam.
    m_pMainCam->RequestStop();
    m_pMainCam->Join();

    // Stop rear ZED cam.
    m_pRearCam->RequestStop();
    m_pRearCam->Join();

    // Stop basic cam.
    // m_pBasicCam->RequestStop();
    // m_pBasicCam->Join();
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
 * @author clayjay3 (claytonraycowen@gmail.com), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2023-09-01
 ******************************************************************************/
std::shared_ptr<ZEDCamera> CameraHandler::GetZED(ZEDCamName eCameraName)
{
    // Determine which camera should be returned.
    switch (eCameraName)
    {
        case ZEDCamName::eHeadMainCam: return m_pMainCam; break;    // Return the main ZEDCam in the autonomy head.
        case ZEDCamName::eRearCam: return m_pRearCam; break;        // Return the rear ZedCam.
        default: return m_pMainCam; break;
    }
}

/******************************************************************************
 * @brief Accessor for Basic cameras.
 *
 * @param eCameraName - The name of the camera to retrieve. An enum defined in and specific to this class.
 * @return std::shared_ptr<BasicCamera> - A pointer to the basic camera pertaining to the given name.
 *
 * @author clayjay3 (claytonraycowen@gmail.com), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2023-09-01
 ******************************************************************************/
std::shared_ptr<BasicCamera> CameraHandler::GetBasicCam(BasicCamName eCameraName)
{
    // Determine which camera should be returned.
    switch (eCameraName)
    {
        default: return nullptr; break;
    }
}
