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
                            constants::ZED_MAINCAM_ENABLE_RECORDING,
                            constants::ZED_DEFAULT_MINIMUM_DISTANCE,
                            constants::ZED_DEFAULT_MAXIMUM_DISTANCE,
                            constants::ZED_MAINCAM_USE_GPU_MAT,
                            constants::ZED_MAINCAM_USE_HALF_PRECISION_DEPTH,
                            constants::ZED_MAINCAM_FUSION_MASTER,
                            constants::ZED_MAINCAM_FRAME_RETRIEVAL_THREADS,
                            constants::ZED_MAINCAM_SERIAL);

    // Initialize left ZED camera.
    m_pLeftCam = new ZEDCam(constants::ZED_LEFTCAM_RESOLUTIONX,
                            constants::ZED_LEFTCAM_RESOLUTIONY,
                            constants::ZED_LEFTCAM_FPS,
                            constants::ZED_LEFTCAM_HORIZONTAL_FOV,
                            constants::ZED_LEFTCAM_VERTICAL_FOV,
                            constants::ZED_LEFTCAM_ENABLE_RECORDING,
                            constants::ZED_DEFAULT_MINIMUM_DISTANCE,
                            constants::ZED_DEFAULT_MAXIMUM_DISTANCE,
                            constants::ZED_LEFTCAM_USE_GPU_MAT,
                            constants::ZED_LEFTCAM_USE_HALF_PRECISION_DEPTH,
                            constants::ZED_LEFTCAM_FUSION_MASTER,
                            constants::ZED_LEFTCAM_FRAME_RETRIEVAL_THREADS,
                            constants::ZED_LEFTCAM_SERIAL);

    // Initialize right ZED camera.
    m_pRightCam = new ZEDCam(constants::ZED_RIGHTCAM_RESOLUTIONX,
                             constants::ZED_RIGHTCAM_RESOLUTIONY,
                             constants::ZED_RIGHTCAM_FPS,
                             constants::ZED_RIGHTCAM_HORIZONTAL_FOV,
                             constants::ZED_RIGHTCAM_VERTICAL_FOV,
                             constants::ZED_RIGHTCAM_ENABLE_RECORDING,
                             constants::ZED_DEFAULT_MINIMUM_DISTANCE,
                             constants::ZED_DEFAULT_MAXIMUM_DISTANCE,
                             constants::ZED_RIGHTCAM_USE_GPU_MAT,
                             constants::ZED_RIGHTCAM_USE_HALF_PRECISION_DEPTH,
                             constants::ZED_RIGHTCAM_FUSION_MASTER,
                             constants::ZED_RIGHTCAM_FRAME_RETRIEVAL_THREADS,
                             constants::ZED_RIGHTCAM_SERIAL);

    // Initialize ground eye.
    m_pGroundCam = new BasicCam(constants::BASICCAM_GROUNDCAM_INDEX,
                                constants::BASICCAM_GROUNDCAM_RESOLUTIONX,
                                constants::BASICCAM_GROUNDCAM_RESOLUTIONY,
                                constants::BASICCAM_GROUNDCAM_FPS,
                                constants::BASICCAM_GROUNDCAM_PIXELTYPE,
                                constants::BASICCAM_GROUNDCAM_HORIZONTAL_FOV,
                                constants::BASICCAM_GROUNDCAM_VERTICAL_FOV,
                                constants::BASICCAM_GROUNDCAM_ENABLE_RECORDING,
                                constants::BASICCAM_GROUNDCAM_FRAME_RETRIEVAL_THREADS);

    // Initialize recording handler for cameras.
    m_pRecordingHandler = new RecordingHandler(RecordingHandler::RecordingMode::eCameraHandler);
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

    // Delete dynamic memory.
    delete m_pMainCam;
    delete m_pLeftCam;
    delete m_pRightCam;
    delete m_pGroundCam;
    delete m_pRecordingHandler;

    // Set dangling pointers to nullptr.
    m_pMainCam          = nullptr;
    m_pLeftCam          = nullptr;
    m_pRightCam         = nullptr;
    m_pGroundCam        = nullptr;
    m_pRecordingHandler = nullptr;
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
    m_pLeftCam->Start();
    m_pRightCam->Start();

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
    m_pLeftCam->RequestStop();
    m_pRightCam->RequestStop();
    m_pMainCam->Join();
    m_pLeftCam->Join();
    m_pRightCam->Join();

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
        case eHeadMainCam: return m_pMainCam;       // Return the ZEDCam in the autonomy head.
        case eFrameLeftCam: return m_pLeftCam;      // Return the ZEDCam on the left side of the rover frame.
        case eFrameRightCam: return m_pRightCam;    // Return the ZEDCam on the right side of the rover frame.
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
        case eHeadGroundCam: return m_pGroundCam;    // Return the ground fisheye cam in the autonomy head.
        default: return m_pGroundCam;
    }
}
