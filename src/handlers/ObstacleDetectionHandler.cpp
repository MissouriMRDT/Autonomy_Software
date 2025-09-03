/******************************************************************************
 * @brief Implements the ObstacleDetectionHandler class.
 *
 * @file ObstacleDetectionHandler.cpp
 * @author UhOhDonovan (donovan@balehaus.org)
 * @date 2025-09-03
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "ObstacleDetectionHandler.h"
#include "../AutonomyConstants.h"
#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Construct a new ObstacleDetectionHandler::ObstacleDetectionHandler object.
 *
 *
 * @author UhOhDonodonovan@balehaus.org)
 * @date 2025-09-03
 ******************************************************************************/
ObstacleDetectionHandler::ObstacleDetectionHandler()
{
    // Initialize detector for main ZEDCam.
    m_pObstacleDetectorMainCam = std::make_shared<ObstacleDetector>(globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam),
                                                                    constants::OBSTACLEDETECT_MAINCAM_MAX_FPS,
                                                                    constants::OBSTACLEDETECT_MAINCAM_ENABLE_RECORDING,
                                                                    constants::OBSTACLEDETECT_MAINCAM_DATA_RETRIEVAL_THREADS,
                                                                    constants::ZED_MAINCAM_USE_GPU_MAT);

    // Attempt to init torch detection.
    if (m_pObstacleDetectorMainCam->InitTorchDetection(constants::OBSTACLEDETECT_MAINCAM_TORCH_MODEL))
    {
        // Set torch detection enabled.
        m_pObstacleDetectorMainCam->EnableTorchDetection(constants::OBSTACLEDETECT_MAINCAM_TORCH_CONFIDENCE, constants::OBSTACLEDETECT_MAINCAM_TORCH_NMS_THRESH)
    }

    // Initialize recording handler for detectors.
    m_pRecordingHandler = std::make_unique<RecordingHandler>(RecordingHandler::RecordingMode::eObstacleDetectionHandler);
}

/******************************************************************************
 * @brief Destroy the ObstacleDetectionHandler::ObstacleDetectionHandler obstacle.
 *
 *
 * @author UhOhDonovan (donovan@balehaus.org)
 * @date 2025-09-03
 ******************************************************************************/
ObstacleDetectionHandler::~ObstacleDetectionHandler()
{
    // Signal and wait for cameras to stop.
    this->StopAllDetectors();
}

/******************************************************************************
 * @brief Signals all detectors to start their threads.
 *
 *
 * @author UhOhDonovan (donovan@balehaus.org)
 * @date 2025-09-03
 ******************************************************************************/
void ObstacleDetectionHandler::StartAllDetectors()
{
    // Start ZED maincam detector.
    m_pObstacleDetectorMainCam->Start();
}

/******************************************************************************
 * @brief Signal the RecordingHandler to start recording feeds from the detectors.
 *
 *
 * @author UhOhDonovan (donovan@balehaus.org)
 * @date 2025-05-05
 ******************************************************************************/
void ObstacleDetectionHandler::StartRecording()
{
    // Start recording for all detectors.
    m_pRecordingHandler->Start();
}

/******************************************************************************
 * @brief Signals all detectors to stop their threads.
 *
 *
 * @author UhOhDonovan (donovan@balehaus.org)
 * @date 2025-09-03
 ******************************************************************************/
void ObstacleDetectionHandler::StopAllDetectors()
{
    // Stop recording handler.
    m_pRecordingHandler->RequestStop();
    m_pRecordingHandler->Join();

    // Stop ZED maincam detector.
    m_pObstacleDetectorMainCam->RequestStop();
    m_pObstacleDetectorMainCam->Join();
}

/******************************************************************************
 * @brief  Signal the RecordingHandler to stop recording feeds from the detectors.
 *
 *
 * @author UhOhDonovan (donovan@balehaus.org)
 * @date 2025-09-03
 ******************************************************************************/
void ObstacleDetectionHandler::StopRecording()
{
    // Stop recording handler.
    m_pRecordingHandler->RequestStop();
    m_pRecordingHandler->Join();
}

/******************************************************************************
 * @brief Accessor for ObstacleDetector detectors.
 *
 * @param eDetectorName - The name of the detector to retrieve. An enum defined in and specific to this class.
 * @return std::shared_ptr<ObstacleDetector> - A pointer to the detector pertaining to the given name.
 *
 * @author UhOhDonovan (donovan@balehaus.org)
 * @date 2025-09-03
 ******************************************************************************/
std::shared_ptr<ObstacleDetector> ObstacleDetectionHandler::GetObstacleDetector(ObstacleDetectors eDetectorName)
{
    // Determine which obstacle detector should be returned.
    switch (eDetectorName)
    {
        case ObstacleDetectors::eHeadMainCam: return m_pObstacleDetectorMainCam; break;
        default: return m_pObstacleDetectorMainCam; break;
    }
}
