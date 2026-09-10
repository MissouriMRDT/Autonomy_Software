/******************************************************************************
 * @brief Implements the TagDetectionHandler class.
 *
 * @file TagDetectionHandler.cpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "TagDetectionHandler.h"
#include "../AutonomyConstants.h"
#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Construct a new TagDetectionHandler::TagDetectionHandler object.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
TagDetectionHandler::TagDetectionHandler()
{
    // Initialize detector for main ZEDCam.
    m_pTagDetectorMainCam = std::make_shared<TagDetector>(globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam),
                                                          constants::TAGDETECT_MAINCAM_CORNER_REFINE_MAX_ITER,
                                                          constants::TAGDETECT_MAINCAM_CORNER_REFINE_METHOD,
                                                          constants::TAGDETECT_MAINCAM_MARKER_BORDER_BITS,
                                                          constants::TAGDETECT_MAINCAM_DETECT_INVERTED_MARKER,
                                                          constants::TAGDETECT_MAINCAM_USE_ARUCO3_DETECTION,
                                                          constants::TAGDETECT_MAINCAM_ENABLE_TRACKING,
                                                          constants::TAGDETECT_MAINCAM_MAX_FPS,
                                                          constants::TAGDETECT_MAINCAM_ENABLE_RECORDING,
                                                          constants::TAGDETECT_MAINCAM_DATA_RETRIEVAL_THREADS,
                                                          constants::ZED_MAINCAM_USE_GPU_MAT);

    // Check if torch detection is enabled for main ZEDCam.
    if (constants::TAGDETECT_MAINCAM_ENABLE_TORCH)
    {
        // Attempt to init torch detection.
        if (m_pTagDetectorMainCam->InitTorchDetection(constants::TAGDETECT_TORCH_MODEL))
        {
            // Set torch detection enabled.
            m_pTagDetectorMainCam->EnableTorchDetection(constants::TAGDETECT_MAINCAM_TORCH_CONFIDENCE, constants::TAGDETECT_MAINCAM_TORCH_NMS_THRESH);
        }
    }

    // Initialize detector for rear ZEDCam.
    m_pTagDetectorRearCam = std::make_shared<TagDetector>(globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eRearCam),
                                                          constants::TAGDETECT_REARCAM_CORNER_REFINE_MAX_ITER,
                                                          constants::TAGDETECT_REARCAM_CORNER_REFINE_METHOD,
                                                          constants::TAGDETECT_REARCAM_MARKER_BORDER_BITS,
                                                          constants::TAGDETECT_REARCAM_DETECT_INVERTED_MARKER,
                                                          constants::TAGDETECT_REARCAM_USE_ARUCO3_DETECTION,
                                                          constants::TAGDETECT_REARCAM_ENABLE_TRACKING,
                                                          constants::TAGDETECT_REARCAM_MAX_FPS,
                                                          constants::TAGDETECT_REARCAM_ENABLE_RECORDING,
                                                          constants::TAGDETECT_REARCAM_DATA_RETRIEVAL_THREADS,
                                                          constants::ZED_REARCAM_USE_GPU_MAT);

    // Check if torch detection is enabled for rear ZEDCam.
    if (constants::TAGDETECT_REARCAM_ENABLE_TORCH)
    {
        // Attempt to init torch detection.
        if (m_pTagDetectorRearCam->InitTorchDetection(constants::TAGDETECT_TORCH_MODEL))
        {
            // Set torch detection enabled.
            m_pTagDetectorRearCam->EnableTorchDetection(constants::TAGDETECT_REARCAM_TORCH_CONFIDENCE, constants::TAGDETECT_REARCAM_TORCH_NMS_THRESH);
        }
    }

    // Initialize recording handler for detectors.
    m_pRecordingHandler = std::make_unique<RecordingHandler>(RecordingHandler::RecordingMode::eTagDetectionHandler);
}

/******************************************************************************
 * @brief Destroy the TagDetectionHandler::TagDetectionHandler object.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
TagDetectionHandler::~TagDetectionHandler()
{
    // Signal and wait for cameras to stop.
    this->StopAllDetectors();
}

/******************************************************************************
 * @brief Signals all detectors to start their threads.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
void TagDetectionHandler::StartAllDetectors()
{
    // Start ZED maincam detector.
    m_pTagDetectorMainCam->Start();

    // Start ZED rearcam detector.
    m_pTagDetectorRearCam->Start();
}

/******************************************************************************
 * @brief Signal the RecordingHandler to start recording video feeds from the TagDetectionHandler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-31
 ******************************************************************************/
void TagDetectionHandler::StartRecording()
{
    // Start recording handler.
    m_pRecordingHandler->Start();
}

/******************************************************************************
 * @brief Signals all detectors to stop their threads.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
void TagDetectionHandler::StopAllDetectors()
{
    // Stop recording handler.
    m_pRecordingHandler->StopRecording();

    // Stop main ZED detectors.
    m_pTagDetectorMainCam->RequestStop();
    m_pTagDetectorMainCam->Join();

    // Stop rear ZED detectors.
    m_pTagDetectorRearCam->RequestStop();
    m_pTagDetectorRearCam->Join();
}

/******************************************************************************
 * @brief Signal the RecordingHandler to stop recording video feeds from the TagDetectionHandler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-01
 ******************************************************************************/
void TagDetectionHandler::StopRecording()
{
    // Stop recording handler.
    m_pRecordingHandler->StopRecording();
}

/******************************************************************************
 * @brief Accessor for TagDetector detectors.
 *
 * @param eDetectorName - The name of the detector to retrieve. An enum defined in and specific to this class.
 * @return std::shared_ptr<TagDetector> - A pointer to the detector pertaining to the given name.
 *
 * @author clayjay3 (claytonraycowen@gmail.com), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
std::shared_ptr<TagDetector> TagDetectionHandler::GetTagDetector(TagDetectors eDetectorName)
{
    // Determine which tag detector should be returned.
    switch (eDetectorName)
    {
        case TagDetectors::eHeadMainCam: return m_pTagDetectorMainCam; break;
        case TagDetectors::eRearCam: return m_pTagDetectorRearCam; break;
        default: return m_pTagDetectorMainCam; break;
    }
}

/******************************************************************************
 * @brief Requests a snapshot of the current tag detection overlay. Blocks execution until the frame is ready.
 *
 * @param eDetector - The detector to request the frame from.
 * @return cv::Mat - The frame with detection overlays.
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2026-01-04
 ******************************************************************************/
cv::Mat TagDetectionHandler::RequestDetectionOverlayFrame(TagDetectors eDetector)
{
    cv::Mat cvFrame;
    std::shared_ptr<TagDetector> pDetector = this->GetTagDetector(eDetector);

    if (pDetector && pDetector->GetIsReady())
    {
        std::future<bool> fuFrame = pDetector->RequestDetectionOverlayFrame(cvFrame);

        if (fuFrame.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
        {
            fuFrame.get();
        }
        else
        {
            LOG_WARNING(logging::g_qSharedLogger, "TagDetectionHandler: Timed out waiting for overlay snapshot.");
        }
    }
    else
    {
        LOG_WARNING(logging::g_qSharedLogger, "TagDetectionHandler: Requested snapshot from invalid or unready detector.");
    }

    return cvFrame;
}
