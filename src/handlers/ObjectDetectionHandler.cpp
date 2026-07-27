/******************************************************************************
 * @brief Implements the ObjectDetectionHandler class.
 *
 * @file ObjectDetectionHandler.cpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-23
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "ObjectDetectionHandler.h"
#include "../AutonomyConstants.h"
#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Construct a new ObjectDetectionHandler::ObjectDetectionHandler object.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
ObjectDetectionHandler::ObjectDetectionHandler()
{
    // Initialize detector for main ZEDCam.
    m_pObjectDetectorMainCam = std::make_shared<ObjectDetector>(globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam),
                                                                constants::OBJECTDETECT_MAINCAM_ENABLE_TRACKING,
                                                                constants::OBJECTDETECT_MAINCAM_MAX_FPS,
                                                                constants::OBJECTDETECT_MAINCAM_ENABLE_RECORDING,
                                                                constants::OBJECTDETECT_MAINCAM_DATA_RETRIEVAL_THREADS,
                                                                constants::ZED_MAINCAM_USE_GPU_MAT);

    // Check if torch detection is enabled for main ZEDCam.
    if (constants::OBJECTDETECT_MAINCAM_ENABLE_TORCH)
    {
        // Attempt to init torch detection.
        if (m_pObjectDetectorMainCam->InitTorchDetection(constants::OBJECTDETECT_TORCH_MODEL))
        {
            // Set torch detection enabled.
            m_pObjectDetectorMainCam->EnableTorchDetection(constants::OBJECTDETECT_MAINCAM_TORCH_CONFIDENCE, constants::OBJECTDETECT_MAINCAM_TORCH_NMS_THRESH);
        }
    }

    // Initialize detector for rear ZEDCam.
    m_pObjectDetectorRearCam = std::make_shared<ObjectDetector>(globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eRearCam),
                                                                constants::OBJECTDETECT_REARCAM_ENABLE_TRACKING,
                                                                constants::OBJECTDETECT_REARCAM_MAX_FPS,
                                                                constants::OBJECTDETECT_REARCAM_ENABLE_RECORDING,
                                                                constants::OBJECTDETECT_REARCAM_DATA_RETRIEVAL_THREADS,
                                                                constants::ZED_REARCAM_USE_GPU_MAT);

    // Check if torch detection is enabled for rear ZEDCam.
    if (constants::OBJECTDETECT_REARCAM_ENABLE_TORCH)
    {
        // Attempt to init torch detection.
        if (m_pObjectDetectorRearCam->InitTorchDetection(constants::OBJECTDETECT_TORCH_MODEL))
        {
            // Set torch detection enabled.
            m_pObjectDetectorRearCam->EnableTorchDetection(constants::OBJECTDETECT_REARCAM_TORCH_CONFIDENCE, constants::OBJECTDETECT_REARCAM_TORCH_NMS_THRESH);
        }
    }

    // Initialize recording handler for detectors.
    m_pRecordingHandler = std::make_unique<RecordingHandler>(RecordingHandler::RecordingMode::eObjectDetectionHandler);
}

/******************************************************************************
 * @brief Destroy the ObjectDetectionHandler::ObjectDetectionHandler object.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
ObjectDetectionHandler::~ObjectDetectionHandler()
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
void ObjectDetectionHandler::StartAllDetectors()
{
    // Start ZED maincam detector.
    m_pObjectDetectorMainCam->Start();

    // Start ZED rearcam detector.
    m_pObjectDetectorRearCam->Start();

    // Register this handler's demand for every detector's overlay channels. Detectors only clone
    // and publish overlay frames while a Subscription is alive, so holding these for the lifetime
    // of the detectors is what keeps GetDetectionOverlayFrame() supplied with frames.
    m_subMainCamOverlay         = m_pObjectDetectorMainCam->GetDetectionOverlayPublisher().Subscribe();
    m_subMainCamLastGoodOverlay = m_pObjectDetectorMainCam->GetLastGoodOverlayPublisher().Subscribe();
    m_subRearCamOverlay         = m_pObjectDetectorRearCam->GetDetectionOverlayPublisher().Subscribe();
    m_subRearCamLastGoodOverlay = m_pObjectDetectorRearCam->GetLastGoodOverlayPublisher().Subscribe();
}

/******************************************************************************
 * @brief Signal the RecordingHandler to start recording feeds from the detectors.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
void ObjectDetectionHandler::StartRecording()
{
    // Start recording for all detectors.
    m_pRecordingHandler->Start();
}

/******************************************************************************
 * @brief Signals all detectors to stop their threads.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
void ObjectDetectionHandler::StopAllDetectors()
{
    // Drop our overlay demand first so the detectors stop cloning frames nobody will read.
    m_subMainCamOverlay.Release();
    m_subMainCamLastGoodOverlay.Release();
    m_subRearCamOverlay.Release();
    m_subRearCamLastGoodOverlay.Release();

    // Stop recording handler.
    m_pRecordingHandler->RequestStop();
    m_pRecordingHandler->Join();

    // Stop ZED maincam detector.
    m_pObjectDetectorMainCam->RequestStop();
    m_pObjectDetectorMainCam->Join();

    // Stop ZED rearcam detector.
    m_pObjectDetectorRearCam->RequestStop();
    m_pObjectDetectorRearCam->Join();
}

/******************************************************************************
 * @brief  Signal the RecordingHandler to stop recording feeds from the detectors.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-05
 ******************************************************************************/
void ObjectDetectionHandler::StopRecording()
{
    // Stop recording handler.
    m_pRecordingHandler->RequestStop();
    m_pRecordingHandler->Join();
}

/******************************************************************************
 * @brief Accessor for ObjectDetector detectors.
 *
 * @param eDetectorName - The name of the detector to retrieve. An enum defined in and specific to this class.
 * @return std::shared_ptr<ObjectDetector> - A pointer to the detector pertaining to the given name.
 *
 * @author clayjay3 (claytonraycowen@gmail.com), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
std::shared_ptr<ObjectDetector> ObjectDetectionHandler::GetObjectDetector(ObjectDetectors eDetectorName)
{
    // Determine which object detector should be returned.
    switch (eDetectorName)
    {
        case ObjectDetectors::eHeadMainCam: return m_pObjectDetectorMainCam; break;
        case ObjectDetectors::eRearCam: return m_pObjectDetectorRearCam; break;
        default: return m_pObjectDetectorMainCam; break;
    }
}

/******************************************************************************
 * @brief Returns a snapshot of the current detection overlay. Does not block: it copies
 *      whatever the detector published most recently.
 *
 * @param eDetector - The detector to read the frame from.
 * @return cv::Mat - The frame with detection overlays, or an empty cv::Mat if the
 *                  detector is invalid, not ready, or has not published a frame yet.
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2026-01-01
 ******************************************************************************/
cv::Mat ObjectDetectionHandler::GetDetectionOverlayFrame(ObjectDetectors eDetector)
{
    // Create an empty frame to store the result.
    cv::Mat cvFrame;
    // Get the specific detector (e.g., Head Main Cam).
    std::shared_ptr<ObjectDetector> pDetector = this->GetObjectDetector(eDetector);

    // Check if the detector is valid and running.
    if (pDetector == nullptr || !pDetector->GetIsReady())
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "ObjectDetectionHandler: Requested snapshot from invalid or unready detector.");
        // Return the empty frame; the state machine handles that case.
        return cvFrame;
    }

    // Load the newest published overlay snapshot once into a local. This handler holds a
    // Subscription for the detector's lifetime, so the detector is publishing this channel.
    pubsub::Publisher<cv::Mat>::SharedSnapshot pSnapshot = pDetector->GetDetectionOverlayPublisher().Get();
    if (pSnapshot == nullptr)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "ObjectDetectionHandler: No detection overlay has been published yet.");
        // Return the empty frame.
        return cvFrame;
    }

    // Deep copy the immutable snapshot so the caller owns its frame.
    pSnapshot->tData.copyTo(cvFrame);

    // Return the frame.
    return cvFrame;
}
