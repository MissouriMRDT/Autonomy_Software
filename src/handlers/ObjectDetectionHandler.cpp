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
 * @author ClayJay3 (claytonraycowen@gmail.com)
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
        if (m_pObjectDetectorMainCam->InitTorchDetection(constants::OBJECTDETECT_MAINCAM_TORCH_MODEL))
        {
            // Set torch detection enabled.
            m_pObjectDetectorMainCam->EnableTorchDetection(constants::OBJECTDETECT_MAINCAM_TORCH_CONFIDENCE, constants::OBJECTDETECT_MAINCAM_TORCH_NMS_THRESH);
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
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
void ObjectDetectionHandler::StartAllDetectors()
{
    // Start ZED maincam detector.
    m_pObjectDetectorMainCam->Start();
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
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
void ObjectDetectionHandler::StopAllDetectors()
{
    // Stop recording handler.
    m_pRecordingHandler->RequestStop();
    m_pRecordingHandler->Join();

    // Stop ZED maincam detector.
    m_pObjectDetectorMainCam->RequestStop();
    m_pObjectDetectorMainCam->Join();
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
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
std::shared_ptr<ObjectDetector> ObjectDetectionHandler::GetObjectDetector(ObjectDetectors eDetectorName)
{
    // Determine which object detector should be returned.
    switch (eDetectorName)
    {
        case ObjectDetectors::eHeadMainCam: return m_pObjectDetectorMainCam; break;
        default: return m_pObjectDetectorMainCam; break;
    }
}

/******************************************************************************
 * @brief Requests a snapshot of the current detection overlay. Blocks execution until the frame is ready.
 *
 * @param eDetector - The detector to request the frame from.
 * @return cv::Mat - The frame with detection overlays.
 * @author Targed (ltklionel@gmail.com)
 * @date 2026-01-01
 ******************************************************************************/
cv::Mat ObjectDetectionHandler::RequestDetectionOverlayFrame(ObjectDetectors eDetector)
{
    // Create an empty frame to store the result
    cv::Mat cvFrame;

    // Get the specific detector (e.g., Head Main Cam)
    std::shared_ptr<ObjectDetector> pDetector = this->GetObjectDetector(eDetector);

    // Check if the detector is valid and running
    if (pDetector && pDetector->GetIsReady())
    {
        // Request the frame. This returns a "future" (a promise that data will come later)
        std::future<bool> fuFrame = pDetector->RequestDetectionOverlayFrame(cvFrame);

        // Wait for the detector thread to fulfill the promise
        if (fuFrame.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
        {
            // Retrieve the result (this ensures any exceptions are handled, though rare here)
            fuFrame.get();
        }
        else
        {
            LOG_WARNING(logging::g_qSharedLogger, "ObjectDetectionHandler: Timed out waiting for overlay snapshot.");
        }
    }
    else
    {
        LOG_WARNING(logging::g_qSharedLogger, "ObjectDetectionHandler: Requested snapshot from invalid or unready detector.");
    }

    // Return the frame (it will be empty if anything failed, which the State Machine handles)
    return cvFrame;
}