/******************************************************************************
 * @brief Implements the TagDetectionHandler class.
 *
 * @file TagDetectionHandler.cpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "TagDetectionHandler.h"
#include "../AutonomyConstants.h"
#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Construct a new TagDetectionHandler::TagDetectionHandler object.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
TagDetectionHandler::TagDetectionHandler()
{
    // Initialize detector for main ZEDCam.
    m_pTagDetectorMainCam = new TagDetector(globals::g_pCameraHandler->GetZED(CameraHandler::eHeadMainCam),
                                            constants::TAGDETECT_MAINCAM_CORNER_REFINE_MAX_ITER,
                                            constants::TAGDETECT_MAINCAM_CORNER_REFINE_METHOD,
                                            constants::TAGDETECT_MAINCAM_MARKER_BORDER_BITS,
                                            constants::TAGDETECT_MAINCAM_DETECT_INVERTED_MARKER,
                                            constants::TAGDETECT_MAINCAM_USE_ARUCO3_DETECTION,
                                            constants::TAGDETECT_MAINCAM_MAX_FPS,
                                            constants::TAGDETECT_MAINCAM_ENABLE_RECORDING,
                                            constants::TAGDETECT_MAINCAM_DATA_RETRIEVAL_THREADS,
                                            constants::ZED_MAINCAM_USE_GPU_MAT);

    // Initialize detector for left aruco BasicCam.
    m_pTagDetectorLeftCam = new TagDetector(globals::g_pCameraHandler->GetBasicCam(CameraHandler::eHeadLeftArucoEye),
                                            constants::TAGDETECT_LEFTCAM_CORNER_REFINE_MAX_ITER,
                                            constants::TAGDETECT_LEFTCAM_CORNER_REFINE_METHOD,
                                            constants::TAGDETECT_LEFTCAM_MARKER_BORDER_BITS,
                                            constants::TAGDETECT_LEFTCAM_DETECT_INVERTED_MARKER,
                                            constants::TAGDETECT_LEFTCAM_USE_ARUCO3_DETECTION,
                                            constants::TAGDETECT_LEFTCAM_MAX_FPS,
                                            constants::TAGDETECT_LEFTCAM_ENABLE_RECORDING,
                                            constants::TAGDETECT_LEFTCAM_DATA_RETRIEVAL_THREADS,
                                            false);

    // Initialize detector for right aruco BasicCam.
    m_pTagDetectorRightCam = new TagDetector(globals::g_pCameraHandler->GetBasicCam(CameraHandler::eHeadRightArucoEye),
                                             constants::TAGDETECT_RIGHTCAM_CORNER_REFINE_MAX_ITER,
                                             constants::TAGDETECT_RIGHTCAM_CORNER_REFINE_METHOD,
                                             constants::TAGDETECT_RIGHTCAM_MARKER_BORDER_BITS,
                                             constants::TAGDETECT_RIGHTCAM_DETECT_INVERTED_MARKER,
                                             constants::TAGDETECT_RIGHTCAM_USE_ARUCO3_DETECTION,
                                             constants::TAGDETECT_RIGHTCAM_MAX_FPS,
                                             constants::TAGDETECT_RIGHTCAM_ENABLE_RECORDING,
                                             constants::TAGDETECT_RIGHTCAM_DATA_RETRIEVAL_THREADS,
                                             false);

    // // Check if tensorflow detection is enabled for main ZEDCam.
    // if (constants::TAGDETECT_MAINCAM_ENABLE_DNN)
    // {
    //     // Attempt to init tensorflow detection.
    //     if (m_pTagDetectorMainCam->InitTensorflowDetection(constants::TAGDETECT_MAINCAM_MODEL_PATH))
    //     {
    //         // Set tensorflow detection enabled.
    //         m_pTagDetectorMainCam->EnableTensorflowDetection(0.4f, 0.6f);
    //     }
    // }
    // // Check if tensorflow detection is enabled for left BasicCam.
    // if (constants::TAGDETECT_LEFTCAM_ENABLE_DNN)
    // {
    //     // Attempt to init tensorflow detection.
    //     if (m_pTagDetectorLeftCam->InitTensorflowDetection(constants::TAGDETECT_LEFTCAM_MODEL_PATH))
    //     {
    //         // Set tensorflow detection enabled.
    //         m_pTagDetectorLeftCam->EnableTensorflowDetection(0.4f, 0.6f);
    //     }
    // }
    // // Check if tensorflow detection is enabled for right BasicCam.
    // if (constants::TAGDETECT_RIGHTCAM_ENABLE_DNN)
    // {
    //     // Attempt to init tensorflow detection.
    //     if (m_pTagDetectorRightCam->InitTensorflowDetection(constants::TAGDETECT_RIGHTCAM_MODEL_PATH))
    //     {
    //         // Set tensorflow detection enabled.
    //         m_pTagDetectorRightCam->EnableTensorflowDetection(0.4f, 0.6f);
    //     }
    // }

    // Initialize recording handler for detectors.
    m_pRecordingHandler = new RecordingHandler(RecordingHandler::RecordingMode::eTagDetectionHandler);
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

    // Delete dynamic memory.
    delete m_pTagDetectorMainCam;
    delete m_pTagDetectorLeftCam;
    delete m_pTagDetectorRightCam;
    delete m_pRecordingHandler;

    // Set dangling pointers to nullptr.
    m_pTagDetectorMainCam  = nullptr;
    m_pTagDetectorLeftCam  = nullptr;
    m_pTagDetectorRightCam = nullptr;
    m_pRecordingHandler    = nullptr;
}

/******************************************************************************
 * @brief Signals all detectors to start their threads.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
void TagDetectionHandler::StartAllDetectors()
{
    // Start ZED maincam detector.
    m_pTagDetectorMainCam->Start();

    // Start the left and right aruco eyes.
    m_pTagDetectorLeftCam->Start();
    m_pTagDetectorRightCam->Start();
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
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
void TagDetectionHandler::StopAllDetectors()
{
    // Stop recording handler.
    m_pRecordingHandler->RequestStop();
    m_pRecordingHandler->Join();

    // Stop ZED maincam detector.
    m_pTagDetectorMainCam->RequestStop();
    m_pTagDetectorMainCam->Join();

    // Stop BasicCam left aruco eye detector.
    m_pTagDetectorLeftCam->RequestStop();
    m_pTagDetectorRightCam->RequestStop();
    m_pTagDetectorLeftCam->Join();
    m_pTagDetectorRightCam->Join();
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
    m_pRecordingHandler->RequestStop();
    m_pRecordingHandler->Join();
}

/******************************************************************************
 * @brief Accessor for TagDetector detectors.
 *
 * @param eDetectorName - The name of the detector to retrieve. An enum defined in and specific to this class.
 * @return TagDetector* - A pointer to the detector pertaining to the given name.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
TagDetector* TagDetectionHandler::GetTagDetector(TagDetectors eDetectorName)
{
    // Determine which tag detector should be returned.
    switch (eDetectorName)
    {
        case eHeadMainCam: return m_pTagDetectorMainCam;
        case eHeadLeftArucoEye: return m_pTagDetectorLeftCam;
        case eHeadRightArucoEye: return m_pTagDetectorRightCam;
        default: return m_pTagDetectorMainCam;
    }
}
