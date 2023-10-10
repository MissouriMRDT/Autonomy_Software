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
    // Initialize detector for main ZEDcam.
    m_pTagDetectorMainCam = new TagDetector(globals::g_pCameraHandler->GetZED(CameraHandler::eHeadMainCam),
                                            constants::TAGDETECT_MAINCAM_DATA_RETRIEVAL_THREADS,
                                            constants::ZED_MAINCAM_USE_GPU_MAT);

    // Initialize detector for left aruco BasicCam.
    m_pTagDetectorLeftCam =
        new TagDetector(globals::g_pCameraHandler->GetBasicCam(CameraHandler::eHeadLeftArucoEye), constants::TAGDETECT_LEFTCAM_DATA_RETRIEVAL_THREADS, false);
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

    // Set dangling pointers to nullptr.
    m_pTagDetectorMainCam = nullptr;
    m_pTagDetectorLeftCam = nullptr;
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
    // Stop ZED maincam detector.
    m_pTagDetectorMainCam->RequestStop();
    m_pTagDetectorMainCam->Join();

    // Stop BasicCam left aruco eye detector.
    m_pTagDetectorLeftCam->RequestStop();
    m_pTagDetectorLeftCam->Join();
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
        default: return m_pTagDetectorMainCam;
    }
}
