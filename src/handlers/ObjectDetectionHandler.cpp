/******************************************************************************
 * @brief Implements the ObjectDetectionHandler class.
 *
 * @file ObjectDetectionHandler.cpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-23
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
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
    m_pObjectDetectorMainCam = new ObjectDetector(globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam),
                                                  constants::OBJECTDETECT_MAINCAM_DATA_RETRIEVAL_THREADS,
                                                  constants::ZED_MAINCAM_USE_GPU_MAT);

    // Initialize detector for left aruco BasicCam.
    m_pObjectDetectorLeftCam = new ObjectDetector(globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eFrameLeftCam),
                                                  constants::OBJECTDETECT_LEFTCAM_DATA_RETRIEVAL_THREADS,
                                                  constants::ZED_LEFTCAM_USE_GPU_MAT);

    // Initialize detector for right aruco BasicCam.
    m_pObjectDetectorRightCam = new ObjectDetector(globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eFrameRightCam),
                                                   constants::OBJECTDETECT_RIGHTCAM_DATA_RETRIEVAL_THREADS,
                                                   constants::ZED_RIGHTCAM_USE_GPU_MAT);
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

    // Delete dynamic memory.
    delete m_pObjectDetectorMainCam;
    delete m_pObjectDetectorLeftCam;
    delete m_pObjectDetectorRightCam;

    // Set dangling pointers to nullptr.
    m_pObjectDetectorMainCam  = nullptr;
    m_pObjectDetectorLeftCam  = nullptr;
    m_pObjectDetectorRightCam = nullptr;
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

    // Start the left and right aruco eyes.
    m_pObjectDetectorLeftCam->Start();
    m_pObjectDetectorRightCam->Start();
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
    // Stop ZED maincam detector.
    m_pObjectDetectorMainCam->RequestStop();
    m_pObjectDetectorMainCam->Join();

    // Stop BasicCam left aruco eye detector.
    m_pObjectDetectorLeftCam->RequestStop();
    m_pObjectDetectorRightCam->RequestStop();
    m_pObjectDetectorLeftCam->Join();
    m_pObjectDetectorRightCam->Join();
}

/******************************************************************************
 * @brief Accessor for ObjectDetector detectors.
 *
 * @param eDetectorName - The name of the detector to retrieve. An enum defined in and specific to this class.
 * @return ObjectDetector* - A pointer to the detector pertaining to the given name.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
ObjectDetector* ObjectDetectionHandler::GetObjectDetector(ObjectDetectors eDetectorName)
{
    // Determine which object detector should be returned.
    switch (eDetectorName)
    {
        case ObjectDetectors::eHeadMainCam: return m_pObjectDetectorMainCam;
        case ObjectDetectors::eFrameLeftCam: return m_pObjectDetectorLeftCam;
        case ObjectDetectors::eFrameRightCam: return m_pObjectDetectorRightCam;
        default: return m_pObjectDetectorMainCam;
    }
}
