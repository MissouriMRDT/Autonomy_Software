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
                                                                constants::OBJECTDETECT_MAINCAM_DATA_RETRIEVAL_THREADS,
                                                                constants::ZED_MAINCAM_USE_GPU_MAT);
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
