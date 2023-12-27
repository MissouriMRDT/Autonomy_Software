/******************************************************************************
 * @brief Implements the RecordingHandler class.
 *
 * @file RecordingHandler.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-26
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "RecordingHandler.h"
#include "../AutonomyConstants.h"
#include "../AutonomyGlobals.h"
#include "../AutonomyLogging.h"

/******************************************************************************
 * @brief Construct a new Recording Handler:: Recording Handler object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-26
 ******************************************************************************/
RecordingHandler::RecordingHandler()
{
    // Initialize member variables.
    m_nRecordingFPS = constants::RECORDER_FPS;
    m_nTotalCameras = CameraHandler::BasicCamName::BASICCAM_END + CameraHandler::ZEDCamName::ZEDCAM_END - 2;

    // Resize the video writer vector to match total number of cameras.
    m_vZEDCameras.resize(m_nTotalCameras);
    m_vBasicCameras.resize(m_nTotalCameras);
    m_vCameraWriters.resize(m_nTotalCameras);
    m_vRecordingToggles.resize(m_nTotalCameras);
    m_vFrames.resize(m_nTotalCameras);
    m_vGPUFrames.resize(m_nTotalCameras);
}

/******************************************************************************
 * @brief Destroy the Recording Handler:: Recording Handler object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-26
 ******************************************************************************/
RecordingHandler::~RecordingHandler()
{
    // Signal and wait for recording thread to stop.
    this->RequestStop();
    this->Join();
}

/******************************************************************************
 * @brief This code will run continuously in a separate thread. New frames from
 *      the cameras that have recording enabled are grabbed and the images are
 *      written to the filesystem.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-26
 ******************************************************************************/
void RecordingHandler::ThreadedContinuousCode()
{
    /////////////////////////////////////////
    // Update recordable cameras.
    /////////////////////////////////////////
    // Loop through all Basic cameras from the CameraHandler.
    for (int nCamera = CameraHandler::BasicCamName::BASICCAM_START + 1; nCamera != CameraHandler::BasicCamName::BASICCAM_END; ++nCamera)
    {
        // Get pointer to camera.
        BasicCam* pBasicCamera = globals::g_pCameraHandler->GetBasicCam(static_cast<CameraHandler::BasicCamName>(nCamera));
        // Store camera pointer in vector so we can get images later.
        m_vBasicCameras[nCamera - 1] = pBasicCamera;

        // Check if recording for this camera is enabled.
        if (pBasicCamera->GetEnableRecordingFlag() && pBasicCamera->GetCameraIsOpen())
        {
            // Set recording toggle.
            m_vRecordingToggles[nCamera - 1] = true;
            // Setup VideoWriter if needed.
            if (!m_vCameraWriters[nCamera - 1].isOpened())
            {
                // Assemble filepath string.
                std::string szFilenameWithExtension;
                szFilenameWithExtension = constants::RECORDER_OUTPUT_PATH_RELATIVE + "/";    // Main location for all recordings.
                szFilenameWithExtension += logging::g_szProgramStartTimeString + "/";        // Folder for each program run.
                szFilenameWithExtension += pBasicCamera->GetCameraLocation() + ".mp4";       // Folder for each camera index or name.

                // Open writer.
                m_vCameraWriters[nCamera - 1].open(szFilenameWithExtension,
                                                   cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                                                   constants::RECORDER_FPS,
                                                   cv::Size(pBasicCamera->GetPropResolutionX(), pBasicCamera->GetPropResolutionY()));
            }
        }
        else
        {
            // Set recording toggle.
            m_vRecordingToggles[nCamera - 1] = false;
        }
    }

    // Get index offset so we don't overwrite BasicCam pointers and booleans.
    int nIndexOffset = CameraHandler::BasicCamName::BASICCAM_END - 2;
    // Loop through all ZED cameras from the CameraHandler.
    for (int nCamera = CameraHandler::ZEDCamName::ZEDCAM_START + 1; nCamera != CameraHandler::ZEDCamName::ZEDCAM_END; ++nCamera)
    {
        // Get pointer to camera.
        ZEDCam* pZEDCamera = globals::g_pCameraHandler->GetZED(static_cast<CameraHandler::ZEDCamName>(nCamera));
        // Store camera pointer in vector so we can get images later.
        m_vZEDCameras[nCamera + nIndexOffset] = pZEDCamera;

        // Check if recording for this camera is enabled.
        if (pZEDCamera->GetEnableRecordingFlag() && pZEDCamera->GetCameraIsOpen())
        {
            // Set recording toggle.
            m_vRecordingToggles[nCamera + nIndexOffset] = true;
            // Setup VideoWriter if needed.
            if (!m_vCameraWriters[nCamera + nIndexOffset].isOpened())
            {
                // Assemble filepath string.
                std::string szFilenameWithExtension;
                szFilenameWithExtension = constants::RECORDER_OUTPUT_PATH_RELATIVE + "/";             // Main location for all recordings.
                szFilenameWithExtension += logging::g_szProgramStartTimeString + "/";                 // Folder for each program run.
                szFilenameWithExtension += std::to_string(pZEDCamera->GetCameraSerial()) + ".mp4";    // Folder for each camera index or name.

                // Open writer.
                m_vCameraWriters[nCamera + nIndexOffset].open(szFilenameWithExtension,
                                                              cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                                                              constants::RECORDER_FPS,
                                                              cv::Size(pZEDCamera->GetPropResolutionX(), pZEDCamera->GetPropResolutionY()));
            }
        }
        else
        {
            // Set recording toggle.
            m_vRecordingToggles[nCamera + nIndexOffset] = false;
        }
    }

    /////////////////////////////////////////
    // Request camera frames and write to files.
    /////////////////////////////////////////
    // Loop through total number of cameras and request frames.
    for (int nIter = 0; nIter < m_nTotalCameras; ++nIter)
    {
        // Check if recording for the camera at this index is enabled.
        if (m_vRecordingToggles[nIter])
        {
            // Request frame copy.
                }
    }
}

/******************************************************************************
 * @brief This method holds the code that is ran in the thread pool started by
 *      the ThreadedLinearCode() method. It currently does nothing and is not
 *      needed in the current implementation of the RecordingHandler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-26
 ******************************************************************************/
void RecordingHandler::PooledLinearCode() {}

/******************************************************************************
 * @brief Mutator for the desired FPS for all camera recordings.
 *
 * @param nRecordingFPS - The frames per second to record all cameras at.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-26
 ******************************************************************************/
void RecordingHandler::SetRecordingFPS(const int nRecordingFPS)
{
    // Assign member variable.
    m_nRecordingFPS = nRecordingFPS;
}

/******************************************************************************
 * @brief Accessor for the desired FPS for all camera recordings.
 *
 * @return int - The FPS of all camera recordings.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-26
 ******************************************************************************/
int RecordingHandler::GetRecordingFPS() const
{
    // Return member variable value.
    return m_nRecordingFPS;
}
