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

#include <filesystem>

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

    // Resize vectors to match number of cameras.
    m_vZEDCameras.resize(m_nTotalCameras);
    m_vBasicCameras.resize(m_nTotalCameras);
    m_vCameraWriters.resize(m_nTotalCameras);
    m_vRecordingToggles.resize(m_nTotalCameras);
    m_vFrames.resize(m_nTotalCameras);
    m_vGPUFrames.resize(m_nTotalCameras);
    m_vFrameFutures.resize(m_nTotalCameras);

    // Set the max iterations per second of the recording handler.
    this->SetMainThreadMaxIPS(m_nRecordingFPS);
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

    // Loop through and close video writers.
    for (cv::VideoWriter cvCameraWriter : m_vCameraWriters)
    {
        // Release video writer.
        cvCameraWriter.release();
    }
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
                std::filesystem::path szFilePath;
                std::filesystem::path szFilenameWithExtension;
                szFilePath = constants::RECORDER_OUTPUT_PATH_ABSOLUTE;                              // Main location for all recordings.
                szFilePath += logging::g_szProgramStartTimeString + "/";                            // Folder for each program run.
                szFilenameWithExtension = "camera" + pBasicCamera->GetCameraLocation() + ".mp4";    // Folder for each camera index or name.

                // Check if directory exists.
                if (!std::filesystem::exists(szFilePath))
                {
                    // Create directory.
                    if (!std::filesystem::create_directory(szFilePath))
                    {
                        // Submit logger message.
                        LOG_ERROR(logging::g_qSharedLogger,
                                  "Unable to create the VideoWriter output directory: {} for camera {}",
                                  szFilePath.string(),
                                  pBasicCamera->GetCameraLocation());
                    }
                }
                else
                {
                    // Submit logger message.
                    LOG_ERROR(logging::g_qSharedLogger, "Unable to create VideoWriter output directory {}: it already exists.", szFilePath.string());
                }

                // Construct the full output path.
                std::filesystem::path szFullOutputPath = szFilePath / szFilenameWithExtension;

                // Open writer.
                bool bWriterOpened = m_vCameraWriters[nCamera - 1].open(szFullOutputPath.string(),
                                                                        cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                                                                        constants::RECORDER_FPS,
                                                                        cv::Size(pBasicCamera->GetPropResolutionX(), pBasicCamera->GetPropResolutionY()));

                // Check writer opened status.
                if (!bWriterOpened)
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger,
                                "RecordingHandler: Failed to open cv::VideoWriter for basic camera at path/index {}",
                                pBasicCamera->GetCameraLocation());
                }
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
                std::filesystem::path szFilePath;
                std::filesystem::path szFilenameWithExtension;
                szFilePath = constants::RECORDER_OUTPUT_PATH_ABSOLUTE;                                              // Main location for all recordings.
                szFilePath += logging::g_szProgramStartTimeString + "/";                                            // Folder for each program run.
                szFilenameWithExtension =
                    pZEDCamera->GetCameraModel() + "_" + std::to_string(pZEDCamera->GetCameraSerial()) + ".mp4";    // Folder for each camera index or name.

                // Check if directory exists.
                if (!std::filesystem::exists(szFilePath))
                {
                    // Create directory.
                    if (!std::filesystem::create_directory(szFilePath))
                    {
                        // Submit logger message.
                        LOG_ERROR(logging::g_qSharedLogger,
                                  "Unable to create the output directory: {} for camera {} ({})",
                                  szFilePath.string(),
                                  pZEDCamera->GetCameraModel(),
                                  pZEDCamera->GetCameraSerial());
                    }
                }

                // Construct the full output path.
                std::filesystem::path szFullOutputPath = szFilePath / szFilenameWithExtension;

                // Open writer.
                bool bWriterOpened = m_vCameraWriters[nCamera + nIndexOffset].open(szFullOutputPath,
                                                                                   cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                                                                                   constants::RECORDER_FPS,
                                                                                   cv::Size(pZEDCamera->GetPropResolutionX(), pZEDCamera->GetPropResolutionY()));

                // Check writer opened status.
                if (!bWriterOpened)
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger,
                                "RecordingHandler: Failed to open cv::VideoWriter for ZED camera with serial {}",
                                pZEDCamera->GetCameraSerial());
                }
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
            // Check if the camera at the current index is a BasicCam or ZEDCam.
            if (m_vBasicCameras[nIter] != nullptr)
            {
                // Request frame.
                m_vFrameFutures[nIter] = m_vBasicCameras[nIter]->RequestFrameCopy(m_vFrames[nIter]);
            }
            else if (m_vZEDCameras[nIter] != nullptr)
            {
                // Check if the camera is setup to use CPU or GPU mats.
                if (constants::ZED_MAINCAM_USE_GPU_MAT)
                {
                    // Grab frames from camera.
                    m_vFrameFutures[nIter] = m_vZEDCameras[nIter]->RequestFrameCopy(m_vGPUFrames[nIter]);
                }
                else
                {
                    // Grab frames from camera.
                    m_vFrameFutures[nIter] = m_vZEDCameras[nIter]->RequestFrameCopy(m_vFrames[nIter]);
                }
            }
        }
    }

    // Loop through cameras and wait for frame requests to be fulfilled.
    for (int nIter = 0; nIter < m_nTotalCameras; ++nIter)
    {
        // Check if recording for the camera at this index is enabled.
        if (m_vRecordingToggles[nIter])
        {
            // Check if the camera at the current index is a BasicCam or ZEDCam.
            if (m_vBasicCameras[nIter] != nullptr)
            {
                // Wait for future to be fulfilled.
                if (m_vFrameFutures[nIter].get() && !m_vFrames[nIter].empty())
                {
                    // Write frame to OpenCV video writer.
                    m_vCameraWriters[nIter].write(m_vFrames[nIter]);
                }
            }
            else if (m_vZEDCameras[nIter] != nullptr)
            {
                // Check if the camera is setup to use CPU or GPU mats.
                if (constants::ZED_MAINCAM_USE_GPU_MAT)
                {
                    // Wait for future to be fulfilled.
                    if (m_vFrameFutures[nIter].get() && !m_vGPUFrames[nIter].empty())
                    {
                        // Download GPU mat frame to normal mat.
                        m_vGPUFrames[nIter].download(m_vFrames[nIter]);
                        // Write frame to OpenCV video writer.
                        m_vCameraWriters[nIter].write(m_vFrames[nIter]);
                    }
                }
                else
                {
                    // Wait for future to be fulfilled.
                    if (m_vFrameFutures[nIter].get() && !m_vFrames[nIter].empty())
                    {
                        // Write frame to OpenCV video writer.
                        m_vCameraWriters[nIter].write(m_vFrames[nIter]);
                    }
                }
            }
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
