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

/// \cond
#include <filesystem>

/// \endcond

/******************************************************************************
 * @brief Construct a new Recording Handler:: Recording Handler object.
 *
 * @param eRecordingMode -
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-05
 ******************************************************************************/
RecordingHandler::RecordingHandler(RecordingMode eRecordingMode)
{
    // Initialize member variables.
    m_eRecordingMode = eRecordingMode;
    // Set max FPS of the ThreadedContinuousCode method.
    this->SetRecordingFPS(constants::RECORDER_FPS);

    // Resize vectors to match number of video feeds.
    switch (eRecordingMode)
    {
        // RecordingHandler was initialized to record feeds from the CameraHandler.
        case eCameraHandler:
            // Initialize member variables.
            m_nTotalVideoFeeds = CameraHandler::BasicCamName::BASICCAM_END + CameraHandler::ZEDCamName::ZEDCAM_END - 2;
            // Resize member vectors to match number of total video feeds to record.
            m_vZEDCameras.resize(m_nTotalVideoFeeds);
            m_vBasicCameras.resize(m_nTotalVideoFeeds);
            m_vCameraWriters.resize(m_nTotalVideoFeeds);
            m_vRecordingToggles.resize(m_nTotalVideoFeeds);
            m_vFrames.resize(m_nTotalVideoFeeds);
            m_vGPUFrames.resize(m_nTotalVideoFeeds);
            m_vFrameFutures.resize(m_nTotalVideoFeeds);
            break;

        // RecordingHandler was initialized to record feeds from the TagDetectionHandler.
        case eTagDetectionHandler:
            // Initialize member variables.
            m_nTotalVideoFeeds = TagDetectionHandler::TagDetectors::TAGDETECTOR_END - 1;
            // Resize member vectors to match number of total video feeds to record.
            m_vTagDetectors.resize(m_nTotalVideoFeeds);
            m_vCameraWriters.resize(m_nTotalVideoFeeds);
            m_vRecordingToggles.resize(m_nTotalVideoFeeds);
            m_vFrames.resize(m_nTotalVideoFeeds);
            m_vFrameFutures.resize(m_nTotalVideoFeeds);
            break;

        default:
            // Do nothing.
            break;
    }
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
    // Check what mode recorder was initialized with.
    switch (m_eRecordingMode)
    {
        // Record video feeds from the CameraHandler.
        case eCameraHandler:
            // Update recordable cameras.
            this->UpdateRecordableCameras();
            // Grab and write frames to VideoWriters.
            this->RequestAndWriteCameraFrames();
            break;

        // Record video feeds from the TagDetectionHandler.
        case eTagDetectionHandler:
            // Update recordable detectors.
            this->UpdateRecordableTagDetectors();
            // Grab and write overlay frames to VideoWriters.
            this->RequestAndWriteTagDetectorFrames();
            break;

        // Shutdown recording handler.
        default:
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger,
                      "The RecordingHandler was initialized with a RecordingMode enum value that doesn't make sense! Thread is shutting down...");
            // Request main thread stop.
            this->RequestStop();
            break;
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
 * @brief This method is used internally by the class to update the number of cameras
 *      that have recording enabled from the camera handler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-31
 ******************************************************************************/
void RecordingHandler::UpdateRecordableCameras()
{
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
                szFilePath = constants::LOGGING_OUTPUT_PATH_ABSOLUTE;                    // Main location for all recordings.
                szFilePath += logging::g_szProgramStartTimeString + "/cameras";          // Folder for each program run.
                szFilenameWithExtension = pBasicCamera->GetCameraLocation() + ".mkv";    // Folder for each camera index or name.

                // Check if directory exists.
                if (!std::filesystem::exists(szFilePath))
                {
                    // Create directory.
                    if (!std::filesystem::create_directories(szFilePath))
                    {
                        // Submit logger message.
                        LOG_ERROR(logging::g_qSharedLogger,
                                  "Unable to create the VideoWriter output directory: {} for camera {}",
                                  szFilePath.string(),
                                  pBasicCamera->GetCameraLocation());
                    }
                }

                // Construct the full output path.
                std::filesystem::path szFullOutputPath = szFilePath / szFilenameWithExtension;

                // Open writer.
                bool bWriterOpened = m_vCameraWriters[nCamera - 1].open(szFullOutputPath.string(),
                                                                        cv::VideoWriter::fourcc('H', '2', '6', '4'),
                                                                        constants::RECORDER_FPS,
                                                                        pBasicCamera->GetPropResolution());

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
                szFilePath = constants::LOGGING_OUTPUT_PATH_ABSOLUTE;                                               // Main location for all recordings.
                szFilePath += logging::g_szProgramStartTimeString + "/cameras";                                     // Folder for each program run.
                szFilenameWithExtension =
                    pZEDCamera->GetCameraModel() + "_" + std::to_string(pZEDCamera->GetCameraSerial()) + ".mkv";    // Folder for each camera index or name.

                // Check if directory exists.
                if (!std::filesystem::exists(szFilePath))
                {
                    // Create directory.
                    if (!std::filesystem::create_directories(szFilePath))
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
                                                                                   cv::VideoWriter::fourcc('H', '2', '6', '4'),
                                                                                   constants::RECORDER_FPS,
                                                                                   pZEDCamera->GetPropResolution());

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
}

/******************************************************************************
 * @brief This method is used internally by the RecordingHandler to request and write
 *      frames to from the cameras stored in the member variable vectors.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-01
 ******************************************************************************/
void RecordingHandler::RequestAndWriteCameraFrames()
{
    // Loop through total number of cameras and request frames.
    for (int nIter = 0; nIter < m_nTotalVideoFeeds; ++nIter)
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
    for (int nIter = 0; nIter < m_nTotalVideoFeeds; ++nIter)
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
                    // Check if this is a grayscale or color image.
                    if (m_vFrames[nIter].channels() == 1)
                    {
                        // Convert frame from 1 channel grayscale to 3 channel BGR.
                        cv::cvtColor(m_vFrames[nIter], m_vFrames[nIter], cv::COLOR_GRAY2BGR);
                    }
                    // Check if this has an alpha channel.
                    else if (m_vFrames[nIter].channels() == 4)
                    {
                        // Convert from from 4 channels to 3 channels.
                        cv::cvtColor(m_vFrames[nIter], m_vFrames[nIter], cv::COLOR_BGRA2BGR);
                    }

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

                        // Check if this is a grayscale or color image.
                        if (m_vFrames[nIter].channels() == 1)
                        {
                            // Convert frame from 1 channel grayscale to 3 channel BGR.
                            cv::cvtColor(m_vFrames[nIter], m_vFrames[nIter], cv::COLOR_GRAY2BGR);
                        }
                        // Check if this has an alpha channel.
                        else if (m_vFrames[nIter].channels() == 4)
                        {
                            // Convert from from 4 channels to 3 channels.
                            cv::cvtColor(m_vFrames[nIter], m_vFrames[nIter], cv::COLOR_BGRA2BGR);
                        }

                        // Write frame to OpenCV video writer.
                        m_vCameraWriters[nIter].write(m_vFrames[nIter]);
                    }
                }
                else
                {
                    // Wait for future to be fulfilled.
                    if (m_vFrameFutures[nIter].get() && !m_vFrames[nIter].empty())
                    {
                        // Check if this is a grayscale or color image.
                        if (m_vFrames[nIter].channels() == 1)
                        {
                            // Convert frame from 1 channel grayscale to 3 channel BGR.
                            cv::cvtColor(m_vFrames[nIter], m_vFrames[nIter], cv::COLOR_GRAY2BGR);
                        }
                        // Check if this has an alpha channel.
                        else if (m_vFrames[nIter].channels() == 4)
                        {
                            // Convert from from 4 channels to 3 channels.
                            cv::cvtColor(m_vFrames[nIter], m_vFrames[nIter], cv::COLOR_BGRA2BGR);
                        }

                        // Write frame to OpenCV video writer.
                        m_vCameraWriters[nIter].write(m_vFrames[nIter]);
                    }
                }
            }
        }
    }
}

/******************************************************************************
 * @brief This method is used internally by the class to update the number of TagDetectors
 *      that have recording enabled from the camera handler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-31
 ******************************************************************************/
void RecordingHandler::UpdateRecordableTagDetectors()
{
    // Loop through all Basic cameras from the CameraHandler.
    for (int nDetector = TagDetectionHandler::TagDetectors::TAGDETECTOR_START + 1; nDetector != TagDetectionHandler::TagDetectors::TAGDETECTOR_END; ++nDetector)
    {
        // Get pointer to camera.
        TagDetector* pTagDetector = globals::g_pTagDetectionHandler->GetTagDetector(static_cast<TagDetectionHandler::TagDetectors>(nDetector));
        // Store camera pointer in vector so we can get images later.
        m_vTagDetectors[nDetector - 1] = pTagDetector;

        // Check if recording for this camera is enabled.
        if (pTagDetector->GetEnableRecordingFlag() && pTagDetector->GetIsReady())
        {
            // Set recording toggle.
            m_vRecordingToggles[nDetector - 1] = true;
            // Setup VideoWriter if needed.
            if (!m_vCameraWriters[nDetector - 1].isOpened())
            {
                // Assemble filepath string.
                std::filesystem::path szFilePath;
                std::filesystem::path szFilenameWithExtension;
                szFilePath = constants::LOGGING_OUTPUT_PATH_ABSOLUTE;                  // Main location for all recordings.
                szFilePath += logging::g_szProgramStartTimeString + "/tagdetector";    // Folder for each program run.
                szFilenameWithExtension = pTagDetector->GetCameraName() + ".mkv";      // Folder for each camera index or name.

                // Check if directory exists.
                if (!std::filesystem::exists(szFilePath))
                {
                    // Create directory.
                    if (!std::filesystem::create_directories(szFilePath))
                    {
                        // Submit logger message.
                        LOG_ERROR(logging::g_qSharedLogger,
                                  "Unable to create the VideoWriter output directory: {} for camera {}",
                                  szFilePath.string(),
                                  pTagDetector->GetCameraName());
                    }
                }

                // Construct the full output path.
                std::filesystem::path szFullOutputPath = szFilePath / szFilenameWithExtension;

                // Open writer.
                bool bWriterOpened = m_vCameraWriters[nDetector - 1].open(szFullOutputPath.string(),
                                                                          cv::VideoWriter::fourcc('H', '2', '6', '4'),
                                                                          constants::RECORDER_FPS,
                                                                          pTagDetector->GetProcessFrameResolution());

                // Check writer opened status.
                if (!bWriterOpened)
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger,
                                "RecordingHandler: Failed to open cv::VideoWriter for basic camera at path/index {}",
                                pTagDetector->GetCameraName());
                }
            }
        }
        else
        {
            // Set recording toggle.
            m_vRecordingToggles[nDetector - 1] = false;
        }
    }
}

/******************************************************************************
 * @brief This method is used internally by the RecordingHandler to request and write
 *      frames to from the TagDetectors stored in the member variable vectors.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-01
 ******************************************************************************/
void RecordingHandler::RequestAndWriteTagDetectorFrames()
{
    // Loop through total number of cameras and request frames.
    for (int nIter = 0; nIter < m_nTotalVideoFeeds; ++nIter)
    {
        // Check if recording for the camera at this index is enabled and tag detector at index is not null.
        if (m_vRecordingToggles[nIter] && m_vTagDetectors[nIter] != nullptr)
        {
            // Request frame.
            m_vFrameFutures[nIter] = m_vTagDetectors[nIter]->RequestDetectionOverlayFrame(m_vFrames[nIter]);
        }
    }

    // Loop through cameras and wait for frame requests to be fulfilled.
    for (int nIter = 0; nIter < m_nTotalVideoFeeds; ++nIter)
    {
        // Check if recording for the camera at this index is enabled and tag detector is not null.
        if (m_vRecordingToggles[nIter] && m_vTagDetectors[nIter] != nullptr)
        {
            // Wait for future to be fulfilled.
            if (m_vFrameFutures[nIter].get() && !m_vFrames[nIter].empty())
            {
                // Check if this is a grayscale or color image.
                if (m_vFrames[nIter].channels() == 1)
                {
                    // Convert frame from 1 channel grayscale to 3 channel BGR.
                    cv::cvtColor(m_vFrames[nIter], m_vFrames[nIter], cv::COLOR_GRAY2BGR);
                }
                // Check if this has an alpha channel.
                else if (m_vFrames[nIter].channels() == 4)
                {
                    // Convert from from 4 channels to 3 channels.
                    cv::cvtColor(m_vFrames[nIter], m_vFrames[nIter], cv::COLOR_BGRA2BGR);
                }

                // Write frame to OpenCV video writer.
                m_vCameraWriters[nIter].write(m_vFrames[nIter]);
            }
        }
    }
}

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
    // Set the max iterations per second of the recording handler.
    this->SetMainThreadIPSLimit(nRecordingFPS);
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
    return this->GetMainThreadMaxIPS();
}
