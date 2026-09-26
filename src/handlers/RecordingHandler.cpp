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
    this->SetMainThreadIPSLimit(constants::RECORDER_FPS);
    // Name the OS thread after the feeds it records so profilers, system tools and the visualizer can tell the recorders apart.
    this->SetMainThreadName(eRecordingMode == RecordingMode::eCameraHandler         ? "RecCameras"
                            : eRecordingMode == RecordingMode::eTagDetectionHandler ? "RecTagDetect"
                                                                                    : "RecObjDetect");

    // Resize vectors to match number of video feeds.
    switch (eRecordingMode)
    {
        // RecordingHandler was initialized to record feeds from the CameraHandler.
        case RecordingMode::eCameraHandler:
            // Initialize member variables.
            m_nTotalVideoFeeds = int(CameraHandler::BasicCamName::BASICCAM_END) + int(CameraHandler::ZEDCamName::ZEDCAM_END) - 2;
            // Resize member vectors to match number of total video feeds to record.
            m_vZEDCameras.resize(m_nTotalVideoFeeds);
            m_vBasicCameras.resize(m_nTotalVideoFeeds);
            m_vCameraWriters.resize(m_nTotalVideoFeeds);
            m_vRecordingToggles.resize(m_nTotalVideoFeeds);
            m_vFrames.resize(m_nTotalVideoFeeds);
            m_vGPUFrames.resize(m_nTotalVideoFeeds);
            m_vFrameReadersCPU.resize(m_nTotalVideoFeeds);
            m_vFrameReadersGPU.resize(m_nTotalVideoFeeds);
            break;

        // RecordingHandler was initialized to record feeds from the TagDetectionHandler.
        case RecordingMode::eTagDetectionHandler:
            // Initialize member variables.
            m_nTotalVideoFeeds = int(TagDetectionHandler::TagDetectors::TAGDETECTOR_END) - 1;
            // Resize member vectors to match number of total video feeds to record.
            m_vTagDetectors.resize(m_nTotalVideoFeeds);
            m_vCameraWriters.resize(m_nTotalVideoFeeds);
            m_vRecordingToggles.resize(m_nTotalVideoFeeds);
            m_vFrameReadersCPU.resize(m_nTotalVideoFeeds);
            m_vFrameReadersGPU.resize(m_nTotalVideoFeeds);
            break;

        // RecordingHandler was initialized to record feeds from the TagDetectionHandler.
        case RecordingMode::eObjectDetectionHandler:
            // Initialize member variables.
            m_nTotalVideoFeeds = int(ObjectDetectionHandler::ObjectDetectors::OBJECTDETECTOR_END) - 1;
            // Resize member vectors to match number of total video feeds to record.
            m_vObjectDetectors.resize(m_nTotalVideoFeeds);
            m_vCameraWriters.resize(m_nTotalVideoFeeds);
            m_vRecordingToggles.resize(m_nTotalVideoFeeds);
            m_vFrameReadersCPU.resize(m_nTotalVideoFeeds);
            m_vFrameReadersGPU.resize(m_nTotalVideoFeeds);
            break;

        default:
            // Do nothing.
            break;
    }

    // Create one (closed) encoder per feed. They are opened when the feed's recording is enabled.
    for (std::unique_ptr<VideoEncoder>& pEncoder : m_vCameraWriters)
    {
        pEncoder = std::make_unique<VideoEncoder>();
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

    // Loop through and close the encoders. This flushes their delayed frames and finalizes the files.
    for (std::unique_ptr<VideoEncoder>& pEncoder : m_vCameraWriters)
    {
        // Close the encoder.
        pEncoder->Close();
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
    ZoneScopedC(tracy::Color::DarkOrange1);
    // Check what mode recorder was initialized with.
    switch (m_eRecordingMode)
    {
        // Record video feeds from the CameraHandler.
        case RecordingMode::eCameraHandler:
            // Update recordable cameras.
            this->UpdateRecordableCameras();
            // Grab and write frames to the video files.
            this->RequestAndWriteCameraFrames();
            break;

        // Record video feeds from the TagDetectionHandler.
        case RecordingMode::eTagDetectionHandler:
            // Update recordable detectors.
            this->UpdateRecordableTagDetectors();
            // Grab and write overlay frames to the video files.
            this->RequestAndWriteTagDetectorFrames();
            break;

        // Record video feeds from the ObjectDetectionHandler.
        case RecordingMode::eObjectDetectionHandler:
            // Update recordable detectors.
            this->UpdateRecordableObjectDetectors();
            // Grab and write overlay frames to the video files.
            this->RequestAndWriteObjectDetectorFrames();
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
    ZoneScopedC(tracy::Color::DarkOrange2);
    // Loop through all Basic cameras from the CameraHandler.
    for (int nCamera = int(CameraHandler::BasicCamName::BASICCAM_START) + 1; nCamera != int(CameraHandler::BasicCamName::BASICCAM_END); ++nCamera)
    {
        // Get pointer to camera.
        std::shared_ptr<BasicCamera> pBasicCamera = globals::g_pCameraHandler->GetBasicCam(static_cast<CameraHandler::BasicCamName>(nCamera));
        // Store camera pointer in vector so we can get images later.
        m_vBasicCameras[nCamera - 1] = pBasicCamera;

        // Check if recording for this camera is enabled.
        if (pBasicCamera->GetEnableRecordingFlag() && pBasicCamera->GetCameraIsOpen())
        {
            // Set recording toggle.
            m_vRecordingToggles[nCamera - 1] = true;
            // Take a read handle on this camera's frames if we have not already. The camera only
            // retrieves and publishes frames while a Reader is alive, so this is what makes the
            // feed available to us at all.
            if (!m_vFrameReadersCPU[nCamera - 1].IsActive())
            {
                // Take a persistent read handle for this feed.
                m_vFrameReadersCPU[nCamera - 1] = pBasicCamera->GetFrameReader();
            }
            // Setup the video encoder if needed.
            if (!m_vCameraWriters[nCamera - 1]->IsOpen())
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
                                  "Unable to create the video output directory: {} for camera {}",
                                  szFilePath.string(),
                                  pBasicCamera->GetCameraLocation());
                    }
                }

                // Construct the full output path.
                std::filesystem::path szFullOutputPath = szFilePath / szFilenameWithExtension;

                // Open writer.
                bool bWriterOpened = m_vCameraWriters[nCamera - 1]->Open(szFullOutputPath.string(),
                                                                         pBasicCamera->GetPropResolution(),
                                                                         constants::RECORDER_FPS,
                                                                         constants::RECORDER_X264_PRESET,
                                                                         constants::RECORDER_ENCODER_THREADS);

                // Check writer opened status.
                if (!bWriterOpened)
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger,
                                "RecordingHandler: Failed to open the video encoder for basic camera at path/index {}",
                                pBasicCamera->GetCameraLocation());
                }
            }
        }
        else
        {
            // Set recording toggle.
            m_vRecordingToggles[nCamera - 1] = false;
            // Drop our demand so the camera stops doing work nobody is recording.
            m_vFrameReadersCPU[nCamera - 1].Release();
        }
    }

    // Get index offset so we don't overwrite BasicCam pointers and booleans.
    int nIndexOffset = int(CameraHandler::BasicCamName::BASICCAM_END) - 2;
    // Loop through all ZED cameras from the CameraHandler.
    for (int nCamera = int(CameraHandler::ZEDCamName::ZEDCAM_START) + 1; nCamera != int(CameraHandler::ZEDCamName::ZEDCAM_END); ++nCamera)
    {
        // Get pointer to camera.
        std::shared_ptr<ZEDCamera> pZEDCamera = globals::g_pCameraHandler->GetZED(static_cast<CameraHandler::ZEDCamName>(nCamera));
        // Store camera pointer in vector so we can get images later.
        m_vZEDCameras[nCamera + nIndexOffset] = pZEDCamera;

        // Check if recording for this camera is enabled.
        if (pZEDCamera->GetEnableRecordingFlag() && pZEDCamera->GetCameraIsOpen())
        {
            // Set recording toggle.
            m_vRecordingToggles[nCamera + nIndexOffset] = true;
            // Take a read handle on this camera's frames if we have not already, on whichever
            // memory channel the camera is configured for. The camera only retrieves and publishes
            // frames while a Reader is alive, so this is what makes the feed available to us.
            if (!m_vFrameReadersCPU[nCamera + nIndexOffset].IsActive() && !m_vFrameReadersGPU[nCamera + nIndexOffset].IsActive())
            {
                // Take a persistent read handle on the matching memory channel.
                if (pZEDCamera->GetUsingGPUMem())
                {
                    // GPU memory mode: read the GPU frame channel.
                    m_vFrameReadersGPU[nCamera + nIndexOffset] = pZEDCamera->GetFrameGPUReader();
                }
                else
                {
                    // CPU memory mode: read the CPU frame channel.
                    m_vFrameReadersCPU[nCamera + nIndexOffset] = pZEDCamera->GetFrameCPUReader();
                }
            }
            // Setup the video encoder if needed.
            if (!m_vCameraWriters[nCamera + nIndexOffset]->IsOpen())
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
                bool bWriterOpened = m_vCameraWriters[nCamera + nIndexOffset]->Open(szFullOutputPath.string(),
                                                                                    pZEDCamera->GetPropResolution(),
                                                                                    constants::RECORDER_FPS,
                                                                                    constants::RECORDER_X264_PRESET,
                                                                                    constants::RECORDER_ENCODER_THREADS);

                // Check writer opened status.
                if (!bWriterOpened)
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger,
                                "RecordingHandler: Failed to open the video encoder for ZED camera with serial {}",
                                pZEDCamera->GetCameraSerial());
                }
            }
        }
        else
        {
            // Set recording toggle.
            m_vRecordingToggles[nCamera + nIndexOffset] = false;
            // Drop our demand so the camera stops doing work nobody is recording.
            m_vFrameReadersCPU[nCamera + nIndexOffset].Release();
            m_vFrameReadersGPU[nCamera + nIndexOffset].Release();
        }
    }
}

/******************************************************************************
 * @brief This method is used internally by the RecordingHandler to read and write the
 *      newest published frame from each camera stored in the member variable vectors.
 *
 *      Each read is a lock-free load of the camera's newest immutable snapshot, so this
 *      never blocks on a camera's loop and no camera does any per-recorder copy work.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-01
 ******************************************************************************/
void RecordingHandler::RequestAndWriteCameraFrames()
{
    ZoneScopedC(tracy::Color::DarkOrange2);
    // Loop through total number of cameras, read the newest frame, and write it.
    for (int nIter = 0; nIter < m_nTotalVideoFeeds; ++nIter)
    {
        // Check if recording for the camera at this index is enabled.
        if (!m_vRecordingToggles[nIter])
        {
            // Nothing to record for this feed.
            continue;
        }

        // Check if the camera at the current index is a BasicCam or ZEDCam.
        if (m_vBasicCameras[nIter] != nullptr)
        {
            // Load the newest published frame snapshot once into a local.
            pubsub::SharedSnapshot<cv::Mat> pSnapshot = m_vFrameReadersCPU[nIter].Get();
            // Nothing has been published yet.
            if (pSnapshot == nullptr)
            {
                // Skip this feed for this iteration.
                continue;
            }
            // Encode straight from the immutable snapshot. The encoder only reads it.
            this->WriteFrameToVideo(nIter, pSnapshot->tData);
        }
        else if (m_vZEDCameras[nIter] != nullptr)
        {
            // Check if the camera is setup to use CPU or GPU mats.
            if (m_vZEDCameras[nIter]->GetUsingGPUMem())
            {
                // Load the newest published GPU frame snapshot once into a local.
                pubsub::SharedSnapshot<cv::cuda::GpuMat> pSnapshot = m_vFrameReadersGPU[nIter].Get();
                // Nothing has been published yet.
                if (pSnapshot == nullptr)
                {
                    // Skip this feed for this iteration.
                    continue;
                }
                // Download from GPU memory. Done here, on this thread, off the camera's critical path.
                pSnapshot->tData.download(m_vFrames[nIter]);
                // Encode the downloaded frame.
                this->WriteFrameToVideo(nIter, m_vFrames[nIter]);
            }
            else
            {
                // Load the newest published CPU frame snapshot once into a local.
                pubsub::SharedSnapshot<cv::Mat> pSnapshot = m_vFrameReadersCPU[nIter].Get();
                // Nothing has been published yet.
                if (pSnapshot == nullptr)
                {
                    // Skip this feed for this iteration.
                    continue;
                }
                // Encode straight from the immutable snapshot. The encoder only reads it.
                this->WriteFrameToVideo(nIter, pSnapshot->tData);
            }
        }
    }
}

/******************************************************************************
 * @brief Encode a frame into the video file of the given feed. Shared by the camera,
 *      tag detector, and object detector recording paths.
 *
 * @param nFeedIndex - The index of the video feed to write to.
 * @param cvFrame - The frame to write. 1 (gray), 3 (BGR) or 4 (BGRA) channels. It is only read,
 *                  so a published snapshot can be passed directly.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-26
 ******************************************************************************/
void RecordingHandler::WriteFrameToVideo(const int nFeedIndex, const cv::Mat& cvFrame)
{
    // Nothing to write for an empty frame or a closed encoder.
    if (cvFrame.empty() || !m_vCameraWriters[nFeedIndex]->IsOpen())
    {
        // Skip this write.
        return;
    }

    // The encoder converts gray, BGR and BGRA straight to YUV, so no conversion or copy is needed here.
    m_vCameraWriters[nFeedIndex]->Write(cvFrame);
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
    ZoneScopedC(tracy::Color::DarkOrange2);
    // Loop through all Basic cameras from the CameraHandler.
    for (int nDetector = int(TagDetectionHandler::TagDetectors::TAGDETECTOR_START) + 1; nDetector != int(TagDetectionHandler::TagDetectors::TAGDETECTOR_END); ++nDetector)
    {
        // Get pointer to camera.
        std::shared_ptr<TagDetector> pTagDetector = globals::g_pTagDetectionHandler->GetTagDetector(static_cast<TagDetectionHandler::TagDetectors>(nDetector));
        // Store camera pointer in vector so we can get images later.
        m_vTagDetectors[nDetector - 1] = pTagDetector;

        // Check if recording for this camera is enabled.
        if (pTagDetector->GetEnableRecordingFlag() && pTagDetector->GetIsReady())
        {
            // Set recording toggle.
            m_vRecordingToggles[nDetector - 1] = true;
            // Take a read handle on this detector's overlay frames if we have not already. The
            // detector only clones and publishes overlays while a Reader is alive.
            if (!m_vFrameReadersCPU[nDetector - 1].IsActive())
            {
                // Take a persistent read handle for this feed.
                m_vFrameReadersCPU[nDetector - 1] = pTagDetector->GetDetectionOverlayReader();
            }
            // Setup the video encoder if needed.
            if (!m_vCameraWriters[nDetector - 1]->IsOpen())
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
                                  "Unable to create the video output directory: {} for tag detector {}",
                                  szFilePath.string(),
                                  pTagDetector->GetCameraName());
                    }
                }

                // Construct the full output path.
                std::filesystem::path szFullOutputPath = szFilePath / szFilenameWithExtension;

                // Open writer.
                bool bWriterOpened = m_vCameraWriters[nDetector - 1]->Open(szFullOutputPath.string(),
                                                                           pTagDetector->GetProcessFrameResolution(),
                                                                           constants::RECORDER_FPS,
                                                                           constants::RECORDER_X264_PRESET,
                                                                           constants::RECORDER_ENCODER_THREADS);

                // Check writer opened status.
                if (!bWriterOpened)
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger,
                                "RecordingHandler: Failed to open the video encoder for tag detector using camera {}",
                                pTagDetector->GetCameraName());
                }
            }
        }
        else
        {
            // Set recording toggle.
            m_vRecordingToggles[nDetector - 1] = false;
            // Drop our demand so the detector stops cloning overlays nobody is recording.
            m_vFrameReadersCPU[nDetector - 1].Release();
        }
    }
}

/******************************************************************************
 * @brief This method is used internally by the RecordingHandler to read and write the
 *      newest published detection-overlay frame from each TagDetector stored in the
 *      member variable vectors. Each read is a lock-free load of the detector's newest
 *      immutable snapshot and never blocks on the detector's loop.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-01
 ******************************************************************************/
void RecordingHandler::RequestAndWriteTagDetectorFrames()
{
    ZoneScopedC(tracy::Color::DarkOrange2);
    // Loop through total number of detectors, read the newest overlay frame, and write it.
    for (int nIter = 0; nIter < m_nTotalVideoFeeds; ++nIter)
    {
        // Check if recording for the detector at this index is enabled and the detector exists.
        if (!m_vRecordingToggles[nIter] || m_vTagDetectors[nIter] == nullptr)
        {
            // Nothing to record for this feed.
            continue;
        }

        // Load the newest published overlay snapshot once into a local.
        pubsub::SharedSnapshot<cv::Mat> pSnapshot = m_vFrameReadersCPU[nIter].Get();
        // Nothing has been published yet.
        if (pSnapshot == nullptr)
        {
            // Skip this feed for this iteration.
            continue;
        }
        // Encode straight from the immutable snapshot. The encoder only reads it.
        this->WriteFrameToVideo(nIter, pSnapshot->tData);
    }
}

/******************************************************************************
 * @brief This method is used internally by the class to update the number of ObjectDetectors
 *      that have recording enabled from the camera handler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-31
 ******************************************************************************/
void RecordingHandler::UpdateRecordableObjectDetectors()
{
    ZoneScopedC(tracy::Color::DarkOrange2);
    // Loop through all Basic cameras from the CameraHandler.
    for (int nDetector = int(ObjectDetectionHandler::ObjectDetectors::OBJECTDETECTOR_START) + 1;
         nDetector != int(ObjectDetectionHandler::ObjectDetectors::OBJECTDETECTOR_END);
         ++nDetector)
    {
        // Get pointer to camera.
        std::shared_ptr<ObjectDetector> pObjectDetector =
            globals::g_pObjectDetectionHandler->GetObjectDetector(static_cast<ObjectDetectionHandler::ObjectDetectors>(nDetector));
        // Store camera pointer in vector so we can get images later.
        m_vObjectDetectors[nDetector - 1] = pObjectDetector;

        // Check if recording for this camera is enabled.
        if (pObjectDetector->GetEnableRecordingFlag() && pObjectDetector->GetIsReady())
        {
            // Set recording toggle.
            m_vRecordingToggles[nDetector - 1] = true;
            // Take a read handle on this detector's overlay frames if we have not already. The
            // detector only clones and publishes overlays while a Reader is alive.
            if (!m_vFrameReadersCPU[nDetector - 1].IsActive())
            {
                // Take a persistent read handle for this feed.
                m_vFrameReadersCPU[nDetector - 1] = pObjectDetector->GetDetectionOverlayReader();
            }
            // Setup the video encoder if needed.
            if (!m_vCameraWriters[nDetector - 1]->IsOpen())
            {
                // Assemble filepath string.
                std::filesystem::path szFilePath;
                std::filesystem::path szFilenameWithExtension;
                szFilePath = constants::LOGGING_OUTPUT_PATH_ABSOLUTE;                     // Main location for all recordings.
                szFilePath += logging::g_szProgramStartTimeString + "/objectdetector";    // Folder for each program run.
                szFilenameWithExtension = pObjectDetector->GetCameraName() + ".mkv";      // Folder for each camera index or name.

                // Check if directory exists.
                if (!std::filesystem::exists(szFilePath))
                {
                    // Create directory.
                    if (!std::filesystem::create_directories(szFilePath))
                    {
                        // Submit logger message.
                        LOG_ERROR(logging::g_qSharedLogger,
                                  "Unable to create the video output directory: {} for tag detector {}",
                                  szFilePath.string(),
                                  pObjectDetector->GetCameraName());
                    }
                }

                // Construct the full output path.
                std::filesystem::path szFullOutputPath = szFilePath / szFilenameWithExtension;

                // Open writer.
                bool bWriterOpened = m_vCameraWriters[nDetector - 1]->Open(szFullOutputPath.string(),
                                                                           pObjectDetector->GetProcessFrameResolution(),
                                                                           constants::RECORDER_FPS,
                                                                           constants::RECORDER_X264_PRESET,
                                                                           constants::RECORDER_ENCODER_THREADS);

                // Check writer opened status.
                if (!bWriterOpened)
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger,
                                "RecordingHandler: Failed to open the video encoder for tag detector using camera {}",
                                pObjectDetector->GetCameraName());
                }
            }
        }
        else
        {
            // Set recording toggle.
            m_vRecordingToggles[nDetector - 1] = false;
            // Drop our demand so the detector stops cloning overlays nobody is recording.
            m_vFrameReadersCPU[nDetector - 1].Release();
        }
    }
}

/******************************************************************************
 * @brief This method is used internally by the RecordingHandler to read and write the
 *      newest published detection-overlay frame from each ObjectDetector stored in the
 *      member variable vectors. Each read is a lock-free load of the detector's newest
 *      immutable snapshot and never blocks on the detector's loop.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-01
 ******************************************************************************/
void RecordingHandler::RequestAndWriteObjectDetectorFrames()
{
    ZoneScopedC(tracy::Color::DarkOrange2);
    // Loop through total number of detectors, read the newest overlay frame, and write it.
    for (int nIter = 0; nIter < m_nTotalVideoFeeds; ++nIter)
    {
        // Check if recording for the detector at this index is enabled and the detector exists.
        if (!m_vRecordingToggles[nIter] || m_vObjectDetectors[nIter] == nullptr)
        {
            // Nothing to record for this feed.
            continue;
        }

        // Load the newest published overlay snapshot once into a local.
        pubsub::SharedSnapshot<cv::Mat> pSnapshot = m_vFrameReadersCPU[nIter].Get();
        // Nothing has been published yet.
        if (pSnapshot == nullptr)
        {
            // Skip this feed for this iteration.
            continue;
        }
        // Encode straight from the immutable snapshot. The encoder only reads it.
        this->WriteFrameToVideo(nIter, pSnapshot->tData);
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
