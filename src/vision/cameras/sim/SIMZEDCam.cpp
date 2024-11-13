/******************************************************************************
 * @brief Implements the SIMZEDCam class.
 *
 * @file SIMZEDCam.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "SIMZEDCam.h"

#include "../../../AutonomyConstants.h"
#include "../../../AutonomyLogging.h"
#include "../../../util/TranscodeOperations.hpp"

/// \cond
#include <nlohmann/json.hpp>

/// \endcond

/******************************************************************************
 * @brief Construct a new SIM Cam:: SIM Cam object.
 *
 * @param szCameraPath - The file path to the camera hardware.
 * @param nPropResolutionX - X res of camera.
 * @param nPropResolutionY - Y res of camera.
 * @param nPropFramesPerSecond - FPS camera is running at.
 * @param ePropPixelFormat - The pixel layout/format of the image.
 * @param dPropHorizontalFOV - The horizontal field of view.
 * @param dPropVerticalFOV - The vertical field of view.
 * @param bEnableRecordingFlag - Whether or not this camera should be recorded.
 * @param nNumFrameRetrievalThreads - The number of threads to use for frame queueing and copying.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
SIMZEDCam::SIMZEDCam(const std::string szCameraPath,
                     const int nPropResolutionX,
                     const int nPropResolutionY,
                     const int nPropFramesPerSecond,
                     const PIXEL_FORMATS ePropPixelFormat,
                     const double dPropHorizontalFOV,
                     const double dPropVerticalFOV,
                     const bool bEnableRecordingFlag,
                     const int nNumFrameRetrievalThreads) :
    Camera(nPropResolutionX, nPropResolutionY, nPropFramesPerSecond, ePropPixelFormat, dPropHorizontalFOV, dPropVerticalFOV, bEnableRecordingFlag)
{
    // Assign member variables.
    m_szCameraPath              = szCameraPath;
    m_nNumFrameRetrievalThreads = nNumFrameRetrievalThreads;

    // Construct the WebRTC peer connection and data channel for receiving data from the simulator.
    rtc::WebSocket::Configuration rtcWebSocketConfig;
    rtc::Configuration rtcPeerConnectionConfig;
    m_pWebSocket      = std::make_shared<rtc::WebSocket>(rtcWebSocketConfig);
    m_pPeerConnection = std::make_shared<rtc::PeerConnection>(rtcPeerConnectionConfig);
    m_pDataChannel    = m_pPeerConnection->createDataChannel("data_channel");

    // Attempt to connect to the signalling server.
    this->ConnectToSignallingServer(m_szCameraPath);

    // Set max FPS of the ThreadedContinuousCode method.
    this->SetMainThreadIPSLimit(nPropFramesPerSecond);
}

/******************************************************************************
 * @brief Destroy the SIM Cam:: SIM Cam object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
SIMZEDCam::~SIMZEDCam()
{
    // Stop threaded code.
    this->RequestStop();
    this->Join();
}

/******************************************************************************
 * @brief The code inside this private method runs in a separate thread, but still
 *      has access to this*. This method continuously get new frames from the OpenCV
 *      VideoCapture object and stores it in a member variable. Then a thread pool is
 *      started and joined once per iteration to mass copy the frames and/or measure
 *      to any other thread waiting in the queues.
 *
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
void SIMZEDCam::ThreadedContinuousCode()
{
    // Check if camera is NOT open.
    // if (!m_cvCamera.isOpened())
    // {
    //     // Shutdown threads for this SIMZEDCam.
    //     this->RequestStop();

    //     // Submit logger message.
    //     LOG_CRITICAL(logging::g_qSharedLogger,
    //                  "Camera start was attempted for camera at {}/{}, but camera never properly opened or it has become disconnected!",
    //                  m_nCameraIndex,
    //                  m_szCameraPath);
    // }
    // else
    // {
    //     // TODO: PUT CODE HERE FOR GETTING FRAMES AND DATA FROM SIMULATOR.
    // }

    // Acquire a shared_lock on the frame copy queue.
    std::shared_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Check if the frame copy queue is empty.
    if (!m_qFrameCopySchedule.empty())
    {
        // Start the thread pool to store multiple copies of the sl::Mat into the given cv::Mats.
        this->RunDetachedPool(m_qFrameCopySchedule.size(), m_nNumFrameRetrievalThreads);
        // Wait for thread pool to finish.
        this->JoinPool();
        // Release lock on frame copy queue.
        lkSchedulers.unlock();
    }
}

/******************************************************************************
 * @brief This method holds the code that is ran in the thread pool started by
 *      the ThreadedLinearCode() method. It copies the data from the different
 *      data objects to references of the same type stored in a vector queued up by the
 *      Grab methods.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
void SIMZEDCam::PooledLinearCode()
{
    /////////////////////////////
    //  Frame queue.
    /////////////////////////////

    // Acquire mutex for getting frames out of the queue.
    std::unique_lock<std::shared_mutex> lkFrameQueue(m_muFrameCopyMutex);
    // Check if the queue is empty.
    if (!m_qFrameCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::FrameFetchContainer<cv::Mat> stContainer = m_qFrameCopySchedule.front();
        // Pop out of queue.
        m_qFrameCopySchedule.pop();
        // Release lock.
        lkFrameQueue.unlock();

        // Determine which frame should be copied.
        switch (stContainer.eFrameType)
        {
            case PIXEL_FORMATS::eBGRA: *(stContainer.pFrame) = m_cvFrame; break;
            case PIXEL_FORMATS::eDepthMeasure: *(stContainer.pFrame) = m_cvDepthMeasure; break;
            case PIXEL_FORMATS::eDepthImage: *(stContainer.pFrame) = m_cvDepthImage; break;
            case PIXEL_FORMATS::eXYZ: *(stContainer.pFrame) = m_cvPointCloud; break;
            default: *(stContainer.pFrame) = m_cvFrame; break;
        }

        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedFrameStatus->set_value(true);
    }
}

/******************************************************************************
 * @brief Connected to the Unreal Engine 5 hosted Signalling Server for WebRTC negotiation.
 *
 * @param szSignallingServerURL - The full URL of the signalling server. Should be in the format of "ws://<IP>:<PORT>"
 * @return true - Successfully connected to the signalling server.
 * @return false - Failed to connect to the signalling server.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-11-11
 ******************************************************************************/
bool SIMZEDCam::ConnectToSignallingServer(const std::string& szSignallingServerURL)
{
    // Connect to the signalling server via a websocket to handle WebRTC negotiation and signalling.
    m_pWebSocket->open(szSignallingServerURL);

    /////////////////////////////////////////////////////////////////////
    // Set some callbacks on important events for the websocket.
    /////////////////////////////////////////////////////////////////////

    // WebSocket has been opened.
    m_pWebSocket->onOpen(
        [this]()
        {
            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Connected to the signalling server via {}. Sending local description to signalling server...", m_szCameraPath);

            // Get local description.
            std::optional<rtc::Description> rtcTest = m_pPeerConnection->localDescription();
            // Send local description to signalling server.
            nlohmann::json jsnMessage;
            jsnMessage["type"] = rtcTest->typeString();
            jsnMessage["sdp"]  = rtcTest->generateSdp();
            m_pWebSocket->send(jsnMessage.dump());
        });

    // WebSocket has been closed.
    m_pWebSocket->onClosed(
        [this]()
        {
            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Disconnected from the signalling server.");
        });

    // Handling signalling server messages. (offer/answer/ICE candidate)
    m_pWebSocket->onMessage(
        [this](std::variant<rtc::binary, rtc::string> szMessage)
        {
            // Create instance variables.
            nlohmann::json jsnMessage;

            // Check if data is of type rtc::string.
            if (std::holds_alternative<rtc::string>(szMessage))
            {
                // Retrieve the string message
                std::string message = std::get<rtc::string>(szMessage);

                // Parse the JSON message from the signaling server.
                jsnMessage = nlohmann::json::parse(message);
                LOG_INFO(logging::g_qSharedLogger, "Received message from signalling server: {}", message);
            }
            else if (std::holds_alternative<rtc::binary>(szMessage))
            {
                // Retrieve the binary message.
                rtc::binary rtcBinaryData = std::get<rtc::binary>(szMessage);
                // Print length of binary data.
                LOG_INFO(logging::g_qSharedLogger, "Received binary data of length: {}", rtcBinaryData.size());

                // Convert the binary data to a string.
                std::string szBinaryDataStr(reinterpret_cast<const char*>(rtcBinaryData.data()), rtcBinaryData.size());
                // Print the binary data as a string.
                LOG_INFO(logging::g_qSharedLogger, "Received binary data: {}", szBinaryDataStr);
                // Parse the binary data as JSON.
                jsnMessage = nlohmann::json::parse(szBinaryDataStr);

                // Process the binary data (e.g., decode video frame)
                // Example: Decode the binary data as a video frame
                // decodeVideoFrame(rtcBinaryData);
            }
            else
            {
                LOG_ERROR(logging::g_qSharedLogger, "Received unknown message type from signalling server");
            }

            if (jsnMessage.contains("type"))
            {
                std::string type = jsnMessage["type"];
                if (type == "answer")
                {
                    // Get the SDP offer and set it as the remote description
                    std::string sdp = jsnMessage["sdp"];
                    m_pPeerConnection->setRemoteDescription(rtc::Description(sdp, "answer"));
                }
                else if (type == "iceCandidate")
                {
                    // Handle ICE candidate
                    nlohmann::json jsnCandidate = jsnMessage["candidate"];
                    std::string szCandidateStr  = jsnCandidate["candidate"];
                    rtc::Candidate rtcCandidate = rtc::Candidate(szCandidateStr);
                    m_pPeerConnection->addRemoteCandidate(rtcCandidate);
                }
                else if (type == "config")
                {
                    // Do nothing for config.
                }
            }
        });

    m_pWebSocket->onError(
        [this](const std::string& szError)
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "Error occurred on WebSocket: {}", szError);
        });

    m_pPeerConnection->onLocalDescription(
        [this](rtc::Description rtcDescription)
        {
            // Send the local description to the signalling server
            nlohmann::json jsnMessage;
            jsnMessage["type"] = rtcDescription.typeString();
            jsnMessage["sdp"]  = rtcDescription.generateSdp();
            m_pWebSocket->send(jsnMessage.dump());
            LOG_WARNING(logging::g_qSharedLogger, "Sending local description to signalling server");
        });

    m_pDataChannel->onOpen([this]() { LOG_INFO(logging::g_qSharedLogger, "Data channel opened."); });

    m_pDataChannel->onMessage(
        [this](std::variant<rtc::binary, rtc::string> szMessage)
        {
            // Check if the message is a string.
            if (std::holds_alternative<rtc::string>(szMessage))
            {
                // Retrieve the string message.
                std::string message = std::get<rtc::string>(szMessage);
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "Received message from data channel: {}", message);
            }
            else if (std::holds_alternative<rtc::binary>(szMessage))
            {
                // Retrieve the binary message.
                rtc::binary rtcBinaryData = std::get<rtc::binary>(szMessage);
                // Print length of binary data.
                LOG_INFO(logging::g_qSharedLogger, "Received binary data of length: {}", rtcBinaryData.size());

                // Convert the binary data to a string.
                std::string szBinaryDataStr(reinterpret_cast<const char*>(rtcBinaryData.data()), rtcBinaryData.size());
                // Print the binary data as a string.
                LOG_INFO(logging::g_qSharedLogger, "Received binary data: {}", transops::DecodeUTF8EncodedString(szBinaryDataStr));
            }
            else
            {
                LOG_ERROR(logging::g_qSharedLogger, "Received unknown message type from data channel");
            }
        });

    return true;
}

/******************************************************************************
 * @brief Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
 *      Remember, this code will be ran in whatever, class/thread calls it.
 *
 * @param cvFrame - A reference to the cv::Mat to store the frame in.
 * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
std::future<bool> SIMZEDCam::RequestFrameCopy(cv::Mat& cvFrame)
{
    // Assemble the FrameFetchContainer.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvFrame, m_ePropPixelFormat);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Requests a depth measure or image from the camera.
 *      Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
 *      This image has the same shape as a grayscale image, but the values represent the depth in
 *      MILLIMETERS. The ZEDSDK will always return this measure in MILLIMETERS.
 *
 * @param cvDepth - A reference to the cv::Mat to copy the depth frame to.
 * @param bRetrieveMeasure - False to get depth IMAGE instead of MEASURE. Do not use the 8-bit grayscale depth image
 *                  purposes other than displaying depth.
 * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
std::future<bool> SIMZEDCam::RequestDepthCopy(cv::Mat& cvDepth, const bool bRetrieveMeasure)
{
    // Create instance variables.
    PIXEL_FORMATS eFrameType;

    // Check if the container should be set to retrieve an image or a measure.
    bRetrieveMeasure ? eFrameType = PIXEL_FORMATS::eDepthMeasure : eFrameType = PIXEL_FORMATS::eDepthImage;
    // Assemble container.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvDepth, eFrameType);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkSchedulers.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Requests a point cloud image from the camera. This image has the same resolution as a normal
 *      image but with three XYZ values replacing the old color values in the 3rd dimension.
 *      The units and sign of the XYZ values are determined by ZED_MEASURE_UNITS and ZED_COORD_SYSTEM
 *      constants set in AutonomyConstants.h.
 *
 *      Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
 *
 * @param cvPointCloud - A reference to the cv::Mat to copy the point cloud frame to.
 * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
std::future<bool> SIMZEDCam::RequestPointCloudCopy(cv::Mat& cvPointCloud)
{
    // Assemble the FrameFetchContainer.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvPointCloud, PIXEL_FORMATS::eXYZ);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkSchedulers.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Accessor for the camera open status.
 *
 * @return true - The camera has been successfully opened.
 * @return false - The camera has not been successfully opened.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
bool SIMZEDCam::GetCameraIsOpen()
{
    // Get camera status from OpenCV.
    // return m_cvCamera.isOpened();
    // TODO: Put code here for determining if the stream is open.
    return true;
}

/******************************************************************************
 * @brief Accessor for the cameras path or video index.
 *
 * @return std::string - The path or index of the camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
std::string SIMZEDCam::GetCameraLocation() const
{
    // TODO: Put code here for determining the stream name from WebRTC connection.
    return "PLACEHOLDER";
    // // Check if camera location is a hardware path or video index.
    // if (m_bCameraIsConnectedOnVideoIndex)
    // {
    //     // If video index, return index integer.
    //     return std::to_string(m_nCameraIndex);
    // }
    // else
    // {
    //     // If video path, return path string.
    //     return m_szCameraPath;
    // }
}
