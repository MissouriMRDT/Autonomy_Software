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
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/frame.h>
#include <libavutil/imgutils.h>
#include <libavutil/mem.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
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
        // Acquire shared lock on the WebRTC mutex, so that the WebRTC connection doesn't try to write to the Mats while they are being copied in the thread pool.
        std::shared_lock<std::shared_mutex> lkWebRTC(m_muWebRTCCopyMutex);

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

            // Send local config to signalling server.
            std::string szLocalConfig = "{\"type\":\"config\",\"peerConnectionOptions\":{\"iceServers\":[]}}";
            m_pWebSocket->send(nlohmann::json::parse(szLocalConfig).dump());
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
        [this](std::variant<rtc::binary, rtc::string> rtcMessage)
        {
            try
            {
                // Create instance variables.
                nlohmann::json jsnMessage;

                // Check if data is of type rtc::string.
                if (std::holds_alternative<rtc::string>(rtcMessage))
                {
                    // Retrieve the string message
                    std::string szMessage = std::get<rtc::string>(rtcMessage);

                    // Parse the JSON message from the signaling server.
                    jsnMessage = nlohmann::json::parse(szMessage);
                    LOG_INFO(logging::g_qSharedLogger, "Received message from signalling server: {}", szMessage);
                }
                else if (std::holds_alternative<rtc::binary>(rtcMessage))
                {
                    // Retrieve the binary message.
                    rtc::binary rtcBinaryData = std::get<rtc::binary>(rtcMessage);
                    // Print length of binary data.
                    LOG_INFO(logging::g_qSharedLogger, "Received binary data of length: {}", rtcBinaryData.size());

                    // Convert the binary data to a string.
                    std::string szBinaryDataStr(reinterpret_cast<const char*>(rtcBinaryData.data()), rtcBinaryData.size());
                    // Print the binary data as a string.
                    LOG_INFO(logging::g_qSharedLogger, "Received binary data: {}", szBinaryDataStr);
                    // Parse the binary data as JSON.
                    jsnMessage = nlohmann::json::parse(szBinaryDataStr);
                }
                else
                {
                    LOG_ERROR(logging::g_qSharedLogger, "Received unknown message type from signalling server");
                }

                // Check if the message contains a type.
                if (jsnMessage.contains("type"))
                {
                    std::string szType = jsnMessage["type"];
                    // If the message from the server is an offer, set the remote description offer.
                    if (szType == "offer")
                    {
                        // Get the SDP offer and set it as the remote description.
                        std::string sdp = jsnMessage["sdp"];
                        m_pPeerConnection->setRemoteDescription(rtc::Description(sdp, "offer"));
                    }
                    // If the message from the server is an answer, set the remote description answer.
                    else if (szType == "answer")
                    {
                        // Get the SDP answer and set it as the remote description.
                        std::string sdp = jsnMessage["sdp"];
                        // m_pPeerConnection->setRemoteDescription(rtc::Description(sdp, "answer"));
                    }
                    // If the message from the server is advertising an ICE candidate, add it to the peer connection.
                    else if (szType == "iceCandidate")
                    {
                        // Handle ICE candidate
                        nlohmann::json jsnCandidate = jsnMessage["candidate"];
                        std::string szCandidateStr  = jsnCandidate["candidate"];
                        rtc::Candidate rtcCandidate = rtc::Candidate(szCandidateStr);
                        m_pPeerConnection->addRemoteCandidate(rtcCandidate);
                    }
                    else if (szType == "config")
                    {
                        // Do nothing for config.
                    }
                }
            }
            catch (const std::exception& e)
            {
                // Submit logger message.
                LOG_ERROR(logging::g_qSharedLogger, "Error occurred while negotiating with the Signalling Server: {}", e.what());
            }
        });

    m_pWebSocket->onError(
        [this](const std::string& szError)
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "Error occurred on WebSocket: {}", szError);
        });

    /////////////////////////////////////////////////////////////////////
    // Set some callbacks on important events for the peer connection.
    /////////////////////////////////////////////////////////////////////

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

    m_pPeerConnection->onGatheringStateChange(
        [this](rtc::PeerConnection::GatheringState eGatheringState)
        {
            // Switch to translate the state to a string.
            switch (eGatheringState)
            {
                case rtc::PeerConnection::GatheringState::Complete: LOG_INFO(logging::g_qSharedLogger, "PeerConnection ICE gathering state changed to: Complete"); break;

                case rtc::PeerConnection::GatheringState::InProgress:
                    LOG_INFO(logging::g_qSharedLogger, "PeerConnection ICE gathering state changed to: InProgress");
                    break;
                case rtc::PeerConnection::GatheringState::New: LOG_INFO(logging::g_qSharedLogger, "PeerConnection ICE gathering state changed to: New"); break;
                default: LOG_INFO(logging::g_qSharedLogger, "Peer connection ICE gathering state changed to: Unknown"); break;
            }
        });

    m_pPeerConnection->onIceStateChange(
        [this](rtc::PeerConnection::IceState eIceState)
        {
            // Switch to translate the state to a string.
            switch (eIceState)
            {
                case rtc::PeerConnection::IceState::Checking: LOG_INFO(logging::g_qSharedLogger, "PeerConnection ICE state changed to: Checking"); break;
                case rtc::PeerConnection::IceState::Closed: LOG_INFO(logging::g_qSharedLogger, "PeerConnection ICE state changed to: Closed"); break;
                case rtc::PeerConnection::IceState::Completed: LOG_INFO(logging::g_qSharedLogger, "PeerConnection ICE state changed to: Completed"); break;
                case rtc::PeerConnection::IceState::Connected: LOG_INFO(logging::g_qSharedLogger, "PeerConnection ICE state changed to: Connected"); break;
                case rtc::PeerConnection::IceState::Disconnected: LOG_INFO(logging::g_qSharedLogger, "PeerConnection ICE state changed to: Disconnected"); break;
                case rtc::PeerConnection::IceState::Failed: LOG_INFO(logging::g_qSharedLogger, "PeerConnection ICE state changed to: Failed"); break;
                case rtc::PeerConnection::IceState::New: LOG_INFO(logging::g_qSharedLogger, "PeerConnection ICE state changed to: New"); break;
                default: LOG_INFO(logging::g_qSharedLogger, "Peer connection ICE state changed to: Unknown"); break;
            }
        });
    m_pPeerConnection->onSignalingStateChange(
        [this](rtc::PeerConnection::SignalingState eSignalingState)
        {
            // Switch to translate the state to a string.
            switch (eSignalingState)
            {
                case rtc::PeerConnection::SignalingState::HaveLocalOffer:
                {
                    // Get the local description.
                    std::optional<rtc::Description> rtcDescription = m_pPeerConnection->localDescription();
                    // Send the local description to the signalling server
                    nlohmann::json jsnMessage;
                    jsnMessage["type"] = rtcDescription->typeString();
                    jsnMessage["sdp"]  = rtcDescription->generateSdp();
                    m_pWebSocket->send(jsnMessage.dump());
                    LOG_WARNING(logging::g_qSharedLogger, "Sending local description to signalling server");
                    LOG_INFO(logging::g_qSharedLogger, "PeerConnection signaling state changed to: HaveLocalOffer");
                    break;
                }
                case rtc::PeerConnection::SignalingState::HaveLocalPranswer:
                    LOG_INFO(logging::g_qSharedLogger, "PeerConnection signaling state changed to: HaveLocalPranswer");
                    break;
                case rtc::PeerConnection::SignalingState::HaveRemoteOffer:
                    LOG_INFO(logging::g_qSharedLogger, "PeerConnection signaling state changed to: HaveRemoteOffer");
                    break;
                case rtc::PeerConnection::SignalingState::HaveRemotePranswer:
                    LOG_INFO(logging::g_qSharedLogger, "PeerConnection signaling state changed to: HaveRemotePrAnswer");
                    break;
                case rtc::PeerConnection::SignalingState::Stable: LOG_INFO(logging::g_qSharedLogger, "PeerConnection signaling state changed to: Stable"); break;
                default: LOG_INFO(logging::g_qSharedLogger, "Peer connection signaling state changed to: Unknown"); break;
            }
        });

    m_pPeerConnection->onStateChange(
        [this](rtc::PeerConnection::State eState)
        {
            // Switch to translate the state to a string.
            switch (eState)
            {
                case rtc::PeerConnection::State::Closed: LOG_INFO(logging::g_qSharedLogger, "Peer connection state changed to: Closed"); break;
                case rtc::PeerConnection::State::Connected: LOG_INFO(logging::g_qSharedLogger, "Peer connection state changed to: Connected"); break;
                case rtc::PeerConnection::State::Connecting: LOG_INFO(logging::g_qSharedLogger, "Peer connection state changed to: Connecting"); break;
                case rtc::PeerConnection::State::Disconnected: LOG_INFO(logging::g_qSharedLogger, "Peer connection state changed to: Disconnected"); break;
                case rtc::PeerConnection::State::Failed: LOG_INFO(logging::g_qSharedLogger, "Peer connection state changed to: Failed"); break;
                case rtc::PeerConnection::State::New: LOG_INFO(logging::g_qSharedLogger, "Peer connection state changed to: New"); break;
                default: LOG_INFO(logging::g_qSharedLogger, "Peer connection state changed to: Unknown"); break;
            }
        });

    /////////////////////////////////////////////////////////////////////
    // Set some callbacks on important events for the data channel.
    /////////////////////////////////////////////////////////////////////

    m_pDataChannel->onOpen(
        [this]()
        {
            // Add peer connection track callback. It's only safe to add this callback after the data channel is opened.
            m_pPeerConnection->onTrack(
                [this](std::shared_ptr<rtc::Track> rtcTrack)
                {
                    // Submit logger message.
                    rtc::Description::Media rtcMediaDescription = rtcTrack->description();
                    // Get some information about the track.
                    std::string szMediaType = rtcMediaDescription.type();

                    // Check if the track is a video track.
                    if (szMediaType != "video")
                    {
                        return;
                    }

                    // Set member variable to the video track.
                    rtcVideoTrack1 = rtcTrack;

                    // Create a H264 depacketization handler and rtcp receiving session.
                    rtcTrack1H264DepacketizationHandler = std::make_shared<rtc::H264RtpDepacketizer>();
                    // rtcRTCPReceivingSession       = std::make_shared<rtc::RtcpReceivingSession>();
                    // rtcRTPDepacketizer = std::make_shared<rtc::RtpDepacketizer>();
                    // rtcRTCPSrReporter             = std::make_shared<rtc::RtcpSrReporter>(
                    // std::make_shared<rtc::RtpPacketizationConfig>(rtc::SSRC(0), "ZEDFrontRGBAndDepth", 96, rtc::H264RtpPacketizer::defaultClockRate));
                    // rtcRTCPNackResponder = std::make_shared<rtc::RtcpNackResponder>();
                    // For the receiving size of the media track, the pipeline is run from first to last for sending and from last to first for
                    // receiving, which allows you to setup a bidirectional track. Set the media handler for the video track.
                    // rtcH264DepacketizationHandler->addToChain(rtcRTCPSrReporter);
                    // rtcH264DepacketizationHandler->addToChain(rtcRTCPNackResponder);
                    // rtcH264DepacketizationHandler->addToChain(rtcRTPDepacketizer);
                    // rtcH264DepacketizationHandler->addToChain(rtcRTCPReceivingSession);
                    rtcVideoTrack1->setMediaHandler(rtcTrack1H264DepacketizationHandler);

                    // Set the onMessage callback for the video track.
                    rtcVideoTrack1->onFrame(
                        [this](rtc::binary rtcBinaryMessage, rtc::FrameInfo rtcFrameInfo)
                        {
                            // Print frame info timestamp and payload type.
                            // LOG_INFO(logging::g_qSharedLogger, "FrameInfo timestamp: {}, payload type: {}", rtcFrameInfo.timestamp, rtcFrameInfo.payloadType);
                            // Change the rtc::Binary (std::vector<std::byte>) to a std::vector<uint8_t>.
                            std::vector<uint8_t> vH264EncodedBytes;
                            vH264EncodedBytes.reserve(rtcBinaryMessage.size());
                            for (std::byte stdByte : rtcBinaryMessage)
                            {
                                vH264EncodedBytes.push_back(static_cast<uint8_t>(stdByte));
                            }
                            // Acquire lock on the WebRTC copy mutex.
                            std::unique_lock<std::shared_mutex> lkWebRTC(m_muWebRTCCopyMutex);
                            // Decode the H264 encoded bytes to a cv::Mat.
                            this->DecodeH264BytesToCVMat(vH264EncodedBytes, m_cvFrame);
                        });
                });

            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Data channel opened.");
        });

    m_pDataChannel->onMessage(
        [this](std::variant<rtc::binary, rtc::string> rtcMessage)
        {
            // Create instance variables.
            nlohmann::json jsnMessage;

            // Check if the message is a string.
            if (std::holds_alternative<rtc::string>(rtcMessage))
            {
                // Retrieve the string message.
                std::string szMessage = std::get<rtc::string>(rtcMessage);
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "Received message from data channel: {}", szMessage);
            }
            else if (std::holds_alternative<rtc::binary>(rtcMessage))
            {
                // Retrieve the binary message.
                rtc::binary rtcBinaryData = std::get<rtc::binary>(rtcMessage);
                // Print length of binary data.
                LOG_INFO(logging::g_qSharedLogger, "Received binary data of length: {}", rtcBinaryData.size());

                // Convert the binary data to a string.
                std::string szBinaryDataStr(reinterpret_cast<const char*>(rtcBinaryData.data()), rtcBinaryData.size());
                // Print the UTF-8 string
                std::cout << szBinaryDataStr << std::endl;
                // Just for fun, print out if the peer connection had media available.
                LOG_INFO(logging::g_qSharedLogger, "PeerConnection has media available: {}", m_pPeerConnection->hasMedia());

                // Parse the binary data as JSON.
                // jsnMessage = nlohmann::json::parse(szBinaryDataStr);
            }
            else
            {
                LOG_ERROR(logging::g_qSharedLogger, "Received unknown message type from data channel");
            }

            // Check if the message is a PixelStreaming message that contains information about the WebRTC configuration.
            if (jsnMessage.contains("PixelStreaming") && jsnMessage.contains("WebRTC"))
            {
                // Get the WebRTC configuration from the message.
                std::string szDegradationPreference = jsnMessage["WebRTC"]["DegradationPref"];
                int nFramerate                      = jsnMessage["WebRTC"]["FPS"];
                int nMinBitrate                     = jsnMessage["WebRTC"]["MinBitrate"];
                int nMaxBitrate                     = jsnMessage["WebRTC"]["MaxBitrate"];
                int nLowQP                          = jsnMessage["WebRTC"]["LowQP"];
                int nHighQP                         = jsnMessage["WebRTC"]["HighQP"];

                // Don't know what to do with this information yet.
            }
        });

    return true;
}

/******************************************************************************
 * @brief Decodes H264 encoded bytes to a cv::Mat using FFmpeg.
 *
 * @param vH264EncodedBytes - The H264 encoded bytes.
 * @param cvDecodedFrame - The decoded frame.
 * @return true - Frame was successfully decoded.
 * @return false - Frame was not successfully decoded.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-11-16
 ******************************************************************************/
bool SIMZEDCam::DecodeH264BytesToCVMat(const std::vector<uint8_t>& vH264EncodedBytes, cv::Mat& cvDecodedFrame)
{
    // Find the H264 decoder
    const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec)
    {
        std::cerr << "H264 codec not found!" << std::endl;
        return false;
    }

    // Create codec context
    AVCodecContext* codecCtx = avcodec_alloc_context3(codec);
    if (!codecCtx)
    {
        std::cerr << "Failed to allocate codec context!" << std::endl;
        return false;
    }

    // Open the codec
    if (avcodec_open2(codecCtx, codec, nullptr) < 0)
    {
        std::cerr << "Failed to open codec!" << std::endl;
        avcodec_free_context(&codecCtx);
        return false;
    }

    // Allocate the AVPacket
    AVPacket* packet = av_packet_alloc();
    if (!packet)
    {
        std::cerr << "Failed to allocate packet!" << std::endl;
        avcodec_free_context(&codecCtx);
        return false;
    }

    // Allocate the AVFrame
    AVFrame* frame = av_frame_alloc();
    if (!frame)
    {
        std::cerr << "Failed to allocate frame!" << std::endl;
        av_packet_free(&packet);
        avcodec_free_context(&codecCtx);
        return false;
    }

    // Fill the packet with input data
    packet->data = const_cast<uint8_t*>(vH264EncodedBytes.data());
    packet->size = static_cast<int>(vH264EncodedBytes.size());

    // Send the packet to the decoder
    if (avcodec_send_packet(codecCtx, packet) < 0)
    {
        std::cerr << "Failed to send packet to decoder!" << std::endl;
        av_frame_free(&frame);
        av_packet_free(&packet);
        avcodec_free_context(&codecCtx);
        return false;
    }

    // Receive the decoded frame
    if (avcodec_receive_frame(codecCtx, frame) < 0)
    {
        std::cerr << "Failed to receive frame from decoder!" << std::endl;
        av_frame_free(&frame);
        av_packet_free(&packet);
        avcodec_free_context(&codecCtx);
        return false;
    }

    // Check if the format is BGRA
    if (frame->format != AV_PIX_FMT_BGRA)
    {
        std::cerr << "Unexpected pixel format: " << frame->format << std::endl;
        av_frame_free(&frame);
        av_packet_free(&packet);
        avcodec_free_context(&codecCtx);
        return false;
    }

    // Create OpenCV Mat (preserving alpha channel), but only if the frame isn't already the correct size and format.
    if (cvDecodedFrame.empty() || cvDecodedFrame.cols != frame->width || cvDecodedFrame.rows != frame->height || cvDecodedFrame.type() != CV_8UC4)
    {
        cvDecodedFrame = cv::Mat(frame->height, frame->width, CV_8UC4);
    }

    // Copy data directly to OpenCV Mat
    for (int y = 0; y < frame->height; ++y)
    {
        std::memcpy(cvDecodedFrame.ptr(y), frame->data[0] + y * frame->linesize[0], frame->width * 4);
    }

    // Cleanup
    av_frame_free(&frame);
    av_packet_free(&packet);
    avcodec_free_context(&codecCtx);

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
