/******************************************************************************
 * @brief Implements the WebRTC class.
 *
 * @file WebRTC.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-11-30
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "WebRTC.h"
#include "../../../AutonomyLogging.h"

#include <regex>

/******************************************************************************
 * @brief Construct a new Web RTC::WebRTC object.
 *
 * @param szSignallingServerURL -  The URL of the signalling server.
 * @param szStreamerID - The ID of the streamer.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-02
 ******************************************************************************/
WebRTC::WebRTC(const std::string& szSignallingServerURL, const std::string& szStreamerID)
{
    // Set member variables.
    m_szSignallingServerURL     = szSignallingServerURL;
    m_szStreamerID              = szStreamerID;
    m_tmLastKeyFrameRequestTime = std::chrono::system_clock::now();

    // Setup the FFMPEG H264 decoder.
    this->InitializeH264Decoder();

    // Enable logging from the WebRTC LibDataChannel library for debugging.
    // rtc::InitLogger(rtc::LogLevel::Verbose);

    // Construct the WebRTC peer connection and data channel for receiving data from the simulator.
    rtc::WebSocket::Configuration rtcWebSocketConfig;
    rtc::Configuration rtcPeerConnectionConfig;
    rtcPeerConnectionConfig.forceMediaTransport = true;
    rtcPeerConnectionConfig.maxMessageSize      = 100000000;
    m_pWebSocket                                = std::make_shared<rtc::WebSocket>(rtcWebSocketConfig);
    m_pPeerConnection                           = std::make_shared<rtc::PeerConnection>(rtcPeerConnectionConfig);
    m_pDataChannel                              = m_pPeerConnection->createDataChannel("webrtc-datachannel");

    // Attempt to connect to the signalling server.
    this->ConnectToSignallingServer(szSignallingServerURL);
}

/******************************************************************************
 * @brief Destroy the Web RTC::WebRTC object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-02
 ******************************************************************************/
WebRTC::~WebRTC()
{
    // Close the video track, peer connection, data channel, and websocket.
    m_pVideoTrack1->close();
    m_pPeerConnection->close();
    m_pDataChannel->close();
    m_pWebSocket->close();

    // Wait for all connections to close.
    while (!m_pVideoTrack1->isClosed() || !m_pDataChannel->isClosed() || !m_pWebSocket->isClosed())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Manually destroy the smart pointers.
    m_pVideoTrack1.reset();
    m_pPeerConnection.reset();
    m_pDataChannel.reset();
    m_pWebSocket.reset();

    // Free the codec context.
    if (m_pSWSContext)
    {
        sws_freeContext(m_pSWSContext);
    }
    if (m_pFrame)
    {
        av_frame_free(&m_pFrame);
    }
    if (m_pPacket)
    {
        av_packet_free(&m_pPacket);
    }
    if (m_pAVCodecContext)
    {
        avcodec_free_context(&m_pAVCodecContext);
    }

    // Set dangling pointers to nullptr.
    m_pAVCodecContext = nullptr;
    m_pSWSContext     = nullptr;
}

/******************************************************************************
 * @brief Set the callback function for when a new frame is received.
 *
 * @param fnOnFrameReceivedCallback - The callback function to set.
 * @param eOutputPixelFormat - The output pixel format to use.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-02
 ******************************************************************************/
void WebRTC::SetOnFrameReceivedCallback(std::function<void(cv::Mat&)> fnOnFrameReceivedCallback, const AVPixelFormat eOutputPixelFormat)
{
    // Set the callback function.
    m_fnOnFrameReceivedCallback = fnOnFrameReceivedCallback;
    // Set the output pixel format.
    m_eOutputPixelFormat = eOutputPixelFormat;
}

/******************************************************************************
 * @brief Get the connection status of the WebRTC object.
 *
 * @return true - The WebRTC object is connected.
 * @return false - The WebRTC object is not connected.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
bool WebRTC::GetIsConnected() const
{
    return m_pWebSocket->isOpen();
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
bool WebRTC::ConnectToSignallingServer(const std::string& szSignallingServerURL)
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
            LOG_INFO(logging::g_qSharedLogger, "Connected to the signalling server via {}. Checking if stream {} exists...", m_szSignallingServerURL, m_szStreamerID);

            // Request the streamer list from the server. This also kicks off the negotiation process.
            nlohmann::json jsnStreamList;
            jsnStreamList["type"] = "listStreamers";
            m_pWebSocket->send(jsnStreamList.dump());
        });

    // WebSocket has been closed.
    m_pWebSocket->onClosed(
        [this]()
        {
            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Closed {} stream and disconnected from the signalling server.", m_szStreamerID);
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
                    LOG_DEBUG(logging::g_qSharedLogger, "Received message from signalling server: {}", szMessage);
                }
                else if (std::holds_alternative<rtc::binary>(rtcMessage))
                {
                    // Retrieve the binary message.
                    rtc::binary rtcBinaryData = std::get<rtc::binary>(rtcMessage);
                    // Print length of binary data.
                    LOG_DEBUG(logging::g_qSharedLogger, "Received binary data of length: {}", rtcBinaryData.size());

                    // Convert the binary data to a string.
                    std::string szBinaryDataStr(reinterpret_cast<const char*>(rtcBinaryData.data()), rtcBinaryData.size());
                    // Print the binary data as a string.
                    LOG_DEBUG(logging::g_qSharedLogger, "Received binary data: {}", szBinaryDataStr);
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
                    // If the message from the server is a config message, do nothing.
                    if (szType == "config")
                    {
                        // Submit logger message.
                        LOG_DEBUG(logging::g_qSharedLogger, "Received config message from signalling server: {}", jsnMessage.dump());
                    }
                    // If the message from the server is an offer, set the remote description offer.
                    if (szType == "offer")
                    {
                        // Get the SDP offer and set it as the remote description.
                        std::string sdp = jsnMessage["sdp"];
                        m_pPeerConnection->setRemoteDescription(rtc::Description(sdp, "offer"));
                        LOG_DEBUG(logging::g_qSharedLogger, "Processing SDP offer from signalling server: {}", sdp);
                    }
                    // If the message from the server is an answer, set the remote description answer.
                    else if (szType == "answer")
                    {
                        // Get the SDP answer and set it as the remote description.
                        std::string sdp = jsnMessage["sdp"];
                        m_pPeerConnection->setRemoteDescription(rtc::Description(sdp, "answer"));
                        LOG_DEBUG(logging::g_qSharedLogger, "Processing SDP answer from signalling server: {}", sdp);
                    }
                    // If the message from the server is advertising an ICE candidate, add it to the peer connection.
                    else if (szType == "iceCandidate")
                    {
                        // Handle ICE candidate
                        nlohmann::json jsnCandidate = jsnMessage["candidate"];
                        std::string szCandidateStr  = jsnCandidate["candidate"];

                        rtc::Candidate rtcCandidate = rtc::Candidate(szCandidateStr);
                        m_pPeerConnection->addRemoteCandidate(rtcCandidate);
                        LOG_DEBUG(logging::g_qSharedLogger, "Added ICE candidate to peer connection: {}", szCandidateStr);
                    }
                    else if (szType == "streamerList")
                    {
                        // Print the streamer list.
                        LOG_DEBUG(logging::g_qSharedLogger, "Streamer List: {}", jsnMessage.dump());

                        // Check that the streamer ID given by the user is in the streamer list.
                        if (jsnMessage.contains("ids"))
                        {
                            std::vector<std::string> streamerList = jsnMessage["ids"].get<std::vector<std::string>>();
                            if (std::find(streamerList.begin(), streamerList.end(), m_szStreamerID) != streamerList.end())
                            {
                                // Send what stream we want to the server.
                                nlohmann::json jsnStream;
                                jsnStream["type"]       = "subscribe";
                                jsnStream["streamerId"] = m_szStreamerID;
                                m_pWebSocket->send(jsnStream.dump());
                                // Submit logger message.
                                LOG_DEBUG(logging::g_qSharedLogger, "Streamer ID {} found in the streamer list. Subscribing to stream...", m_szStreamerID);
                            }
                            else
                            {
                                LOG_ERROR(logging::g_qSharedLogger, "Streamer ID {} not found in the streamer list!", m_szStreamerID);
                            }
                        }
                        else
                        {
                            LOG_ERROR(logging::g_qSharedLogger, "Streamer list does not contain 'ids' field!");
                        }
                    }
                    else
                    {
                        LOG_ERROR(logging::g_qSharedLogger, "Unknown message type received from signalling server: {}", szType);
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
            // Check the type of the description.
            if (rtcDescription.typeString() == "offer")
            {
                return;
            }

            // First lets send some preconfig stuff.
            nlohmann::json jsnConfigMessage;
            jsnConfigMessage["type"]          = "layerPreference";
            jsnConfigMessage["spatialLayer"]  = 0;
            jsnConfigMessage["temporalLayer"] = 0;
            jsnConfigMessage["playerId"]      = "";
            m_pWebSocket->send(jsnConfigMessage.dump());

            // Send the local description to the signalling server
            nlohmann::json jsnMessage;
            jsnMessage["type"] = rtcDescription.typeString();
            // This next bit is specific to the Unreal Engine 5 Signalling Server, we must append the min/max bitrate to the message.
            jsnMessage["minBitrateBps"] = 0;
            jsnMessage["maxBitrateBps"] = 0;
            // Here's our actual SDP.
            std::string szSDP = rtcDescription.generateSdp();
            // Munger the SDP to add the bitrate.
            std::string szMungedSDP =
                std::regex_replace(szSDP, std::regex("(a=fmtp:\\d+ level-asymmetry-allowed=.*)\r\n"), "$1;x-google-start-bitrate=10000;x-google-max-bitrate=100000\r\n");
            jsnMessage["sdp"] = szMungedSDP;
            // Send the message.
            m_pWebSocket->send(jsnMessage.dump());

            // Submit logger message.
            LOG_DEBUG(logging::g_qSharedLogger, "Sending local description to signalling server: {}", jsnMessage.dump());
        });

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
            m_pVideoTrack1 = rtcTrack;

            // Create a H264 depacketization handler and rtcp receiving session.
            m_pTrack1H264DepacketizationHandler = std::make_shared<rtc::H264RtpDepacketizer>(rtc::NalUnit::Separator::LongStartSequence);
            m_pTrack1RtcpReceivingSession       = std::make_shared<rtc::RtcpReceivingSession>();
            m_pTrack1H264DepacketizationHandler->addToChain(m_pTrack1RtcpReceivingSession);
            m_pVideoTrack1->setMediaHandler(m_pTrack1H264DepacketizationHandler);

            // Set the onMessage callback for the video track.
            m_pVideoTrack1->onFrame(
                [this](rtc::binary rtcBinaryMessage, rtc::FrameInfo rtcFrameInfo)
                {
                    // Assuming 96 is the H264 payload type.
                    if (rtcFrameInfo.payloadType == 96)
                    {
                        // Change the rtc::Binary (std::vector<std::byte>) to a std::vector<uint8_t>.
                        std::vector<uint8_t> vH264EncodedBytes;
                        vH264EncodedBytes.reserve(rtcBinaryMessage.size());
                        for (std::byte stdByte : rtcBinaryMessage)
                        {
                            vH264EncodedBytes.push_back(static_cast<uint8_t>(stdByte));
                        }

                        // Pass to FFmpeg decoder
                        this->DecodeH264BytesToCVMat(vH264EncodedBytes, m_cvFrame, m_eOutputPixelFormat);

                        // Check if the callback function is set before calling it.
                        if (m_fnOnFrameReceivedCallback)
                        {
                            // Call the user's callback function.
                            m_fnOnFrameReceivedCallback(m_cvFrame);
                        }
                    }
                });
        });

    m_pPeerConnection->onGatheringStateChange(
        [this](rtc::PeerConnection::GatheringState eGatheringState)
        {
            // Switch to translate the state to a string.
            switch (eGatheringState)
            {
                case rtc::PeerConnection::GatheringState::Complete: LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection ICE gathering state changed to: Complete"); break;

                case rtc::PeerConnection::GatheringState::InProgress:
                    LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection ICE gathering state changed to: InProgress");
                    break;
                case rtc::PeerConnection::GatheringState::New: LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection ICE gathering state changed to: New"); break;
                default: LOG_DEBUG(logging::g_qSharedLogger, "Peer connection ICE gathering state changed to: Unknown"); break;
            }
        });

    m_pPeerConnection->onIceStateChange(
        [this](rtc::PeerConnection::IceState eIceState)
        {
            // Switch to translate the state to a string.
            switch (eIceState)
            {
                case rtc::PeerConnection::IceState::Checking: LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection ICE state changed to: Checking"); break;
                case rtc::PeerConnection::IceState::Closed: LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection ICE state changed to: Closed"); break;
                case rtc::PeerConnection::IceState::Completed: LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection ICE state changed to: Completed"); break;
                case rtc::PeerConnection::IceState::Connected: LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection ICE state changed to: Connected"); break;
                case rtc::PeerConnection::IceState::Disconnected: LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection ICE state changed to: Disconnected"); break;
                case rtc::PeerConnection::IceState::Failed: LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection ICE state changed to: Failed"); break;
                case rtc::PeerConnection::IceState::New: LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection ICE state changed to: New"); break;
                default: LOG_DEBUG(logging::g_qSharedLogger, "Peer connection ICE state changed to: Unknown"); break;
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
                    LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection signaling state changed to: HaveLocalOffer");
                    break;
                }
                case rtc::PeerConnection::SignalingState::HaveLocalPranswer:
                    LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection signaling state changed to: HaveLocalPranswer");
                    break;
                case rtc::PeerConnection::SignalingState::HaveRemoteOffer:
                    LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection signaling state changed to: HaveRemoteOffer");
                    break;
                case rtc::PeerConnection::SignalingState::HaveRemotePranswer:
                    LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection signaling state changed to: HaveRemotePrAnswer");
                    break;
                case rtc::PeerConnection::SignalingState::Stable: LOG_DEBUG(logging::g_qSharedLogger, "PeerConnection signaling state changed to: Stable"); break;
                default: LOG_DEBUG(logging::g_qSharedLogger, "Peer connection signaling state changed to: Unknown"); break;
            }
        });

    m_pPeerConnection->onStateChange(
        [this](rtc::PeerConnection::State eState)
        {
            // Switch to translate the state to a string.
            switch (eState)
            {
                case rtc::PeerConnection::State::Closed: LOG_DEBUG(logging::g_qSharedLogger, "Peer connection state changed to: Closed"); break;
                case rtc::PeerConnection::State::Connected: LOG_DEBUG(logging::g_qSharedLogger, "Peer connection state changed to: Connected"); break;
                case rtc::PeerConnection::State::Connecting: LOG_DEBUG(logging::g_qSharedLogger, "Peer connection state changed to: Connecting"); break;
                case rtc::PeerConnection::State::Disconnected: LOG_DEBUG(logging::g_qSharedLogger, "Peer connection state changed to: Disconnected"); break;
                case rtc::PeerConnection::State::Failed: LOG_DEBUG(logging::g_qSharedLogger, "Peer connection state changed to: Failed"); break;
                case rtc::PeerConnection::State::New: LOG_DEBUG(logging::g_qSharedLogger, "Peer connection state changed to: New"); break;
                default: LOG_DEBUG(logging::g_qSharedLogger, "Peer connection state changed to: Unknown"); break;
            }
        });

    /////////////////////////////////////////////////////////////////////
    // Set some callbacks on important events for the data channel.
    /////////////////////////////////////////////////////////////////////

    m_pDataChannel->onOpen(
        [this]()
        {
            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Data channel opened.");

            // Request quality control of the stream.
            m_pDataChannel->send(std::string(1, static_cast<char>(1)));
        });

    m_pDataChannel->onMessage(
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
                    LOG_NOTICE(logging::g_qSharedLogger, "DATA_CHANNEL Received message from peer: {}", szMessage);
                }
                else if (std::holds_alternative<rtc::binary>(rtcMessage))
                {
                    // Retrieve the binary message.
                    rtc::binary rtcBinaryData = std::get<rtc::binary>(rtcMessage);

                    // Convert the binary data to a string, ignoring non-printable characters.
                    std::string szBinaryDataStr;
                    for (auto byte : rtcBinaryData)
                    {
                        if (std::isprint(static_cast<unsigned char>(byte)))
                        {
                            szBinaryDataStr += static_cast<char>(byte);
                        }
                    }

                    // Print the binary data as a string.
                    LOG_DEBUG(logging::g_qSharedLogger, "DATA_CHANNEL Received binary data ({} bytes): {}", rtcBinaryData.size(), szBinaryDataStr);
                }
                else
                {
                    LOG_ERROR(logging::g_qSharedLogger, "Received unknown message type from peer");
                }
            }
            catch (const std::exception& e)
            {
                // Submit logger message.
                LOG_ERROR(logging::g_qSharedLogger, "Error occurred while negotiating with the datachannel: {}", e.what());
            }
        });

    return true;
}

/******************************************************************************
 * @brief Initialize the H264 decoder. Creates the AVCodecContext, AVFrame, and AVPacket.
 *
 * @return true - Successfully initialized the H264 decoder.
 * @return false - Failed to initialize the H264 decoder.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-27
 ******************************************************************************/
bool WebRTC::InitializeH264Decoder()
{
    // Configure logging level from FFMPEG library.
    av_log_set_level(AV_LOG_DEBUG);

    // Find the H264 decoder
    const AVCodec* avCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!avCodec)
    {
        LOG_ERROR(logging::g_qSharedLogger, "H264 codec not found!");
        return false;
    }
    // Create codec context
    m_pAVCodecContext = avcodec_alloc_context3(avCodec);
    if (!m_pAVCodecContext)
    {
        LOG_ERROR(logging::g_qSharedLogger, "Failed to allocate codec context!");
        return false;
    }
    // Set codec context options.
    m_pAVCodecContext->flags |= AV_CODEC_FLAG2_FAST;
    m_pAVCodecContext->err_recognition = AV_EF_COMPLIANT | AV_EF_CAREFUL;
    m_pAVCodecContext->rc_buffer_size  = 50 * 1024 * 1024;    // 50 MB buffer size.
    av_opt_set_int(m_pAVCodecContext, "refcounted_frames", 1, 0);
    av_opt_set_int(m_pAVCodecContext, "error_concealment", FF_EC_GUESS_MVS | FF_EC_DEBLOCK, 0);
    av_opt_set_int(m_pAVCodecContext, "threads", 4, 0);

    // Open the codec
    if (avcodec_open2(m_pAVCodecContext, avCodec, nullptr) < 0)
    {
        LOG_ERROR(logging::g_qSharedLogger, "Failed to open codec!");
        return false;
    }

    // Allocate the AVPacket.
    m_pPacket = av_packet_alloc();
    // Allocate the AVFrame.
    m_pFrame = av_frame_alloc();
    if (!m_pPacket || !m_pFrame)
    {
        LOG_ERROR(logging::g_qSharedLogger, "Failed to allocate packet or frame!");
        return false;
    }

    return true;
}

/******************************************************************************
 * @brief Decodes H264 encoded bytes to a cv::Mat using FFmpeg.
 *
 * @param vH264EncodedBytes - The H264 encoded bytes.
 * @param cvDecodedFrame - The decoded frame.
 * @param eOutputPixelFormat - The output pixel format.
 * @return true - Frame was successfully decoded.
 * @return false - Frame was not successfully decoded.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-11-16
 ******************************************************************************/
bool WebRTC::DecodeH264BytesToCVMat(const std::vector<uint8_t>& vH264EncodedBytes, cv::Mat& cvDecodedFrame, const AVPixelFormat eOutputPixelFormat)
{
    // Initialize packet data.
    m_pPacket->data = const_cast<uint8_t*>(vH264EncodedBytes.data());
    m_pPacket->size = static_cast<int>(vH264EncodedBytes.size());

    // Send the packet to the decoder.
    int nReturnCode = avcodec_send_packet(m_pAVCodecContext, m_pPacket);
    if (nReturnCode < 0)
    {
        // Get the error message.
        char aErrorBuffer[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(nReturnCode, aErrorBuffer, AV_ERROR_MAX_STRING_SIZE);
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Failed to send packet to decoder! Error code: {} {}", nReturnCode, aErrorBuffer);
        // Request a new keyframe from the video track.
        this->RequestKeyFrame();

        return false;
    }

    // Receive decoded frames in a loop
    while (true)
    {
        nReturnCode = avcodec_receive_frame(m_pAVCodecContext, m_pFrame);
        if (nReturnCode == AVERROR(EAGAIN) || nReturnCode == AVERROR_EOF)
        {
            // No more frames available in stream.
            break;
        }
        else if (nReturnCode < 0)
        {
            // Get the error message.
            char aErrorBuffer[AV_ERROR_MAX_STRING_SIZE];
            av_strerror(nReturnCode, aErrorBuffer, AV_ERROR_MAX_STRING_SIZE);
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "Failed to receive frame from decoder! Error code: {} {}", nReturnCode, aErrorBuffer);
            // Request a new keyframe from the video track.
            this->RequestKeyFrame();

            return false;
        }

        // Check if the user want to keep the YUV420P data un-altered.
        if (eOutputPixelFormat == AV_PIX_FMT_YUV420P)
        {
            // The frame received from the FFMPEG H264 decoder is already in YUV420P format.
            // We want to keep the raw YUV420P byte data un-altered, but store that data in a RGB 3 channel Mat.
            // Absolutely no colorspace conversion or the binary data will be corrupted.

            // Extract the Y, U, and V planes.
            cv::Mat cvYPlane(m_pFrame->height, m_pFrame->width, CV_8UC1, m_pFrame->data[0]);
            cv::Mat cvUPlane(m_pFrame->height / 2, m_pFrame->width / 2, CV_8UC1, m_pFrame->data[1]);
            cv::Mat cvVPlane(m_pFrame->height / 2, m_pFrame->width / 2, CV_8UC1, m_pFrame->data[2]);
            // Upsample the U and V planes to match the Y plane.
            cv::Mat cvUPlaneUpsampled, cvVPlaneUpsampled;
            cv::resize(cvUPlane, cvUPlaneUpsampled, cv::Size(m_pFrame->width, m_pFrame->height), 0, 0, cv::INTER_NEAREST);
            cv::resize(cvVPlane, cvVPlaneUpsampled, cv::Size(m_pFrame->width, m_pFrame->height), 0, 0, cv::INTER_NEAREST);
            // Merge the Y, U, and V planes into a single 3 channel Mat.
            std::vector<cv::Mat> vYUVPlanes = {cvYPlane, cvUPlaneUpsampled, cvVPlaneUpsampled};
            cv::merge(vYUVPlanes, cvDecodedFrame);
        }
        else
        {
            // Convert the decoded frame to cv::Mat using sws_scale.
            if (!m_pSWSContext)
            {
                m_pSWSContext = sws_getContext(m_pFrame->width,
                                               m_pFrame->height,
                                               static_cast<AVPixelFormat>(m_pFrame->format),
                                               m_pFrame->width,
                                               m_pFrame->height,
                                               eOutputPixelFormat,
                                               SWS_FAST_BILINEAR,
                                               nullptr,
                                               nullptr,
                                               nullptr);
                if (!m_pSWSContext)
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "Failed to initialize SwsContext!");
                    // Request a new keyframe from the video track.
                    this->RequestKeyFrame();

                    return false;
                }

                // Allocate buffer for the frame's data
                int nRetCode = av_image_alloc(m_pFrame->data, m_pFrame->linesize, m_pFrame->width, m_pFrame->height, static_cast<AVPixelFormat>(m_pFrame->format), 32);
                if (nRetCode < 0)
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "Failed to allocate image buffer!");
                    return false;
                }
            }

            // Lock the mutex before calling sws_scale.
            std::lock_guard<std::mutex> lock(m_muDecoderMutex);
            // Create new mat for the decoded frame.
            cvDecodedFrame.create(m_pFrame->height, m_pFrame->width, CV_8UC3);
            std::array<uint8_t*, 4> aDest    = {cvDecodedFrame.data, nullptr, nullptr, nullptr};
            std::array<int, 4> aDestLinesize = {static_cast<int>(cvDecodedFrame.step[0]), 0, 0, 0};
            sws_scale(m_pSWSContext, m_pFrame->data, m_pFrame->linesize, 0, m_pFrame->height, aDest.data(), aDestLinesize.data());
        }

        // Calculate the time since the last key frame request.
        std::chrono::duration<double> tmTimeSinceLastKeyFrameRequest = std::chrono::system_clock::now() - m_tmLastKeyFrameRequestTime;
        // Check if the time since the last key frame request is greater than the key frame request interval.
        if (tmTimeSinceLastKeyFrameRequest.count() > 1.0)
        {
            // Request a new key frame from the video track.
            // this->RequestKeyFrame();

            // Set the QP factor to 0. (Max quality)
            this->SendCommandToStreamer(R"({"Encoder.MaxQP":15})");
            // Set the bitrate limits.
            this->SendCommandToStreamer(R"({"WebRTC.MinBitrate":99999})");
            this->SendCommandToStreamer(R"({"WebRTC.MaxBitrate":99999999})");
            // Set FPS to 30.
            this->SendCommandToStreamer(R"({"WebRTC.Fps":30})");
            this->SendCommandToStreamer(R"({"WebRTC.MaxFps":30})");
            // Here's the magic, this might work. Target bitrate is what has been causing the issues.
            this->SendCommandToStreamer(R"({"Encoder.TargetBitrate":99999999})");

            // Update the time of the last key frame request.
            m_tmLastKeyFrameRequestTime = std::chrono::system_clock::now();
        }
    }

    return true;
}

/******************************************************************************
 * @brief Requests a key frame from the given video track. This is useful for when the
 *      video track is out of sync or has lost frames.
 *
 * @return true - Key frame was successfully requested.
 * @return false - Key frame was not successfully requested.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-11-30
 ******************************************************************************/
bool WebRTC::RequestKeyFrame()
{
    // Check if the video track is valid.
    if (!m_pVideoTrack1)
    {
        LOG_ERROR(logging::g_qSharedLogger, "Invalid video track!");
        return false;
    }

    // Submit logger message.
    LOG_DEBUG(logging::g_qSharedLogger, "Requested key frame from video track. Success?: {}", m_pVideoTrack1->requestKeyframe());

    // Request a key frame from the video track.
    return true;
}

/******************************************************************************
 * @brief This method sends a command to the streamer via the data channel.
 *      The command is a JSON string that is sent as a binary message.
 *      The PixelStreaming plugin handles the command very weirdly, so we have to
 *      sort of encode the command in a specific way. This handles that encoding.
 *
 * @param szCommand - The command to send to the streamer.
 * @return true - Command was successfully sent.
 * @return false - Command was not successfully sent.
 *
 * @note This will only work for valid COMMANDS with ID of type 51. Check the
 *      PixelStreamingInfrastructure repo for more information.
 *      https://github.com/EpicGamesExt/PixelStreamingInfrastructure/blob/13ce022d3a09d315d4ca85c05b61a8d3fe92741c/Extras/JSStreamer/src/protocol.ts#L196
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-01
 ******************************************************************************/
bool WebRTC::SendCommandToStreamer(const std::string& szCommand)
{
    // Check if the data channel is valid.
    if (!m_pDataChannel)
    {
        LOG_ERROR(logging::g_qSharedLogger, "Invalid data channel!");
        return false;
    }

    // Check if the command is empty.
    if (szCommand.empty())
    {
        LOG_ERROR(logging::g_qSharedLogger, "Empty command!");
        return false;
    }

    // Set the max QP to 0.
    std::string szID(1, static_cast<char>(51));    // This is the ID for COMMAND.
    std::string szSize(1, static_cast<char>(szCommand.size()));
    std::string szFinal = szID + szSize + szCommand;
    // Loop through the string and build a binary message.
    rtc::binary rtcBinaryMessage;
    // First byte is the ID.
    rtcBinaryMessage.push_back(static_cast<std::byte>(szID[0]));
    // Next two bytes are the size.
    rtcBinaryMessage.push_back(static_cast<std::byte>(szSize[0]));
    rtcBinaryMessage.push_back(static_cast<std::byte>(szSize[1]));
    // The rest of the bytes are the command, but this is utf16 so we need to add a null byte before each character.
    for (char cLetter : szCommand)
    {
        rtcBinaryMessage.push_back(static_cast<std::byte>(cLetter));
        rtcBinaryMessage.push_back(static_cast<std::byte>(0));
    }

    // Send the binary message.
    return m_pDataChannel->send(rtcBinaryMessage);
}
