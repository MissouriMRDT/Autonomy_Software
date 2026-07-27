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
#include "../../../AutonomyConstants.h"    // Added for constants::SIM_WEBRTC_QP
#include "../../../AutonomyLogging.h"

/// \cond
#include <regex>

/// \endcond

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
    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "WebRTC camera constructing instance. Target URL: {}, StreamerID: {}", szSignallingServerURL, szStreamerID);

    // Set member variables.
    m_szSignallingServerURL     = szSignallingServerURL;
    m_szStreamerID              = szStreamerID;
    m_tmLastKeyFrameRequestTime = std::chrono::system_clock::now();

    // Setup the FFMPEG H264 decoder.
    if (this->InitializeH264Decoder())
    {
        LOG_INFO(logging::g_qSharedLogger, "WebRTC camera {} H264 Decoder initialized successfully.", m_szStreamerID);
    }
    else
    {
        LOG_ERROR(logging::g_qSharedLogger, "WebRTC camera {} Failed to initialize H264 Decoder!", m_szStreamerID);
    }

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

    LOG_INFO(logging::g_qSharedLogger, "WebRTC camera {} PeerConnection and DataChannel objects created.", m_szStreamerID);

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
    LOG_INFO(logging::g_qSharedLogger, "WebRTC camera {} destructor called. Cleaning up...", m_szStreamerID);
    this->CloseConnection();

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
    m_pFrame          = nullptr;
    m_pPacket         = nullptr;
    m_pSWSContext     = nullptr;
    LOG_INFO(logging::g_qSharedLogger, "WebRTC camera {} Cleanup complete.", m_szStreamerID);
}

/******************************************************************************
 * @brief Close the WebRTC connection.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-05-03
 ******************************************************************************/
void WebRTC::CloseConnection()
{
    LOG_INFO(logging::g_qSharedLogger, "WebRTC camera {} closing connections...", m_szStreamerID);
    // Close the WebRTC connections.
    if (m_pVideoTrack1)
    {
        m_pVideoTrack1->resetCallbacks();
        m_pVideoTrack1->close();
    }
    if (m_pDataChannel)
    {
        m_pDataChannel->resetCallbacks();
        m_pDataChannel->close();
    }
    if (m_pPeerConnection)
    {
        m_pPeerConnection->resetCallbacks();
        m_pPeerConnection->close();
    }
    if (m_pWebSocket)
    {
        m_pWebSocket->resetCallbacks();
        m_pWebSocket->close();
    }

    // Wait for all connections to close, but never indefinitely. A peer that negotiated only
    // partially (for example a signalling server that accepted the websocket but never completed
    // the WebRTC handshake) can leave a track or data channel that never reports isClosed(), and
    // an unbounded wait here hangs process shutdown forever. Bound it with a deadline instead: the
    // loop still exits immediately in the normal case, and a stuck peer costs one bounded delay
    // and a warning rather than a hung program.
    const std::chrono::steady_clock::time_point tmCloseDeadline = std::chrono::steady_clock::now() + constants::SIM_STREAM_CLOSE_TIMEOUT;
    while ((m_pVideoTrack1 && !m_pVideoTrack1->isClosed()) || (m_pDataChannel && !m_pDataChannel->isClosed()) || (m_pWebSocket && !m_pWebSocket->isClosed()))
    {
        // Give up waiting once the deadline passes so shutdown always completes.
        if (std::chrono::steady_clock::now() >= tmCloseDeadline)
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger,
                        "WebRTC camera {} did not report all connections closed within {} ms. Continuing shutdown anyway.",
                        m_szStreamerID,
                        constants::SIM_STREAM_CLOSE_TIMEOUT.count());
            break;
        }

        // Poll again shortly.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    LOG_INFO(logging::g_qSharedLogger, "WebRTC camera {} Connections closed.", m_szStreamerID);
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
    LOG_INFO(logging::g_qSharedLogger, "WebRTC camera {} frame received callback set.", m_szStreamerID);
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
    // Check if the datachannel shared pointer is valid.
    if (m_pWebSocket != nullptr)
    {
        // Check if the datachannel is open.
        return m_pWebSocket->isOpen();
    }

    // Return false if the datachannel is not valid.
    return false;
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
    LOG_INFO(logging::g_qSharedLogger, "WebRTC camera {} opening WebSocket to {}...", m_szStreamerID, szSignallingServerURL);
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
            LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} WebSocket OPEN. Connected to {}. Sending 'listStreamers'...", m_szStreamerID, m_szSignallingServerURL);

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
            LOG_INFO(logging::g_qSharedLogger, "WebRTC camera {} WebSocket CLOSED. Disconnected from signalling server.", m_szStreamerID);
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
                    LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} WS Message (String): {}", m_szStreamerID, szMessage);
                }
                else if (std::holds_alternative<rtc::binary>(rtcMessage))
                {
                    // Retrieve the binary message.
                    rtc::binary rtcBinaryData = std::get<rtc::binary>(rtcMessage);
                    // Print length of binary data.
                    LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} WS Message (Binary) of length: {}", m_szStreamerID, rtcBinaryData.size());

                    // Convert the binary data to a string.
                    std::string szBinaryDataStr(reinterpret_cast<const char*>(rtcBinaryData.data()), rtcBinaryData.size());
                    // Print the binary data as a string.
                    LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} WS Binary Content: {}", m_szStreamerID, szBinaryDataStr);
                    // Parse the binary data as JSON.
                    jsnMessage = nlohmann::json::parse(szBinaryDataStr);
                }
                else
                {
                    LOG_ERROR(logging::g_qSharedLogger, "WebRTC camera {} WS Received unknown message type.", m_szStreamerID);
                }

                // Check if the message contains a type.
                if (jsnMessage.contains("type"))
                {
                    std::string szType = jsnMessage["type"];
                    // If the message from the server is a config message, do nothing.
                    if (szType == "config")
                    {
                        // Submit logger message.
                        LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} Received 'config': {}", m_szStreamerID, jsnMessage.dump());
                    }
                    // If the message from the server is an offer, set the remote description offer.
                    else if (szType == "offer")
                    {
                        // Get the SDP offer and set it as the remote description.
                        std::string sdp = jsnMessage["sdp"];
                        m_pPeerConnection->setRemoteDescription(rtc::Description(sdp, "offer"));
                        LOG_DEBUG(logging::g_qSharedLogger,
                                  "WebRTC camera {} Received 'offer'. SDP Length: {}. Setting Remote Description...",
                                  m_szStreamerID,
                                  sdp.length());

                        // Trigger answer creation.
                        m_pPeerConnection->setLocalDescription();
                        LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} Triggered setLocalDescription() to generate answer.", m_szStreamerID);
                    }
                    // If the message from the server is an answer, set the remote description answer.
                    else if (szType == "answer")
                    {
                        // Get the SDP answer and set it as the remote description.
                        std::string sdp = jsnMessage["sdp"];
                        m_pPeerConnection->setRemoteDescription(rtc::Description(sdp, "answer"));
                        LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} Received 'answer'. Setting Remote Description.", m_szStreamerID);
                    }
                    // If the message from the server is advertising an ICE candidate, add it to the peer connection.
                    else if (szType == "iceCandidate")
                    {
                        // Handle ICE candidate
                        nlohmann::json jsnCandidate = jsnMessage["candidate"];
                        std::string szCandidateStr  = jsnCandidate["candidate"];

                        rtc::Candidate rtcCandidate = rtc::Candidate(szCandidateStr);
                        m_pPeerConnection->addRemoteCandidate(rtcCandidate);
                        LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} Received 'iceCandidate'. Added: {}", m_szStreamerID, szCandidateStr);
                    }
                    else if (szType == "streamerList")
                    {
                        // Print the streamer list.
                        LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} Received 'streamerList': {}", m_szStreamerID, jsnMessage.dump());

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
                                LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} Streamer ID {} found! Sending 'subscribe'...", m_szStreamerID, m_szStreamerID);
                            }
                            else
                            {
                                LOG_ERROR(logging::g_qSharedLogger, "WebRTC camera {} Streamer ID {} NOT found in streamer list!", m_szStreamerID, m_szStreamerID);
                            }
                        }
                        else
                        {
                            LOG_ERROR(logging::g_qSharedLogger, "WebRTC camera {} Streamer list does not contain 'ids' field!", m_szStreamerID);
                        }
                    }
                    else
                    {
                        LOG_ERROR(logging::g_qSharedLogger, "WebRTC camera {} Unknown message type received: {}", m_szStreamerID, szType);
                    }
                }
            }
            catch (const std::exception& e)
            {
                // Submit logger message.
                LOG_ERROR(logging::g_qSharedLogger, "WebRTC camera {} Exception during Negotiation: {}", m_szStreamerID, e.what());
            }
        });

    m_pWebSocket->onError(
        [this](const std::string& szError)
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "WebRTC camera {} WebSocket Error: {}", m_szStreamerID, szError);
        });

    /////////////////////////////////////////////////////////////////////
    // Set some callbacks on important events for the peer connection.
    /////////////////////////////////////////////////////////////////////

    m_pPeerConnection->onLocalDescription(
        [this](rtc::Description rtcDescription)
        {
            LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} Generated Local Description (Type: {}).", m_szStreamerID, rtcDescription.typeString());

            // Check the type of the description.
            if (rtcDescription.typeString() == "offer")
            {
                LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} Ignoring 'offer' type in onLocalDescription.", m_szStreamerID);
                return;
            }

            // First lets send some preconfig stuff.
            nlohmann::json jsnConfigMessage;
            jsnConfigMessage["type"]          = "layerPreference";
            jsnConfigMessage["spatialLayer"]  = 0;
            jsnConfigMessage["temporalLayer"] = 0;
            jsnConfigMessage["playerId"]      = "";
            m_pWebSocket->send(jsnConfigMessage.dump());
            LOG_DEBUG(logging::g_qSharedLogger, "WebRTC Sent 'layerPreference'.");

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
            LOG_INFO(logging::g_qSharedLogger, "WebRTC camera {} Sent Local Description to Server (Munged SDP).", m_szStreamerID);
        });

    m_pPeerConnection->onTrack(
        [this](std::shared_ptr<rtc::Track> rtcTrack)
        {
            // Submit logger message.
            rtc::Description::Media rtcMediaDescription = rtcTrack->description();
            // Get some information about the track.
            std::string szMediaType = rtcMediaDescription.type();

            LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} onTrack triggered. Media Type: {}", m_szStreamerID, szMediaType);

            // Check if the track is a video track.
            if (szMediaType != "video")
            {
                LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} Ignoring non-video track.", m_szStreamerID);
                return;
            }

            // Set member variable to the video track.
            m_pVideoTrack1 = rtcTrack;

            // Create a H264 depacketization handler and rtcp receiving session.
            m_pTrack1H264DepacketizationHandler = std::make_shared<rtc::H264RtpDepacketizer>(rtc::NalUnit::Separator::LongStartSequence);
            m_pTrack1RtcpReceivingSession       = std::make_shared<rtc::RtcpReceivingSession>();
            m_pTrack1H264DepacketizationHandler->addToChain(m_pTrack1RtcpReceivingSession);
            m_pVideoTrack1->setMediaHandler(m_pTrack1H264DepacketizationHandler);

            LOG_INFO(logging::g_qSharedLogger, "WebRTC camera {} Video Track Handler Configured. Waiting for frames...", m_szStreamerID);

            // Set the onMessage callback for the video track.
            m_pVideoTrack1->onFrame(
                [this](rtc::binary rtcBinaryMessage, rtc::FrameInfo rtcFrameInfo)
                {
                    // CRITICAL FIX: Ignore empty packets to prevent flushing the decoder.
                    if (rtcBinaryMessage.empty())
                    {
                        return;
                    }
                    ZoneScopedNC("WebRTC::onFrame", tracy::Color::Green);
                    ZoneName(m_szStreamerID.c_str(), m_szStreamerID.length());
                    ZoneValue(rtcFrameInfo.payloadType);
                    ZoneValue(rtcFrameInfo.timestamp);

                    // Prepare buffer for H.264 bytes.
                    std::vector<uint8_t> vH264EncodedBytes;
                    // Reserve space + FFmpeg Padding safety buffer.
                    vH264EncodedBytes.reserve(rtcBinaryMessage.size() + 16 + AV_INPUT_BUFFER_PADDING_SIZE);

                    if (rtcFrameInfo.payloadType == 96)
                    {
                        // Standard H264 Packet (Already Depacketized by LibDataChannel)
                        // It usually includes the Start Code (00 00 00 01) because of the Handler config.

                        const uint8_t* pData = reinterpret_cast<const uint8_t*>(rtcBinaryMessage.data());
                        vH264EncodedBytes.insert(vH264EncodedBytes.end(), pData, pData + rtcBinaryMessage.size());
                    }
                    else if (rtcFrameInfo.payloadType == 97)
                    {
                        // RTX (Retransmission) Packet
                        // Structure: [OSN (2 bytes)] [Original RTP Payload]
                        // This packet bypassed the Depacketizer, so it is "Raw".
                        // To decode it, we must strip the OSN and manually add the Start Code.

                        if (rtcBinaryMessage.size() <= 2)
                            return;    // Too small to contain data.

                        // Start code. (Long Start Sequence: 00 00 00 01)
                        vH264EncodedBytes.push_back(0);
                        vH264EncodedBytes.push_back(0);
                        vH264EncodedBytes.push_back(0);
                        vH264EncodedBytes.push_back(1);

                        // Original payload. (Skip first 2 bytes of RTX header)
                        const uint8_t* pData = reinterpret_cast<const uint8_t*>(rtcBinaryMessage.data());
                        vH264EncodedBytes.insert(vH264EncodedBytes.end(), pData + 2, pData + rtcBinaryMessage.size());
                    }
                    else
                    {
                        // Unknown payload type.
                        return;
                    }

                    // Zero-initialize padding bytes. (required by FFmpeg safety)
                    vH264EncodedBytes.insert(vH264EncodedBytes.end(), AV_INPUT_BUFFER_PADDING_SIZE, 0);

                    // Decode.
                    std::unique_lock lkDecoderLock(m_muDecoderMutex);
                    bool bDecoded = this->DecodeH264BytesToCVMat(vH264EncodedBytes, m_cvFrame, m_eOutputPixelFormat);

                    if (bDecoded && m_fnOnFrameReceivedCallback)
                    {
                        m_fnOnFrameReceivedCallback(m_cvFrame);
                    }
                    lkDecoderLock.unlock();
                });
        });

    m_pPeerConnection->onGatheringStateChange(
        [this](rtc::PeerConnection::GatheringState eGatheringState)
        {
            // Switch to translate the state to a string.
            switch (eGatheringState)
            {
                case rtc::PeerConnection::GatheringState::Complete:
                    LOG_DEBUG(logging::g_qSharedLogger, "Camera {} PeerConnection ICE gathering state changed to: Complete", m_szStreamerID);
                    break;
                case rtc::PeerConnection::GatheringState::InProgress:
                    LOG_DEBUG(logging::g_qSharedLogger, "Camera {} PeerConnection ICE gathering state changed to: InProgress", m_szStreamerID);
                    break;
                case rtc::PeerConnection::GatheringState::New:
                    LOG_DEBUG(logging::g_qSharedLogger, "Camera {} PeerConnection ICE gathering state changed to: New", m_szStreamerID);
                    break;
                default: LOG_DEBUG(logging::g_qSharedLogger, "Camera {} Peer connection ICE gathering state changed to: Unknown", m_szStreamerID); break;
            }
        });

    m_pPeerConnection->onIceStateChange(
        [this](rtc::PeerConnection::IceState eIceState)
        {
            // Switch to translate the state to a string.
            switch (eIceState)
            {
                case rtc::PeerConnection::IceState::Checking:
                    LOG_INFO(logging::g_qSharedLogger, "Camera {} PeerConnection ICE state changed to: Checking", m_szStreamerID);
                    break;
                case rtc::PeerConnection::IceState::Closed:
                    LOG_INFO(logging::g_qSharedLogger, "Camera {} PeerConnection ICE state changed to: Closed", m_szStreamerID);
                    break;
                case rtc::PeerConnection::IceState::Completed:
                    LOG_INFO(logging::g_qSharedLogger, "Camera {} PeerConnection ICE state changed to: Completed", m_szStreamerID);
                    break;
                case rtc::PeerConnection::IceState::Connected:
                    LOG_INFO(logging::g_qSharedLogger, "Camera {} PeerConnection ICE state changed to: Connected", m_szStreamerID);
                    break;
                case rtc::PeerConnection::IceState::Disconnected:
                    LOG_INFO(logging::g_qSharedLogger, "Camera {} PeerConnection ICE state changed to: Disconnected", m_szStreamerID);
                    break;
                case rtc::PeerConnection::IceState::Failed:
                    LOG_INFO(logging::g_qSharedLogger, "Camera {} PeerConnection ICE state changed to: Failed", m_szStreamerID);
                    break;
                case rtc::PeerConnection::IceState::New: LOG_INFO(logging::g_qSharedLogger, "Camera {} PeerConnection ICE state changed to: New", m_szStreamerID); break;
                default: LOG_INFO(logging::g_qSharedLogger, "Camera {} Peer connection ICE state changed to: Unknown", m_szStreamerID); break;
            }
        });
    m_pPeerConnection->onSignalingStateChange(
        [this](rtc::PeerConnection::SignalingState eSignalingState)
        {
            // Switch to translate the state to a string.
            switch (eSignalingState)
            {
                case rtc::PeerConnection::SignalingState::HaveLocalOffer:
                    LOG_INFO(logging::g_qSharedLogger, "Camera {} PeerConnection signaling state changed to: HaveLocalOffer", m_szStreamerID);
                    break;
                case rtc::PeerConnection::SignalingState::HaveLocalPranswer:
                    LOG_INFO(logging::g_qSharedLogger, "Camera {} PeerConnection signaling state changed to: HaveLocalPranswer", m_szStreamerID);
                    break;
                case rtc::PeerConnection::SignalingState::HaveRemoteOffer:
                    LOG_INFO(logging::g_qSharedLogger, "Camera {} PeerConnection signaling state changed to: HaveRemoteOffer", m_szStreamerID);
                    break;
                case rtc::PeerConnection::SignalingState::HaveRemotePranswer:
                    LOG_INFO(logging::g_qSharedLogger, "Camera {} PeerConnection signaling state changed to: HaveRemotePrAnswer", m_szStreamerID);
                    break;
                case rtc::PeerConnection::SignalingState::Stable:
                    LOG_INFO(logging::g_qSharedLogger, "Camera {} PeerConnection signaling state changed to: Stable", m_szStreamerID);
                    break;
                default: LOG_INFO(logging::g_qSharedLogger, "Camera {} Peer connection signaling state changed to: Unknown", m_szStreamerID); break;
            }
        });

    m_pPeerConnection->onStateChange(
        [this](rtc::PeerConnection::State eState)
        {
            // Switch to translate the state to a string.
            switch (eState)
            {
                case rtc::PeerConnection::State::Closed: LOG_INFO(logging::g_qSharedLogger, "Camera {} Peer connection state changed to: Closed", m_szStreamerID); break;
                case rtc::PeerConnection::State::Connected:
                    LOG_INFO(logging::g_qSharedLogger, "Camera {} Peer connection state changed to: Connected", m_szStreamerID);
                    break;
                case rtc::PeerConnection::State::Connecting:
                    LOG_INFO(logging::g_qSharedLogger, "Camera {} Peer connection state changed to: Connecting", m_szStreamerID);
                    break;
                case rtc::PeerConnection::State::Disconnected:
                    LOG_INFO(logging::g_qSharedLogger, "Camera {} Peer connection state changed to: Disconnected", m_szStreamerID);
                    break;
                case rtc::PeerConnection::State::Failed: LOG_INFO(logging::g_qSharedLogger, "Camera {} Peer connection state changed to: Failed", m_szStreamerID); break;
                case rtc::PeerConnection::State::New: LOG_INFO(logging::g_qSharedLogger, "Camera {} Peer connection state changed to: New", m_szStreamerID); break;
                default: LOG_INFO(logging::g_qSharedLogger, "Camera {} Peer connection state changed to: Unknown", m_szStreamerID); break;
            }
        });

    /////////////////////////////////////////////////////////////////////
    // Set some callbacks on important events for the data channel.
    /////////////////////////////////////////////////////////////////////

    m_pDataChannel->onOpen(
        [this]()
        {
            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "Camera {} WebRTC Data channel OPENED.", m_szStreamerID);

            // Request quality control of the stream.
            m_pDataChannel->send(std::string(1, static_cast<char>(1)));

            // --------------------------------------------------------
            // SEND INITIAL ENCODER CONFIGURATION
            // --------------------------------------------------------
            LOG_INFO(logging::g_qSharedLogger, "Camera {} WebRTC Sending Encoder Configuration to Simulator...", m_szStreamerID);

            // Set the QP factor. (0 = Lossless/Max Quality).
            this->SendCommandToStreamer("{\"Encoder.MaxQP\":" + std::to_string(constants::SIM_WEBRTC_QP) + "}");

            // Set bitrate limits.
            this->SendCommandToStreamer(R"({"WebRTC.MinBitrate":100000})");
            this->SendCommandToStreamer(R"({"WebRTC.MaxBitrate":100000000})");

            // Set FPS.
            this->SendCommandToStreamer(R"({"WebRTC.Fps":60})");
            this->SendCommandToStreamer(R"({"WebRTC.MaxFps":60})");

            // Target Bitrate. (-1 = Use Max/Unlimited)
            this->SendCommandToStreamer(R"({"Encoder.TargetBitrate":-1})");
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
                    LOG_DEBUG(logging::g_qSharedLogger, "Camera {} DATA_CHANNEL Received message from peer: {}", m_szStreamerID, szMessage);
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
                    LOG_DEBUG(logging::g_qSharedLogger,
                              "Camera {} DATA_CHANNEL Received binary data ({} bytes): {}",
                              m_szStreamerID,
                              rtcBinaryData.size(),
                              szBinaryDataStr);
                }
                else
                {
                    LOG_ERROR(logging::g_qSharedLogger, "Camera {} Received unknown message type from peer", m_szStreamerID);
                }
            }
            catch (const std::exception& e)
            {
                // Submit logger message.
                LOG_ERROR(logging::g_qSharedLogger, "Camera {} Error occurred while negotiating with the datachannel: {}", m_szStreamerID, e.what());
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
    av_log_set_level(AV_LOG_QUIET);

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
    av_opt_set_int(m_pAVCodecContext, "threads", 1, 0);    // Single threaded is optimal for decoding H.264

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

    // Set the SWSScale context to nullptr.
    m_pSWSContext = nullptr;

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
    // Safety check
    if (vH264EncodedBytes.empty())
        return false;

    ZoneScopedC(tracy::Color::Green2);

    // Use the actual data size, excluding the padding we added in onFrame.
    size_t nDataSize = vH264EncodedBytes.size() - AV_INPUT_BUFFER_PADDING_SIZE;

    // Initialize packet data.
    m_pPacket->data = const_cast<uint8_t*>(vH264EncodedBytes.data());
    m_pPacket->size = static_cast<int>(nDataSize);    // Tell FFmpeg the real size.

    // Send the packet to the decoder.
    int nReturnCode = avcodec_send_packet(m_pAVCodecContext, m_pPacket);
    if (nReturnCode < 0)
    {
        // Get the error message.
        char aErrorBuffer[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(nReturnCode, aErrorBuffer, AV_ERROR_MAX_STRING_SIZE);
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "WebRTC camera {} FFMPEG send_packet failed. Error: {} {}", m_szStreamerID, nReturnCode, aErrorBuffer);
        // Request a new keyframe from the video track.
        this->RequestKeyFrame();

        return false;
    }

    // Receive decoded frames in a loop
    while (true)
    {
        ZoneScopedNC("Decode Chunk", tracy::Color::Green3);
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
            if (m_pSWSContext == nullptr)
            {
                LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} Initializing SwsContext...", m_szStreamerID);
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
                if (m_pSWSContext == nullptr)
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "Failed to initialize SwsContext!");
                    // Request a new keyframe from the video track.
                    this->RequestKeyFrame();

                    return false;
                }
            }

            // Create new mat for the decoded frame.
            cvDecodedFrame.create(m_pFrame->height, m_pFrame->width, CV_8UC3);
            std::array<uint8_t*, 4> aDest    = {cvDecodedFrame.data, nullptr, nullptr, nullptr};
            std::array<int, 4> aDestLinesize = {static_cast<int>(cvDecodedFrame.step[0]), 0, 0, 0};

            // Convert the frame to the output pixel format.
            sws_scale(m_pSWSContext, m_pFrame->data, m_pFrame->linesize, 0, m_pFrame->height, aDest.data(), aDestLinesize.data());
        }
    }

    return true;
}

/******************************************************************************
 * @brief Requests a key frame from the given video track. This is useful for when the
 * video track is out of sync or has lost frames.
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
 * The command is a JSON string that is sent as a binary message.
 * The PixelStreaming plugin handles the command very weirdly, so we have to
 * sort of encode the command in a specific way. This handles that encoding.
 *
 * @param szCommand - The command to send to the streamer.
 * @return true - Command was successfully sent.
 * @return false - Command was not successfully sent.
 *
 * @note This will only work for valid COMMANDS with ID of type 51. Check the
 * PixelStreamingInfrastructure repo for more information.
 * https://github.com/EpicGamesExt/PixelStreamingInfrastructure/blob/13ce022d3a09d315d4ca85c05b61a8d3fe92741c/Extras/JSStreamer/src/protocol.ts#L196
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-01
 ******************************************************************************/
bool WebRTC::SendCommandToStreamer(const std::string& szCommand)
{
    if (!m_pDataChannel)
        return false;
    if (szCommand.empty())
        return false;

    // Command ID 51.
    std::string szID(1, static_cast<char>(51));

    // Calculate total payload size in BYTES. (UTF-16 = size * 2)
    uint16_t u16Size = static_cast<uint16_t>(szCommand.size() * 2);

    rtc::binary rtcBinaryMessage;
    rtcBinaryMessage.push_back(static_cast<std::byte>(szID[0]));

    // Send Size as 16-bit integer. (Little Endian)
    rtcBinaryMessage.push_back(static_cast<std::byte>(u16Size & 0xFF));           // Low byte
    rtcBinaryMessage.push_back(static_cast<std::byte>((u16Size >> 8) & 0xFF));    // High byte

    // Send payload as UTF-16 Little Endian.
    for (char cLetter : szCommand)
    {
        rtcBinaryMessage.push_back(static_cast<std::byte>(cLetter));
        rtcBinaryMessage.push_back(static_cast<std::byte>(0));
    }

    LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} Sending Data Channel Command: {}", m_szStreamerID, szCommand);
    return m_pDataChannel->send(rtcBinaryMessage);
}
