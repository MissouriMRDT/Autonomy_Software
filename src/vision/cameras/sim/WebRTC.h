/******************************************************************************
 * @brief Defines the WebRTC class.
 *
 * @file WebRTC.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-11-30
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef WEBRTC_H
#define WEBRTC_H

#include "../../../AutonomyConstants.h"
#include "../../../util/threading/RetryTimer.hpp"
#include "../../../util/threading/ThreadRegistry.hpp"

/// \cond
#include <condition_variable>
#include <deque>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <rtc/rtc.hpp>
#include <thread>
#include <tracy/Tracy.hpp>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/imgutils.h>
#include <libavutil/mem.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

/// \endcond

/******************************************************************************
 * @brief A media handler that drops every RTP packet that is not plain H264 before it
 *      reaches the depacketizer. The simulator also sends RTX packets (retransmissions and
 *      bandwidth probes, which repeat old packets). We never request retransmissions, so
 *      they are always duplicates, and mixed into the depacketizer they corrupt the frame
 *      being assembled.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-09-25
 ******************************************************************************/
class H264PayloadFilter final : public rtc::MediaHandler
{
    public:
        explicit H264PayloadFilter(const rtc::Description::Media& rtcMediaDescription);
        void incoming(rtc::message_vector& vMessages, const rtc::message_callback& fnSend) override;

    private:
        std::vector<int> m_vH264PayloadTypes;    // The payload types the track negotiated for H264.
};

/******************************************************************************
 * @brief This class is used to establish a connection with the RoveSoSimulator
 *      and retrieve video streams from it.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-11-30
 ******************************************************************************/
class WebRTC
{
    public:
        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////
        WebRTC(const std::string& szSignallingServerURL, const std::string& szStreamerID);
        ~WebRTC();
        void CloseConnection();

        // Setter for the frame received callback.
        void SetOnFrameReceivedCallback(std::function<void(cv::Mat&)> fnOnFrameReceivedCallback, const AVPixelFormat eOutputPixelFormat = AV_PIX_FMT_BGR24);
        bool GetIsConnected() const;

    private:
        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////
        bool ConnectToSignallingServer(const std::string& szSignallingServerURL);
        void QueueEncodedFrame(std::vector<uint8_t>&& vH264EncodedBytes);
        void DecodeThread(std::stop_token stStopToken);
        bool InitializeH264Decoder();
        bool DecodeH264BytesToCVMat(const std::vector<uint8_t>& vH264EncodedBytes, cv::Mat& cvDecodedFrame, const AVPixelFormat eOutputPixelFormat);
        bool RequestKeyFrame();
        bool SendCommandToStreamer(const std::string& szCommand);

        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        // Normal member variables.
        std::string m_szSignallingServerURL;
        std::string m_szStreamerID;

        // WebRTC connection to RoveSoSimulator Pixel Streamer.
        std::shared_ptr<rtc::WebSocket> m_pWebSocket;
        std::shared_ptr<rtc::PeerConnection> m_pPeerConnection;
        std::shared_ptr<rtc::DataChannel> m_pDataChannel;
        std::shared_ptr<rtc::Track> m_pVideoTrack1;
        std::shared_ptr<rtc::H264RtpDepacketizer> m_pTrack1H264DepacketizationHandler;
        std::shared_ptr<rtc::RtcpReceivingSession> m_pTrack1RtcpReceivingSession;
        threadutils::RetryTimer m_tmKeyFrameRequestTimer{constants::SIM_STREAM_KEYFRAME_REQUEST_INTERVAL};

        // AV codec context for decoding H264.
        AVCodecContext* m_pAVCodecContext  = nullptr;
        AVFrame* m_pFrame                  = nullptr;
        AVPacket* m_pPacket                = nullptr;
        SwsContext* m_pSWSContext          = nullptr;
        AVPixelFormat m_eOutputPixelFormat = AV_PIX_FMT_BGR24;
        threadutils::RetryTimer m_tmDecodeWarnTimer{constants::SIM_STREAM_DECODE_WARN_INTERVAL};
        bool m_bAwaitingKeyFrame = true;    // Output is held back until an intact keyframe restarts the stream. Decoder thread only.

        // Encoded frames handed from the network thread to the decoder thread. Decoding on the network thread
        // stalls socket reads for every stream, so the kernel drops packets and the video corrupts.
        std::mutex m_muEncodedFramesMutex;
        std::condition_variable_any m_cvEncodedFramesReady;
        std::deque<std::vector<uint8_t>> m_dqEncodedFrames;
        bool m_bFlushDecoder = false;
        threadutils::ThreadTelemetry m_stDecodeTelemetry;

        // OpenCV Mat for storing the frame.
        cv::Mat m_cvFrame;

        // Callback function for when a new frame is received.
        std::function<void(cv::Mat&)> m_fnOnFrameReceivedCallback;

        // The destructor stops and joins this before freeing the decoder. Declared last so it is also joined first on any other exit.
        std::jthread m_thDecoder;
};
#endif
