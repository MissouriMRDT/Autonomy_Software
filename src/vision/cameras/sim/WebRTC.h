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

/// \cond
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <rtc/rtc.hpp>

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
        WebRTC(const std::string& szSignallingServerURL);
        ~WebRTC();

        // Setter for the frame received callback.
        void SetOnFrameReceivedCallback(std::function<void(cv::Mat&)> fnOnFrameReceivedCallback);

    private:
        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////
        bool ConnectToSignallingServer(const std::string& szSignallingServerURL);
        bool DecodeH264BytesToCVMat(const std::vector<uint8_t>& vH264EncodedBytes, cv::Mat& cvDecodedFrame);
        bool RequestKeyFrame(std::shared_ptr<rtc::Track> pVideoTrack);

        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        // Normal member variables.
        std::string m_szSignallingServerURL;

        // WebRTC connection to RoveSoSimulator Pixel Streamer.
        std::shared_ptr<rtc::WebSocket> m_pWebSocket;
        std::shared_ptr<rtc::PeerConnection> m_pPeerConnection;
        std::shared_ptr<rtc::DataChannel> m_pDataChannel;
        std::shared_ptr<rtc::Track> m_pVideoTrack1;
        std::shared_ptr<rtc::H264RtpDepacketizer> m_pTrack1H264DepacketizationHandler;
        std::shared_ptr<rtc::RtcpReceivingSession> m_pTrack1RTCPReceivingSession;
        std::chrono::system_clock::time_point m_tmLastKeyFrameRequestTime;

        // AV codec context for decoding H264.
        AVCodecContext* m_pAVCodecContext;
        SwsContext* m_avSWSContext;

        // OpenCV Mat for storing the frame.
        cv::Mat m_cvFrame;

        // Callback function for when a new frame is received.
        std::function<void(cv::Mat&)> m_fnOnFrameReceivedCallback;
};
#endif
