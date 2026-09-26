/******************************************************************************
 * @brief Defines the VideoEncoder class, a small H.264 file writer built directly
 *      on FFmpeg so the x264 preset and thread count can be controlled.
 *
 * @file VideoEncoder.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-09-26
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#ifndef VIDEO_ENCODER_H
#define VIDEO_ENCODER_H

/// \cond
#include <opencv2/opencv.hpp>
#include <string>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

/// \endcond

/******************************************************************************
 * @brief Writes 8-bit OpenCV frames to an H.264 (libx264) video file.
 *
 *      This replaces cv::VideoWriter for recordings. cv::VideoWriter opens libx264 with
 *      its default "medium" preset and one thread per core, and this OpenCV build offers
 *      no working way to change either. On 720p sim footage "veryfast" costs 65% less CPU
 *      than "medium" and produces a 30% smaller file at the same CRF.
 *
 *      Write() accepts 1 channel (gray), 3 channel (BGR) and 4 channel (BGRA) frames and
 *      converts them straight into the encoder's YUV420P frame, so callers can pass a
 *      read-only published snapshot without copying or converting it first. A frame whose
 *      size differs from the size given to Open() is scaled to fit.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-09-26
 ******************************************************************************/
class VideoEncoder
{
    public:
        /////////////////////////////////////////
        // Declare public methods.
        /////////////////////////////////////////

        VideoEncoder() = default;
        ~VideoEncoder();
        VideoEncoder(const VideoEncoder&)            = delete;
        VideoEncoder& operator=(const VideoEncoder&) = delete;

        bool Open(const std::string& szFilePath, const cv::Size& cvFrameSize, const int nFPS, const std::string& szPreset, const int nThreads);
        bool Write(const cv::Mat& cvFrame);
        void Close();
        bool IsOpen() const;

    private:
        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        bool WritePendingPackets();

        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        AVFormatContext* m_pFormatContext = nullptr;
        AVCodecContext* m_pCodecContext   = nullptr;
        AVStream* m_pStream               = nullptr;
        AVFrame* m_pFrame                 = nullptr;
        AVPacket* m_pPacket               = nullptr;
        SwsContext* m_pSWSContext         = nullptr;
        long long m_llNextPTS             = 0;
        bool m_bHeaderWritten             = false;
};

#endif
