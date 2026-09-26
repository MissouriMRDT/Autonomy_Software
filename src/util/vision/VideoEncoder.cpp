/******************************************************************************
 * @brief Implements the VideoEncoder class.
 *
 * @file VideoEncoder.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-09-26
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#include "VideoEncoder.h"
#include "../../AutonomyLogging.h"

/// \cond
extern "C"
{
#include <libavutil/error.h>
#include <libavutil/opt.h>
}

/// \endcond

/******************************************************************************
 * @brief Destroy the VideoEncoder object. Flushes and finalizes the file if it is
 *      still open.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-09-26
 ******************************************************************************/
VideoEncoder::~VideoEncoder()
{
    // Flush the encoder, write the trailer and free everything.
    this->Close();
}

/******************************************************************************
 * @brief Open a new H.264 video file. The container is chosen from the file extension.
 *
 * @param szFilePath - The path of the video file to create.
 * @param cvFrameSize - The resolution of the video. Frames of another size are scaled to it.
 * @param nFPS - The frame rate of the video. Every Write() advances the timestamp by one frame.
 * @param szPreset - The x264 preset (ultrafast, superfast, veryfast, faster, fast, medium, ...).
 * @param nThreads - The number of threads x264 may use. 0 lets x264 pick (one per core).
 * @return true - The file was created and the encoder is ready.
 * @return false - Something failed. The reason was logged and the encoder stays closed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-09-26
 ******************************************************************************/
bool VideoEncoder::Open(const std::string& szFilePath, const cv::Size& cvFrameSize, const int nFPS, const std::string& szPreset, const int nThreads)
{
    // Start from a clean state if this encoder was used before.
    this->Close();

    // H.264 needs even dimensions for 4:2:0 chroma.
    if (cvFrameSize.width <= 0 || cvFrameSize.height <= 0 || cvFrameSize.width % 2 != 0 || cvFrameSize.height % 2 != 0 || nFPS <= 0)
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "VideoEncoder: Invalid size {}x{} or FPS {} for {}.", cvFrameSize.width, cvFrameSize.height, nFPS, szFilePath);
        return false;
    }

    // Find the libx264 encoder.
    const AVCodec* pCodec = avcodec_find_encoder_by_name("libx264");
    if (pCodec == nullptr)
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "VideoEncoder: FFmpeg was built without libx264. Cannot record {}.", szFilePath);
        return false;
    }

    // Create the output container from the file extension.
    if (avformat_alloc_output_context2(&m_pFormatContext, nullptr, nullptr, szFilePath.c_str()) < 0 || m_pFormatContext == nullptr)
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "VideoEncoder: Could not pick a container format for {}.", szFilePath);
        this->Close();
        return false;
    }

    // Configure the encoder. CRF 23 and the default GOP match what cv::VideoWriter produced before.
    m_pCodecContext               = avcodec_alloc_context3(pCodec);
    m_pCodecContext->width        = cvFrameSize.width;
    m_pCodecContext->height       = cvFrameSize.height;
    m_pCodecContext->pix_fmt      = AV_PIX_FMT_YUV420P;
    m_pCodecContext->time_base    = AVRational{1, nFPS};
    m_pCodecContext->framerate    = AVRational{nFPS, 1};
    m_pCodecContext->thread_count = nThreads;
    av_opt_set(m_pCodecContext->priv_data, "preset", szPreset.c_str(), 0);
    av_opt_set(m_pCodecContext->priv_data, "crf", "23", 0);
    // Containers like MKV and MP4 want the SPS/PPS in the stream header instead of in the bitstream.
    if (m_pFormatContext->oformat->flags & AVFMT_GLOBALHEADER)
    {
        m_pCodecContext->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    // Open the encoder.
    int nReturnCode = avcodec_open2(m_pCodecContext, pCodec, nullptr);
    if (nReturnCode < 0)
    {
        // Get the error message.
        char aErrorBuffer[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(nReturnCode, aErrorBuffer, AV_ERROR_MAX_STRING_SIZE);
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "VideoEncoder: Could not open libx264 for {}: {}", szFilePath, aErrorBuffer);
        this->Close();
        return false;
    }

    // Create the video stream and copy the encoder parameters into it.
    m_pStream                 = avformat_new_stream(m_pFormatContext, nullptr);
    m_pStream->time_base      = m_pCodecContext->time_base;
    m_pStream->avg_frame_rate = m_pCodecContext->framerate;
    avcodec_parameters_from_context(m_pStream->codecpar, m_pCodecContext);

    // Open the file and write the container header.
    if (!(m_pFormatContext->oformat->flags & AVFMT_NOFILE) && avio_open(&m_pFormatContext->pb, szFilePath.c_str(), AVIO_FLAG_WRITE) < 0)
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "VideoEncoder: Could not create the file {}.", szFilePath);
        this->Close();
        return false;
    }
    if (avformat_write_header(m_pFormatContext, nullptr) < 0)
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "VideoEncoder: Could not write the header of {}.", szFilePath);
        this->Close();
        return false;
    }
    m_bHeaderWritten = true;

    // Allocate the reusable YUV frame and packet.
    m_pFrame         = av_frame_alloc();
    m_pFrame->format = m_pCodecContext->pix_fmt;
    m_pFrame->width  = m_pCodecContext->width;
    m_pFrame->height = m_pCodecContext->height;
    m_pPacket        = av_packet_alloc();
    if (av_frame_get_buffer(m_pFrame, 0) < 0 || m_pPacket == nullptr)
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "VideoEncoder: Out of memory opening {}.", szFilePath);
        this->Close();
        return false;
    }

    return true;
}

/******************************************************************************
 * @brief Convert one frame to YUV420P and encode it. The frame is only read, so a
 *      published, shared snapshot can be passed directly.
 *
 * @param cvFrame - A CV_8UC1 (gray), CV_8UC3 (BGR) or CV_8UC4 (BGRA) frame.
 * @return true - The frame was handed to the encoder.
 * @return false - The encoder is closed, the frame is empty or of an unsupported type, or encoding failed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-09-26
 ******************************************************************************/
bool VideoEncoder::Write(const cv::Mat& cvFrame)
{
    // Nothing to do without an open encoder or a frame.
    if (m_pCodecContext == nullptr || m_pFrame == nullptr || cvFrame.empty() || cvFrame.depth() != CV_8U)
    {
        return false;
    }

    // Map the OpenCV channel layout to the matching FFmpeg pixel format.
    AVPixelFormat eSourceFormat;
    switch (cvFrame.channels())
    {
        case 1: eSourceFormat = AV_PIX_FMT_GRAY8; break;
        case 3: eSourceFormat = AV_PIX_FMT_BGR24; break;
        case 4: eSourceFormat = AV_PIX_FMT_BGRA; break;
        default: return false;
    }

    // Get a converter for this input. It is only rebuilt when the input format or size changes.
    m_pSWSContext = sws_getCachedContext(m_pSWSContext,
                                         cvFrame.cols,
                                         cvFrame.rows,
                                         eSourceFormat,
                                         m_pCodecContext->width,
                                         m_pCodecContext->height,
                                         m_pCodecContext->pix_fmt,
                                         SWS_BILINEAR,
                                         nullptr,
                                         nullptr,
                                         nullptr);
    if (m_pSWSContext == nullptr)
    {
        return false;
    }

    // The encoder may still hold references to the previous frame's buffers.
    if (av_frame_make_writable(m_pFrame) < 0)
    {
        return false;
    }

    // Convert straight from the caller's buffer into the encoder's frame.
    const uint8_t* aSource[1]  = {cvFrame.data};
    const int aSourceStride[1] = {static_cast<int>(cvFrame.step[0])};
    sws_scale(m_pSWSContext, aSource, aSourceStride, 0, cvFrame.rows, m_pFrame->data, m_pFrame->linesize);

    // Encode it. Each frame advances the timestamp by one frame period.
    m_pFrame->pts = m_llNextPTS++;
    if (avcodec_send_frame(m_pCodecContext, m_pFrame) < 0)
    {
        return false;
    }

    // Write whatever packets the encoder has finished.
    return this->WritePendingPackets();
}

/******************************************************************************
 * @brief Move every packet the encoder has finished into the file.
 *
 * @return true - All available packets were written.
 * @return false - The encoder or muxer reported an error.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-09-26
 ******************************************************************************/
bool VideoEncoder::WritePendingPackets()
{
    while (true)
    {
        // Get the next finished packet.
        int nReturnCode = avcodec_receive_packet(m_pCodecContext, m_pPacket);
        if (nReturnCode == AVERROR(EAGAIN) || nReturnCode == AVERROR_EOF)
        {
            // The encoder needs more input, or it is fully flushed.
            return true;
        }
        else if (nReturnCode < 0)
        {
            return false;
        }

        // Convert the timestamps from frame units to the container's units and write the packet.
        av_packet_rescale_ts(m_pPacket, m_pCodecContext->time_base, m_pStream->time_base);
        m_pPacket->stream_index = m_pStream->index;
        nReturnCode             = av_interleaved_write_frame(m_pFormatContext, m_pPacket);
        if (nReturnCode < 0)
        {
            return false;
        }
    }
}

/******************************************************************************
 * @brief Flush the encoder, finalize the file and free everything. Safe to call
 *      more than once, and on an encoder that never opened.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-09-26
 ******************************************************************************/
void VideoEncoder::Close()
{
    // Drain the encoder and finalize the file, but only if the header was written.
    if (m_bHeaderWritten && m_pPacket != nullptr)
    {
        // A null frame tells the encoder to flush its delayed frames.
        avcodec_send_frame(m_pCodecContext, nullptr);
        this->WritePendingPackets();
        // Write the container index. Without it, seeking in the file does not work.
        av_write_trailer(m_pFormatContext);
    }

    // Close the file.
    if (m_pFormatContext != nullptr && !(m_pFormatContext->oformat->flags & AVFMT_NOFILE) && m_pFormatContext->pb != nullptr)
    {
        avio_closep(&m_pFormatContext->pb);
    }

    // Free the FFmpeg objects. Each of these accepts null.
    sws_freeContext(m_pSWSContext);
    av_frame_free(&m_pFrame);
    av_packet_free(&m_pPacket);
    avcodec_free_context(&m_pCodecContext);
    avformat_free_context(m_pFormatContext);

    // Reset the state so IsOpen() reports closed and Open() can be called again.
    m_pSWSContext    = nullptr;
    m_pFormatContext = nullptr;
    m_pStream        = nullptr;
    m_llNextPTS      = 0;
    m_bHeaderWritten = false;
}

/******************************************************************************
 * @brief Check whether the encoder has an open file.
 *
 * @return true - Open() succeeded and Close() has not been called since.
 * @return false - The encoder is closed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-09-26
 ******************************************************************************/
bool VideoEncoder::IsOpen() const
{
    return m_pCodecContext != nullptr && m_pFrame != nullptr;
}
