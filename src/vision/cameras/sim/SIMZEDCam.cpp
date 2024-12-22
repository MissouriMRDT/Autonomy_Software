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

    // Construct camera stream objects. Append proper camera path arguments to each URL camera path.
    m_pRGBStream          = std::make_unique<WebRTC>(szCameraPath, "ZEDFrontRGB");
    m_pDepthImageStream   = std::make_unique<WebRTC>(szCameraPath, "ZEDFrontDepthImage");
    m_pDepthMeasureStream = std::make_unique<WebRTC>(szCameraPath, "ZEDFrontDepthMeasure");

    // Set the frame callbacks.
    m_pRGBStream->SetOnFrameReceivedCallback(
        [this](cv::Mat& cvFrame)
        {
            // Acquire a lock on the webRTC copy mutex.
            std::unique_lock<std::shared_mutex> lkWebRTC(m_muWebRTCRGBImageCopyMutex);
            // Deep copy the frame.
            m_cvFrame = cvFrame.clone();
        });
    m_pDepthImageStream->SetOnFrameReceivedCallback(
        [this](cv::Mat& cvFrame)
        {
            // Acquire a lock on the webRTC copy mutex.
            std::unique_lock<std::shared_mutex> lkWebRTC(m_muWebRTCDepthImageCopyMutex);
            // Deep copy the frame.
            m_cvDepthImage = cvFrame.clone();
        });
    m_pDepthMeasureStream->SetOnFrameReceivedCallback(
        [this](cv::Mat& cvFrame)
        {
            // Acquire a lock on the webRTC copy mutex.
            std::unique_lock<std::shared_mutex> lkWebRTC(m_muWebRTCDepthMeasureCopyMutex);

            // Deep copy the frame.
            m_cvDepthBuffer = cvFrame.clone();
            // Check if m_cvDepthMeasure is the correct size and type.
            if (m_cvDepthMeasure.empty() || m_cvDepthMeasure.size() != m_cvDepthBuffer.size() || m_cvDepthMeasure.type() != CV_16UC1)
            {
                m_cvDepthMeasure = cv::Mat(m_cvDepthBuffer.size(), CV_16UC1);
            }
            // The Simulator uses this a special method of packing the depth measure data as defined in this paper.
            // http://reality.cs.ucl.ac.uk/projects/depth-streaming/depth-streaming.pdf
            // Here we will decode it.
            float w  = 65536.0;
            float np = 512.0;

            // Iterate over each pixel in the cvDepthMeasure image
            for (int y = 0; y < m_cvDepthBuffer.rows; ++y)
            {
                for (int x = 0; x < m_cvDepthBuffer.cols; ++x)
                {
                    // Extract the encoded depth values
                    cv::Vec3b cvEncodedDepth = m_cvDepthBuffer.at<cv::Vec3b>(y, x);

                    // Extract encoded values
                    float L  = cvEncodedDepth[2] / 255.0;
                    float Ha = cvEncodedDepth[1] / 255.0;
                    float Hb = cvEncodedDepth[0] / 255.0;

                    // Period for triangle waves
                    float p = np / w;

                    // Determine offset and fine-grain correction
                    int m    = fmod((4.0 * (L / p)) - 0.5, 4.0);
                    float L0 = L - fmod(L - (p / 8.0), p) + ((p / 4.0) * m) - (p / 8.0);

                    float delta;
                    if (m == 0)
                        delta = (p / 2.0) * Ha;
                    else if (m == 1)
                        delta = (p / 2.0) * Hb;
                    else if (m == 2)
                        delta = (p / 2.0) * (1.0 - Ha);
                    else if (m == 3)
                        delta = (p / 2.0) * (1.0 - Hb);

                    // Combine to compute the original depth
                    float depth = w * (L0 + delta);

                    // Store the decoded depth in the new cv::Mat
                    m_cvDepthMeasure.at<uint16_t>(y, x) = static_cast<uint16_t>(depth);
                }
            }
        });

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
    // Acquire a shared_lock on the frame copy queue.
    std::shared_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Check if the frame copy queue is empty.
    if (!m_qFrameCopySchedule.empty())
    {
        // Acquire shared lock on the WebRTC mutex, so that the WebRTC connection doesn't try to write to the Mats while they are being copied in the thread pool.
        std::shared_lock<std::shared_mutex> lkWebRTC(m_muWebRTCRGBImageCopyMutex);
        std::shared_lock<std::shared_mutex> lkWebRTC2(m_muWebRTCDepthImageCopyMutex);
        std::shared_lock<std::shared_mutex> lkWebRTC3(m_muWebRTCDepthMeasureCopyMutex);

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
