/******************************************************************************
 * @brief Implements the SIMCam class.
 *
 * @file SIMCam.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "SIMCam.h"
#include "../../AutonomyConstants.h"
#include "../../AutonomyLogging.h"

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
 * @param nNumFrameRetrievalThreads - The number of threads to use for frame queueing and copying.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
SIMCam::SIMCam(const std::string szCameraPath,
               const int nPropResolutionX,
               const int nPropResolutionY,
               const int nPropFramesPerSecond,
               const PIXEL_FORMATS ePropPixelFormat,
               const double dPropHorizontalFOV,
               const double dPropVerticalFOV,
               const int nNumFrameRetrievalThreads) :
    Camera(nPropResolutionX, nPropResolutionY, nPropFramesPerSecond, ePropPixelFormat, dPropHorizontalFOV, dPropVerticalFOV)
{
    // Assign member variables.
    m_szCameraPath              = szCameraPath;
    m_nCameraIndex              = -1;
    m_nNumFrameRetrievalThreads = nNumFrameRetrievalThreads;

    // Set flag specifying that the camera is located at a dev/video index.
    m_bCameraIsConnectedOnVideoIndex = false;

    // Attempt to open camera with OpenCV's VideoCapture and print if successfully opened or not.
    if (m_cvCamera.open(szCameraPath))
    {
        // Submit logger message.
        LOG_DEBUG(g_qSharedLogger, "SIMCamera {} at path/URL {} has been successfully opened.", m_cvCamera.getBackendName(), m_szCameraPath);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(g_qSharedLogger, "Unable to open SIMCamera at path/URL {}", m_szCameraPath);
    }
}

/******************************************************************************
 * @brief Destroy the SIM Cam:: SIM Cam object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
SIMCam::~SIMCam()
{
    // Stop threaded code.
    this->RequestStop();
    this->Join();

    // Release camera capture object.
    m_cvCamera.release();
}

/******************************************************************************
 * @brief The code inside this private method runs in a seperate thread, but still
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
void SIMCam::ThreadedContinuousCode()
{
    // Check if camera is NOT open.
    if (!m_cvCamera.isOpened())
    {
        // Shutdown threads for this SIMCam.
        this->RequestStop();

        // Submit logger message.
        LOG_CRITICAL(g_qSharedLogger,
                     "Camera start was attempted for camera at {}/{}, but camera never properly opened or it has become disconnected!",
                     m_nCameraIndex,
                     m_szCameraPath);
    }
    else
    {
        // TODO: PUT CODE HERE FOR GETTING FRAMES AND DATA FROM SIMULATOR.

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
void SIMCam::PooledLinearCode()
{
    // Aqcuire mutex for getting frames out of the queue.
    std::unique_lock<std::mutex> lkFrameQueue(m_muFrameCopyMutex);
    // Check if the queue is empty.
    if (!m_qFrameCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::FrameFetchContainer<cv::Mat> stContainer = m_qFrameCopySchedule.front();
        // Pop out of queue.
        m_qFrameCopySchedule.pop();
        // Release lock.
        lkFrameQueue.unlock();

        // Copy frame to data container.
        *(stContainer.pFrame) = m_cvFrame.clone();
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
 *                          Value will be true if frame was succesfully retrieved.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
std::future<bool> SIMCam::RequestFrameCopy(cv::Mat& cvFrame)
{
    // Assemble the FrameFetchContainer.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvFrame, m_ePropPixelFormat);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append frame fetch container to the scedule queue.
    m_qFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

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
bool SIMCam::GetCameraIsOpen()
{
    // Get camera status from OpenCV.
    return m_cvCamera.isOpened();
}

/******************************************************************************
 * @brief Accessor for the cameras path or video index.
 *
 * @tparam T - Either an int or string depending on if the camera is located at a
 *          hardware path or a dev/video index.
 * @return T - The path or index of the camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
template<typename T>
T SIMCam::GetCameraLocation() const
{
    // Check if camera location is a hardware path or video index.
    if (m_bCameraIsConnectedOnVideoIndex)
    {
        // If video index, return index integer.
        return m_nCameraIndex;
    }
    else
    {
        // If video path, return path string.
        return m_szCameraPath;
    }
}
