/******************************************************************************
 * @brief Implements the BasicCam class.
 *
 * @file BasicCam.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-19
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "BasicCam.h"
#include "../../AutonomyConstants.h"
#include "../../AutonomyLogging.h"

/******************************************************************************
 * @brief Construct a new Basic Cam:: Basic Cam object.
 *
 * @param szCameraPath - The file path to the camera hardware.
 * @param nPropResolutionX - X res of camera.
 * @param nPropResolutionY - Y res of camera.
 * @param nPropFramesPerSecond - FPS camera is running at.
 * @param ePropPixelFormat - The pixel layout/format of the image.
 * @param dPropHorizontalFOV - The horizontal field of view.
 * @param dPropVerticalFOV - The vertical field of view.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
BasicCam::BasicCam(const std::string szCameraPath,
                   const int nPropResolutionX,
                   const int nPropResolutionY,
                   const int nPropFramesPerSecond,
                   const PIXEL_FORMATS ePropPixelFormat,
                   const double dPropHorizontalFOV,
                   const double dPropVerticalFOV) :
    Camera(nPropResolutionX, nPropResolutionY, nPropFramesPerSecond, ePropPixelFormat, dPropHorizontalFOV, dPropVerticalFOV)
{
    // Assign member variables.
    m_szCameraPath = szCameraPath;
    m_nCameraIndex = -1;

    // Set flag specifying that the camera is located at a dev/video index.
    m_bCameraIsConnectedOnVideoIndex = false;

    // Attempt to open camera with OpenCV's VideoCapture and print if successfully opened or not.
    if (m_cvCamera.open(szCameraPath))
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "Camera {} at path/URL {} has been successfully opened.", m_cvCamera.getBackendName(), m_szCameraPath);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "Unable to open camera at path/URL {}", m_szCameraPath);
    }
}

/******************************************************************************
 * @brief Construct a new Basic Cam:: Basic Cam object. Overloaded for dev/video
 *      indexes.
 *
 * @param nCameraIndex - The video index that the camera is connected on.
 * @param nPropResolutionX - X res of camera.
 * @param nPropResolutionY - Y res of camera.
 * @param nPropFramesPerSecond - FPS camera is running at.
 * @param ePropPixelFormat - The pixel layout/format of the image.
 * @param dPropHorizontalFOV - The horizontal field of view.
 * @param dPropVerticalFOV - The vertical field of view.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
BasicCam::BasicCam(const int nCameraIndex,
                   const int nPropResolutionX,
                   const int nPropResolutionY,
                   const int nPropFramesPerSecond,
                   const PIXEL_FORMATS ePropPixelFormat,
                   const double dPropHorizontalFOV,
                   const double dPropVerticalFOV) :
    Camera(nPropResolutionX, nPropResolutionY, nPropFramesPerSecond, ePropPixelFormat, dPropHorizontalFOV, dPropVerticalFOV)
{
    // Assign member variables.
    m_nCameraIndex = nCameraIndex;
    m_szCameraPath = "";

    // Set flag specifying that the camera is located at a dev/video index.
    m_bCameraIsConnectedOnVideoIndex = true;

    // Set video cap properties.
    m_cvCamera.set(cv::CAP_PROP_FRAME_WIDTH, nPropResolutionX);
    m_cvCamera.set(cv::CAP_PROP_FRAME_HEIGHT, nPropResolutionY);
    m_cvCamera.set(cv::CAP_PROP_FPS, nPropFramesPerSecond);

    // Attempt to open camera with OpenCV's VideoCapture.
    m_cvCamera.open(m_nCameraIndex);
    // Check if the camera was successfully opened.
    if (m_cvCamera.isOpened())
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "Camera {} at video index {} has been successfully opened.", m_cvCamera.getBackendName(), m_nCameraIndex);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "Unable to open camera at video index {}", m_nCameraIndex);
    }
}

/******************************************************************************
 * @brief Destroy the Basic Cam:: Basic Cam object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
BasicCam::~BasicCam()
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
 * @date 2023-09-16
 ******************************************************************************/
void BasicCam::ThreadedContinuousCode()
{
    // Check if camera is NOT open.
    if (!m_cvCamera.isOpened())
    {
        // Shutdown threads for this BasicCam.
        this->RequestStop();

        // Submit logger message.
        LOG_CRITICAL(logging::g_qSharedLogger,
                     "Camera start was attempted for camera at {}/{}, but camera never properly opened or it has become disconnected!",
                     m_nCameraIndex,
                     m_szCameraPath);
    }
    else
    {
        // Check if new frame was computed successfully.
        if (m_cvCamera.read(m_cvFrame))
        {
            // Call FPS tick.
            m_IPS.Tick();
        }
        else
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "Unable to read new frame for camera {}, {}!", m_nCameraIndex, m_szCameraPath);
        }

        // Acquire a shared_lock on the frame copy queue.
        std::shared_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
        // Check if the frame copy queue is empty.
        if (!m_qFrameCopySchedule.empty())
        {
            // Start the thread pool to store multiple copies of the sl::Mat into the given cv::Mats.
            this->RunDetachedPool(m_qFrameCopySchedule.size(), constants::BASIC_LEFTCAM_FRAME_RETRIEVAL_THREADS);
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
 * @date 2023-09-16
 ******************************************************************************/
void BasicCam::PooledLinearCode()
{
    // Aqcuire mutex for getting frames out of the queue.
    std::unique_lock<std::mutex> lkFrameQueue(m_muFrameCopyMutex);
    // Check if the queue is empty.
    if (!m_qFrameCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::FrameFetchContainer<cv::Mat&>& stContainer = m_qFrameCopySchedule.front();
        // Pop out of queue.
        m_qFrameCopySchedule.pop();
        // Release lock.
        lkFrameQueue.unlock();

        // Acquire unique lock on container.
        std::unique_lock<std::mutex> lkConditionLock(stContainer.muConditionMutex);
        // Copy frame to data container.
        stContainer.tFrame = m_cvFrame.clone();
        // Release lock.
        lkConditionLock.unlock();
        // Use the condition variable to notify other waiting threads that this thread is finished.
        stContainer.cdMatWriteSuccess.notify_all();
    }
}

/******************************************************************************
 * @brief Retrieves a frame from the camera. The given mat reference is placed
 *      into a queue for copying. Remember, this code will be ran in whatever
 *      class/thread calls it.
 *
 * @param cvFrame - A reference to the cv::Mat to store the frame in.
 * @return true - Frame successfully retrieved and stored.
 * @return false - Frame was not stored successfully.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-09
 ******************************************************************************/
bool BasicCam::GrabFrame(cv::Mat& cvFrame)
{
    // Assemble the FrameFetchContainer.
    containers::FrameFetchContainer<cv::Mat&> stContainer(cvFrame, m_ePropPixelFormat);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append frame fetch container to the scedule queue.
    m_qFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Create lock variable to be used by condition variable. CV unlocks this during wait().
    std::unique_lock<std::mutex> lkConditionLock(stContainer.muConditionMutex);
    // Wait up to 10 seconds for the condition variable to be notified.
    std::cv_status cdStatus = stContainer.cdMatWriteSuccess.wait_for(lkConditionLock, std::chrono::seconds(10));
    // Release lock.
    lkConditionLock.unlock();

    // Check condition variable status and return.
    if (cdStatus == std::cv_status::no_timeout)
    {
        // Image was successfully written to the given cv::Mat reference.
        return true;
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Reached timeout while retrieving frame from threadpool queue.");
        // Image was not written successfully.
        return false;
    }
}

/******************************************************************************
 * @brief Accessor for the camera open status.
 *
 * @return true - The camera has been successfully opened.
 * @return false - The camera has not been successfully opened.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-20
 ******************************************************************************/
bool BasicCam::GetCameraIsOpen()
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
 * @date 2023-08-20
 ******************************************************************************/
template<typename T>
T BasicCam::GetCameraLocation() const
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
