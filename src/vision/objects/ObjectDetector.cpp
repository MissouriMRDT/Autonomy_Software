/******************************************************************************
 * @brief Implements the ObjectDetector class.
 *
 * @file ObjectDetector.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-23
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "ObjectDetector.h"
#include "../../util/vision/ImageOperations.hpp"

/******************************************************************************
 * @brief Construct a new ObjectDetector object.
 *
 * @param pBasicCam - A pointer to the BasicCam camera to get frames from for detection.
 * @param nNumDetectedObjectsRetrievalThreads - The number of threads to use when fulfilling
 *                                           requests for the detected depth objects. Default is 5.
 * @param bUsingGpuMats - Whether or not the given camera name will be using GpuMats.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-10
 ******************************************************************************/
ObjectDetector::ObjectDetector(BasicCam* pBasicCam, const int nNumDetectedObjectsRetrievalThreads, const bool bUsingGpuMats)
{
    // Initialize member variables.
    m_pCamera                             = dynamic_cast<BasicCam*>(pBasicCam);
    m_bUsingZedCamera                     = false;    // Toggle ZED functions off.
    m_nNumDetectedObjectsRetrievalThreads = nNumDetectedObjectsRetrievalThreads;
    m_bUsingGpuMats                       = bUsingGpuMats;
    m_IPS                                 = IPS();
}

/******************************************************************************
 * @brief Construct a new ObjectDetector object.
 *
 * @param pZEDCam - A pointer to the ZEDCam camera to get frames from for detection. Override for ZED camera.
 * @param nNumDetectedObjectsRetrievalThreads - The number of threads to use when fulfilling
 *                                           requests for the detected depth objects. Default is 5.
 * @param bUsingGpuMats - Whether or not the given camera name will be using GpuMats.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
ObjectDetector::ObjectDetector(ZEDCam* pZEDCam, const int nNumDetectedObjectsRetrievalThreads, const bool bUsingGpuMats)
{
    // Initialize member variables.
    m_pCamera                             = dynamic_cast<ZEDCam*>(pZEDCam);
    m_bUsingZedCamera                     = true;    // Toggle ZED functions off.
    m_nNumDetectedObjectsRetrievalThreads = nNumDetectedObjectsRetrievalThreads;
    m_bUsingGpuMats                       = bUsingGpuMats;
}

/******************************************************************************
 * @brief This code will run continuously in a separate thread. New frames from
 *      the given camera are grabbed and the objects for the camera image are detected,
 *      filtered, and stored. Then any requests for the current objects are fulfilled.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
void ObjectDetector::ThreadedContinuousCode()
{
    // Create future for indicating when the frame has been copied.
    std::future<bool> fuNormalFrame;
    std::future<bool> fuDepthMeasureCopyStatus;

    // Check if the camera is setup to use CPU or GPU mats.
    if (m_bUsingZedCamera)
    {
        // Check if the ZED camera is returning cv::cuda::GpuMat or cv:Mat.
        if (m_bUsingGpuMats)
        {
            // Grabs normal frame and depth measure from ZEDCam. Dynamic casts Camera to ZEDCam* so we can use ZEDCam methods.
            fuNormalFrame            = dynamic_cast<ZEDCam*>(m_pCamera)->RequestFrameCopy(m_cvGPUNormalFrame);
            fuDepthMeasureCopyStatus = dynamic_cast<ZEDCam*>(m_pCamera)->RequestDepthCopy(m_cvGPUDepthMeasure);

            // Wait for requested frames to be retrieved.
            if (fuDepthMeasureCopyStatus.get() && fuNormalFrame.get())
            {
                // Download mat from GPU memory.
                m_cvGPUNormalFrame.download(m_cvNormalFrame);
                m_cvGPUDepthMeasure.download(m_cvDepthMeasure);
            }
            else
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "ObjectDetector unable to get normal frame or depth measure from ZEDCam!");
            }
        }
        else
        {
            // Grabs normal frame and depth measure from ZEDCam. Dynamic casts Camera to ZEDCam* so we can use ZEDCam methods.
            fuNormalFrame            = dynamic_cast<ZEDCam*>(m_pCamera)->RequestFrameCopy(m_cvNormalFrame);
            fuDepthMeasureCopyStatus = dynamic_cast<ZEDCam*>(m_pCamera)->RequestDepthCopy(m_cvDepthMeasure);

            // Wait for requested frames to be retrieved.
            if (!fuDepthMeasureCopyStatus.get() || !fuNormalFrame.get())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "ObjectDetector unable to get normal frame or depth measure from ZEDCam!");
            }
        }
    }
    else
    {
        // Grab frames from camera.
        fuNormalFrame = dynamic_cast<BasicCam*>(m_pCamera)->RequestFrameCopy(m_cvNormalFrame);

        // Wait for requested frames to be retrieved.
        if (!fuNormalFrame.get())
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "ObjectDetector unable to get requested frames from BasicCam!");
        }
    }

    /////////////////////////////////////////
    // Call detection methods and inference.
    /////////////////////////////////////////

    // TODO: Implement when ready, commented out to suppress warnings.
    // Merge the newly detected objects with the pre-existing detected objects
    // this->UpdateDetectedObjects(vNewlyDetectedObjects);

    // Call FPS tick.
    m_IPS.Tick();
    /////////////////////////////////////////////////////////////////////////////////////

    // Acquire a shared_lock on the detected objects copy queue.
    std::shared_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Check if the detected object copy queue is empty.
    if (!m_qDetectedDepthObjectCopySchedule.empty() || !m_qDetectedTensorflowObjectCopySchedule.empty() || !m_qDetectedObjectDrawnOverlayFrames.empty())
    {
        size_t siQueueLength =
            std::max({m_qDetectedDepthObjectCopySchedule.size(), m_qDetectedTensorflowObjectCopySchedule.size(), m_qDetectedObjectDrawnOverlayFrames.size()});
        // Start the thread pool to store multiple copies of the detected objects to the requesting threads
        this->RunDetachedPool(siQueueLength, m_nNumDetectedObjectsRetrievalThreads);
        // Wait for thread pool to finish.
        this->JoinPool();
        // Release lock on frame copy queue.
        lkSchedulers.unlock();
    }
}

/******************************************************************************
 * @brief This method holds the code that is ran in the thread pool started by
 *      the ThreadedLinearCode() method. It copies the data from the different
 *      data objects to references of the same type stored in a queue filled by the
 *      Request methods.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-08
 ******************************************************************************/
void ObjectDetector::PooledLinearCode()
{
    /////////////////////////////
    //  Detection Overlay Frame queue.
    /////////////////////////////
    // Acquire sole writing access to the detectedObjectCopySchedule.
    std::unique_lock<std::mutex> lkObjectOverlayFrameQueue(m_muFrameCopyMutex);
    // Check if there are unfulfilled requests.
    if (!m_qDetectedObjectDrawnOverlayFrames.empty())
    {
        // Get frame container out of queue.
        containers::FrameFetchContainer<cv::Mat> stContainer = m_qDetectedObjectDrawnOverlayFrames.front();
        // Pop out of queue.
        m_qDetectedObjectDrawnOverlayFrames.pop();
        // Release lock.
        lkObjectOverlayFrameQueue.unlock();

        // Check which frame we should copy.
        switch (stContainer.eFrameType)
        {
            case PIXEL_FORMATS::eDepthDetection: *(stContainer.pFrame) = m_cvProcFrame; break;
            case PIXEL_FORMATS::eTensorflowDetection: *(stContainer.pFrame) = m_cvProcFrame; break;
            default: *(stContainer.pFrame) = m_cvProcFrame;
        }

        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedFrameStatus->set_value(true);
    }

    /////////////////////////////
    //  DepthObject queue.
    /////////////////////////////
    // Acquire sole writing access to the detectedObjectCopySchedule.
    std::unique_lock<std::mutex> lkDepthObjectQueue(m_muDepthDataCopyMutex);
    // Check if there are unfulfilled requests.
    if (!m_qDetectedDepthObjectCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::DataFetchContainer<std::vector<depthobject::DepthObject>> stContainer = m_qDetectedDepthObjectCopySchedule.front();
        // Pop out of queue.
        m_qDetectedDepthObjectCopySchedule.pop();
        // Release lock.
        lkDepthObjectQueue.unlock();

        // Copy the detected objects to the target location
        *(stContainer.pData) = m_vDetectedDepthObjects;

        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedDataStatus->set_value(true);
    }

    /////////////////////////////
    //  TensorflowObject queue.
    /////////////////////////////
    // Acquire sole writing access to the detectedObjectCopySchedule.
    std::unique_lock<std::mutex> lkTensorflowObjectQueue(m_muTensorflowDataCopyMutex);
    // Check if there are unfulfilled requests.
    if (!m_qDetectedTensorflowObjectCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::DataFetchContainer<std::vector<tensorflowobject::TensorflowObject>> stContainer = m_qDetectedTensorflowObjectCopySchedule.front();
        // Pop out of queue.
        m_qDetectedTensorflowObjectCopySchedule.pop();
        // Release lock.
        lkTensorflowObjectQueue.unlock();

        // Copy the detected objects to the target location
        *(stContainer.pData) = m_vDetectedTensorObjects;

        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedDataStatus->set_value(true);
    }
}

/******************************************************************************
 * @brief Request a copy of a frame containing the object detection overlays from the
 *      depth library.
 *
 * @param cvFrame - The frame to copy the detection overlay image to.
 * @return std::future<bool> - The future that should be waited on before using the passed in frame.
 *                      Future will be true or false based on whether or not the frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-11
 ******************************************************************************/
std::future<bool> ObjectDetector::RequestDepthDetectionOverlayFrame(cv::Mat& cvFrame)
{
    // Assemble the DataFetchContainer.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvFrame, PIXEL_FORMATS::eDepthDetection);

    // Acquire lock on pool copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qDetectedObjectDrawnOverlayFrames.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Request a copy of a frame containing the object detection overlays from the
 *      tensorflow model.
 *
 * @param cvFrame - The frame to copy the detection overlay image to.
 * @return std::future<bool> - The future that should be waited on before using the passed in frame.
 *                      Future will be true or false based on whether or not the frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-11
 ******************************************************************************/
std::future<bool> ObjectDetector::RequestTensorflowDetectionOverlayFrame(cv::Mat& cvFrame)
{
    // Assemble the DataFetchContainer.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvFrame, PIXEL_FORMATS::eTensorflowDetection);

    // Acquire lock on pool copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qDetectedObjectDrawnOverlayFrames.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Request the most up to date vector of detected objects from OpenCV's Depth
 *      algorithm.
 *
 * @param vDepthObjects - The vector the detected depth objects will be saved to.
 * @return std::future<bool> - The future that should be waited on before using the passed in object vector.
 *                      Future will be true or false based on whether or not the objects were successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
std::future<bool> ObjectDetector::RequestDetectedDepthObjects(std::vector<depthobject::DepthObject>& vDepthObjects)
{
    // Assemble the DataFetchContainer.
    containers::DataFetchContainer<std::vector<depthobject::DepthObject>> stContainer(vDepthObjects);

    // Acquire lock on pool copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append detected object fetch container to the schedule queue.
    m_qDetectedDepthObjectCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedDataStatus->get_future();
}

/******************************************************************************
 * @brief Request the most up to date vector of detected objects from our custom tensorflow
 *      model.
 *
 * @param vTensorflowObjects - The vector the detected tensorflow objects will be saved to.
 * @return std::future<bool> - The future that should be waited on before using the passed in object vector.
 *                      Future will be true or false based on whether or not the objects were successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
std::future<bool> ObjectDetector::RequestDetectedTensorflowObjects(std::vector<tensorflowobject::TensorflowObject>& vTensorflowObjects)
{
    // Assemble the DataFetchContainer.
    containers::DataFetchContainer<std::vector<tensorflowobject::TensorflowObject>> stContainer(vTensorflowObjects);

    // Acquire lock on pool copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append detected object fetch container to the schedule queue.
    m_qDetectedTensorflowObjectCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedDataStatus->get_future();
}

// TODO: Implement when ready, commented out to suppress warnings.
// /******************************************************************************
//  * @brief Updates the detected depth objects including forgetting objects that haven't been seen for long enough.
//  *      If a new object is spotted: add it to the detected objects vector
//  *      If a object has been spotted again: update the objects distance and angle
//  *      If a object hasn't been seen for a while: remove it from the vector
//  *
//  * @param vNewlyDetectedObjects - Input vector of DepthObject structs containing the object info.
//  *
//  * @author jspencerpittman (jspencerpittman@gmail.com)
//  * @date 2023-10-06
//  ******************************************************************************/
// void ObjectDetector::UpdateDetectedObjects(std::vector<depthobject::DepthObject>& vNewlyDetectedObjects)
// {
//     // Put tag filter info here.
//     // Add new tags to member variable.
// }

// TODO: Implement when ready, commented out to suppress warnings.
// /******************************************************************************
//  * @brief Updates the detected tensorflow objects including forgetting objects that haven't been seen for long enough.
//  *      If a new object is spotted: add it to the detected objects vector
//  *      If a object has been spotted again: update the objects distance and angle
//  *      If a object hasn't been seen for a while: remove it from the vector
//  *
//  * @param vNewlyDetectedObjects - Input vector of TensorflowObject structs containing the object info.
//  *
//  * @author clayjay3 (claytonraycowen@gmail.com)
//  * @date 2023-10-07
//  ******************************************************************************/
// void ObjectDetector::UpdateDetectedObjects(std::vector<tensorflowobject::TensorflowObject>& vNewlyDetectedObjects)
// {
//     // Put tag filter info here.
//     // Add new tags to member variable.
// }

/******************************************************************************
 * @brief Accessor for the Frame I P S private member.
 *
 * @return IPS& - The detector objects iteration per second counter.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-10
 ******************************************************************************/
IPS& ObjectDetector::GetIPS()
{
    // Return Iterations Per Second counter.
    return m_IPS;
}
