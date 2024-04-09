/******************************************************************************
 * @brief Implements the TagDetector class.
 *
 * @file TagDetector.cpp
 * @author clayjay3 (claytonraycowen@gmail.com), jspencerpittman (jspencerpittman@gmail.com)
 * @date 2023-10-07
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "TagDetector.h"
#include "../../util/vision/ImageOperations.hpp"

/******************************************************************************
 * @brief Construct a new TagDetector object.
 *
 * @param pBasicCam - A pointer to the BasicCam camera to get frames from for detection.
 * @param nArucoCornerRefinementMaxIterations - The number of iterations to use when refining marker corners.
 * @param nArucoCornerRefinementMethod - The refinement method to use.
 * @param nArucoMarkerBorderBits - The number of border unit squares around the marker.
 * @param bArucoDetectInvertedMarkers - Enable or disable upside-down marker detection.
 * @param bUseAruco3Detection - Whether or not to use the newer/faster method of detection. Experimental.
 * @param nDetectorMaxFPS - The max FPS limit the detector can run at.
 * @param bEnableRecordingFlag - Whether or not this TagDetector's overlay output should be recorded.
 * @param nNumDetectedTagsRetrievalThreads - The number of threads to use when fulfilling
 *                                           requests for the detected aruco tags. Default is 5.
 * @param bUsingGpuMats - Whether or not the given camera name will be using GpuMats.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-10
 ******************************************************************************/
TagDetector::TagDetector(BasicCam* pBasicCam,
                         const int nArucoCornerRefinementMaxIterations,
                         const int nArucoCornerRefinementMethod,
                         const int nArucoMarkerBorderBits,
                         const bool bArucoDetectInvertedMarkers,
                         const bool bUseAruco3Detection,
                         const int nDetectorMaxFPS,
                         const bool bEnableRecordingFlag,
                         const int nNumDetectedTagsRetrievalThreads,
                         const bool bUsingGpuMats)
{
    // Initialize member variables.
    m_pCamera                          = pBasicCam;
    m_bTensorflowInitialized           = false;
    m_bTensorflowEnabled               = false;
    m_bUsingZedCamera                  = false;    // Toggle ZED functions off.
    m_bUsingGpuMats                    = bUsingGpuMats;
    m_bCameraIsOpened                  = false;
    m_nNumDetectedTagsRetrievalThreads = nNumDetectedTagsRetrievalThreads;
    m_szCameraName                     = dynamic_cast<BasicCam*>(pBasicCam)->GetCameraLocation();
    m_bEnableRecordingFlag             = bEnableRecordingFlag;
    m_IPS                              = IPS();

    // Setup aruco detector params.
    m_cvArucoDetectionParams                               = cv::aruco::DetectorParameters();
    m_cvArucoDetectionParams.cornerRefinementMaxIterations = nArucoCornerRefinementMaxIterations;
    m_cvArucoDetectionParams.cornerRefinementMethod        = nArucoCornerRefinementMethod;
    m_cvArucoDetectionParams.markerBorderBits              = nArucoMarkerBorderBits;
    m_cvArucoDetectionParams.detectInvertedMarker          = bArucoDetectInvertedMarkers;
    m_cvArucoDetectionParams.useAruco3Detection            = bUseAruco3Detection;
    // Get aruco dictionary and initialize aruco detector.
    m_cvTagDictionary = cv::aruco::getPredefinedDictionary(constants::ARUCO_DICTIONARY);
    m_cvArucoDetector = cv::aruco::ArucoDetector(m_cvTagDictionary, m_cvArucoDetectionParams);

    // Set max IPS of main thread.
    this->SetMainThreadIPSLimit(nDetectorMaxFPS);

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "TagDetector created for camera at path/index: {}", m_szCameraName);
}

/******************************************************************************
 * @brief Construct a new TagDetector object.
 *
 * @param pZEDCam - A pointer to the ZEDCam camera to get frames from for detection. Override for ZED camera.
 * @param nArucoCornerRefinementMaxIterations - The number of iterations to use when refining marker corners.
 * @param nArucoCornerRefinementMethod - The refinement method to use.
 * @param nArucoMarkerBorderBits - The number of border unit squares around the marker.
 * @param bArucoDetectInvertedMarkers - Enable or disable upside-down marker detection.
 * @param bUseAruco3Detection - Whether or not to use the newer/faster method of detection. Experimental.
 * @param nDetectorMaxFPS - The max FPS limit the detector can run at.
 * @param bEnableRecordingFlag - Whether or not this TagDetector's overlay output should be recorded.
 * @param nNumDetectedTagsRetrievalThreads - The number of threads to use when fulfilling
 *                                           requests for the detected aruco tags. Default is 5.
 * @param bUsingGpuMats - Whether or not the given camera name will be using GpuMats.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
TagDetector::TagDetector(ZEDCam* pZEDCam,
                         const int nArucoCornerRefinementMaxIterations,
                         const int nArucoCornerRefinementMethod,
                         const int nArucoMarkerBorderBits,
                         const bool bArucoDetectInvertedMarkers,
                         const bool bUseAruco3Detection,
                         const int nDetectorMaxFPS,
                         const bool bEnableRecordingFlag,
                         const int nNumDetectedTagsRetrievalThreads,
                         const bool bUsingGpuMats)
{
    // Initialize member variables.
    m_pCamera                          = pZEDCam;
    m_bTensorflowInitialized           = false;
    m_bTensorflowEnabled               = false;
    m_bUsingZedCamera                  = true;    // Toggle ZED functions on.
    m_bUsingGpuMats                    = bUsingGpuMats;
    m_bCameraIsOpened                  = false;
    m_nNumDetectedTagsRetrievalThreads = nNumDetectedTagsRetrievalThreads;
    m_szCameraName                     = dynamic_cast<ZEDCam*>(pZEDCam)->GetCameraModel() + "_" + std::to_string(dynamic_cast<ZEDCam*>(pZEDCam)->GetCameraSerial());
    m_bEnableRecordingFlag             = bEnableRecordingFlag;
    m_IPS                              = IPS();

    // Setup aruco detector params.
    m_cvArucoDetectionParams                               = cv::aruco::DetectorParameters();
    m_cvArucoDetectionParams.cornerRefinementMaxIterations = nArucoCornerRefinementMaxIterations;
    m_cvArucoDetectionParams.cornerRefinementMethod        = nArucoCornerRefinementMethod;
    m_cvArucoDetectionParams.markerBorderBits              = nArucoMarkerBorderBits;
    m_cvArucoDetectionParams.detectInvertedMarker          = bArucoDetectInvertedMarkers;
    m_cvArucoDetectionParams.useAruco3Detection            = bUseAruco3Detection;
    // Get aruco dictionary and initialize aruco detector.
    m_cvTagDictionary = cv::aruco::getPredefinedDictionary(constants::ARUCO_DICTIONARY);
    m_cvArucoDetector = cv::aruco::ArucoDetector(m_cvTagDictionary, m_cvArucoDetectionParams);

    // Set max IPS of main thread.
    this->SetMainThreadIPSLimit(nDetectorMaxFPS);

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "TagDetector created for camera: {}", m_szCameraName);
}

/******************************************************************************
 * @brief Destroy the Tag Detector:: Tag Detector object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-09
 ******************************************************************************/
TagDetector::~TagDetector()
{
    // Stop threaded code.
    this->RequestStop();
    this->Join();

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "TagDetector for camera {} had been successfully destroyed.", this->GetCameraName());
}

/******************************************************************************
 * @brief This code will run continuously in a separate thread. New frames from
 *      the given camera are grabbed and the tags for the camera image are detected,
 *      filtered, and stored. Then any requests for the current tags are fulfilled.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
void TagDetector::ThreadedContinuousCode()
{
    // Check if using ZEDCam or BasicCam.
    if (m_bUsingZedCamera)
    {
        // Check if camera is NOT open.
        if (!dynamic_cast<ZEDCam*>(m_pCamera)->GetCameraIsOpen())
        {
            // Set camera opened toggle.
            m_bCameraIsOpened = false;

            // If camera's not open on first iteration of thread, it's probably not present, so stop.
            if (this->GetThreadState() == eStarting)
            {
                // Shutdown threads for this ZEDCam.
                this->RequestStop();

                // Submit logger message.
                LOG_CRITICAL(logging::g_qSharedLogger,
                             "TagDetector start was attempted for ZED camera with serial number {}, but camera never properly opened or it has been closed/rebooted!",
                             dynamic_cast<ZEDCam*>(m_pCamera)->GetCameraSerial());
            }
        }
        else
        {
            // Set camera opened toggle.
            m_bCameraIsOpened = true;
        }
    }
    else
    {
        // Check if camera is NOT open.
        if (!dynamic_cast<BasicCam*>(m_pCamera)->GetCameraIsOpen())
        {
            // Set camera opened toggle.
            m_bCameraIsOpened = false;

            // If camera's not open on first iteration of thread, it's probably not present, so stop.
            if (this->GetThreadState() == eStarting)
            {
                // Shutdown threads for this BasicCam.
                this->RequestStop();

                // Submit logger message.
                LOG_CRITICAL(logging::g_qSharedLogger,
                             "TagDetector start was attempted for BasicCam at {}, but camera never properly opened or it has become disconnected!",
                             dynamic_cast<BasicCam*>(m_pCamera)->GetCameraLocation());
            }
        }
        else
        {
            // Set camera opened toggle.
            m_bCameraIsOpened = true;
        }
    }

    // Check if camera is opened.
    if (m_bCameraIsOpened)
    {
        // Create future for indicating when the frame has been copied.
        std::future<bool> fuPointCloudCopyStatus;

        // Check if the camera is setup to use CPU or GPU mats.
        if (m_bUsingZedCamera)
        {
            // Check if the ZED camera is returning cv::cuda::GpuMat or cv:Mat.
            if (m_bUsingGpuMats)
            {
                // Grabs point cloud from ZEDCam. Dynamic casts Camera to ZEDCam* so we can use ZEDCam methods.
                fuPointCloudCopyStatus = dynamic_cast<ZEDCam*>(m_pCamera)->RequestPointCloudCopy(m_cvGPUPointCloud);

                // Wait for point cloud to be retrieved.
                if (fuPointCloudCopyStatus.get())
                {
                    // Download mat from GPU memory.
                    m_cvGPUPointCloud.download(m_cvPointCloud);
                    // Split and store colors from point cloud.
                    imgops::SplitPointCloudColors(m_cvPointCloud, m_cvFrame);
                }
                else
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "TagDetector unable to get point cloud from ZEDCam!");
                }
            }
            else
            {
                // Grabs point cloud from ZEDCam.
                fuPointCloudCopyStatus = dynamic_cast<ZEDCam*>(m_pCamera)->RequestPointCloudCopy(m_cvPointCloud);

                // Wait for point cloud to be retrieved.
                if (fuPointCloudCopyStatus.get())
                {
                    // Split and store colors from point cloud.
                    imgops::SplitPointCloudColors(m_cvPointCloud, m_cvFrame);
                }
                else
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "TagDetector unable to get point cloud from ZEDCam!");
                }
            }
        }
        else
        {
            // Grab frames from camera.
            fuPointCloudCopyStatus = dynamic_cast<BasicCam*>(m_pCamera)->RequestFrameCopy(m_cvFrame);

            // Wait for point cloud to be retrieved.
            if (!fuPointCloudCopyStatus.get())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "TagDetector unable to get point cloud from BasicCam!");
            }
        }

        /////////////////////////////////////////
        // Actual detection logic goes here.
        /////////////////////////////////////////
        // Drop the Alpha channel from the image copy to preproc frame.
        cv::cvtColor(m_cvFrame, m_cvArucoProcFrame, cv::COLOR_BGRA2BGR);
        // Run image through some pre-processing step to improve detection.
        arucotag::PreprocessFrame(m_cvArucoProcFrame, m_cvArucoProcFrame);
        // Detect tags in the image
        std::vector<arucotag::ArucoTag> vNewlyDetectedTags = arucotag::Detect(m_cvArucoProcFrame, m_cvArucoDetector);

        /* FIXME: Issue #237
        cv::Matx<float, 4, 1>::Matx(cv::Matx<float, 4, 1> * const this, const float * values) (\usr\local\include\opencv4\opencv2\core\matx.hpp:686)
        cv::Vec<float, 4>::Vec(cv::Vec<float, 4> * const this, const cv::Vec<float, 4> & m) (\usr\local\include\opencv4\opencv2\core\matx.hpp:1050)
        arucotag::EstimatePoseFromPointCloud(const cv::Mat & cvPointCloud, arucotag::ArucoTag & stTag)
        (\workspaces\Autonomy_Software\src\vision\aruco\ArucoDetection.hpp:244) TagDetector::ThreadedContinuousCode(TagDetector * const this)
        (\workspaces\Autonomy_Software\src\vision\aruco\TagDetector.cpp:287) AutonomyThread<void>::RunThread(AutonomyThread<void> * const this, std::atomic_bool &
        bStopThread) (\workspaces\Autonomy_Software\src\interfaces\AutonomyThread.hpp:617) AutonomyThread<void>::Start()::{lambda()#1}::operator()()
        const(AutonomyThread<void> * const this) (\workspaces\Autonomy_Software\src\interfaces\AutonomyThread.hpp:139)
        BS::thread_pool::submit_task<AutonomyThread<void>::Start()::{lambda()#1}, void>(AutonomyThread<void>::Start()::{lambda()#1}&&, short)::{lambda()#1}::operator()()
        const(const struct {...} * const this) (\workspaces\Autonomy_Software\external\threadpool\include\BS_thread_pool.hpp:617) std::__invoke_impl<void,
        BS::thread_pool::submit_task<AutonomyThread<void>::Start()::{lambda()#1}, void>(AutonomyThread<void>::Start()::{lambda()#1}&&,
        short)::{lambda()#1}&>(std::__invoke_other, BS::thread_pool::submit_task<AutonomyThread<void>::Start()::{lambda()#1},
        void>(AutonomyThread<void>::Start()::{lambda()#1}&&, short)::{lambda()#1}&)(struct {...} & __f)
        (\usr\include\c++\10\bits\invoke.h:60) std::__invoke_r<void, BS::thread_pool::submit_task<AutonomyThread<void>::Start()::{lambda()#1},
        void>(AutonomyThread<void>::Start()::{lambda()#1}&&, short)::{lambda()#1}&>(BS::thread_pool::submit_task<AutonomyThread<void>::Start()::{lambda()#1},
        void>(AutonomyThread<void>::Start()::{lambda()#1}&&, short)::{lambda()#1}&)(struct {...} & __fn) (\usr\include\c++\10\bits\invoke.h:110)
        std::_Function_handler<void (), BS::thread_pool::submit_task<AutonomyThread<void>::Start()::{lambda()#1}, void>(AutonomyThread<void>::Start()::{lambda()#1}&&,
        short)::{lambda()#1}>::_M_invoke(std::_Any_data const&)(const std::_Any_data & __functor) (\usr\include\c++\10\bits\std_function.h:291) std::function<void
        ()>::operator()() const(const std::function<void()> * const this) (\usr\include\c++\10\bits\std_function.h:622) BS::thread_pool::worker(unsigned int,
        std::function<void
        ()> const&)(BS::thread_pool * const this, const BS::concurrency_t idx, const std::function<void()> & init_task)
        (\workspaces\Autonomy_Software\external\threadpool\include\BS_thread_pool.hpp:937) std::__invoke_impl<void, void (BS::thread_pool::*)(unsigned int,
        std::function<void ()> const&), BS::thread_pool*, unsigned int, std::function<void ()> >(std::__invoke_memfun_deref, void (BS::thread_pool::*&&)(unsigned int,
        std::function<void ()> const&), BS::thread_pool*&&, unsigned int&&, std::function<void ()>&&)(void (BS::thread_pool::*&&)(BS::thread_pool * const, unsigned int,
        const std::function<void()> &) __f, BS::thread_pool *&& __t) (\usr\include\c++\10\bits\invoke.h:73) std::__invoke<void (BS::thread_pool::*)(unsigned int,
        std::function<void ()> const&), BS::thread_pool*, unsigned int, std::function<void ()> >(void (BS::thread_pool::*&&)(unsigned int, std::function<void ()> const&),
        BS::thread_pool*&&, unsigned int&&, std::function<void
        ()>&&)(void (BS::thread_pool::*&&)(BS::thread_pool * const, unsigned int, const std::function<void()> &) __fn) (\usr\include\c++\10\bits\invoke.h:95)
        std::thread::_Invoker<std::tuple<void (BS::thread_pool::*)(unsigned int, std::function<void ()> const&), BS::thread_pool*, unsigned int, std::function<void ()> >
        >::_M_invoke<0ul, 1ul, 2ul, 3ul>(std::_Index_tuple<0ul, 1ul, 2ul, 3ul>)(std::thread::_Invoker<std::tuple<void (BS::thread_pool::*)(unsigned int, const
        std::function<void()>&), BS::thread_pool*, unsigned int, std::function<void()> > > * const this) (\usr\include\c++\10\thread:264)
        std::thread::_Invoker<std::tuple<void (BS::thread_pool::*)(unsigned int, std::function<void ()> const&), BS::thread_pool*, unsigned int, std::function<void ()> >
        >::operator()()(std::thread::_Invoker<std::tuple<void (BS::thread_pool::*)(unsigned int, const std::function<void()>&), BS::thread_pool*, unsigned int,
        std::function<void()> > > * const this) (\usr\include\c++\10\thread:271) std::thread::_State_impl<std::thread::_Invoker<std::tuple<void
        (BS::thread_pool::*)(unsigned int, std::function<void ()> const&), BS::thread_pool*, unsigned int, std::function<void ()> > >
        >::_M_run()(std::thread::_State_impl<std::thread::_Invoker<std::tuple<void (BS::thread_pool::*)(unsigned int, const std::function<void()>&), BS::thread_pool*,
        unsigned int, std::function<void()> > > > * const this)
        (\usr\include\c++\10\thread:215) libstdc++.so.6![Unknown/Just-In-Time compiled code] (Unknown Source:0) libc.so.6!start_thread(void * arg) (pthread_create.c:442)
        libc.so.6!clone3() (clone3.S:81)
        */

        // Estimate the positions of the tags using the point cloud
        // for (arucotag::ArucoTag& stTag : vNewlyDetectedTags)
        // {
        //     // Use the point cloud to get the location of the tag.
        //     arucotag::EstimatePoseFromPointCloud(m_cvPointCloud, stTag);
        // }
        // Merge the newly detected tags with the pre-existing detected tags
        this->UpdateDetectedTags(vNewlyDetectedTags);
        // Draw tag overlays onto normal image.
        arucotag::DrawDetections(m_cvArucoProcFrame, m_vDetectedArucoTags);

        // Check if tensorflow detection if turned on.
        if (m_bTensorflowEnabled)
        {
            // Drop the Alpha channel from the image copy to preproc frame.
            cv::cvtColor(m_cvFrame, m_cvTensorflowProcFrame, cv::COLOR_BGRA2RGB);
            // Detect tags in the image.
            m_vDetectedTensorTags = tensorflowtag::Detect(m_cvTensorflowProcFrame, *m_pTensorflowDetector, m_fMinObjectConfidence, m_fNMSThreshold);
            // Estimate the positions of the tags using the point cloud
            for (tensorflowtag::TensorflowTag& stTag : m_vDetectedTensorTags)
            {
                // Use the point cloud to get the location of the tag.
                tensorflowtag::EstimatePoseFromPointCloud(m_cvPointCloud, stTag);
            }
            // Draw tag overlays onto normal image.
            tensorflowtag::DrawDetections(m_cvArucoProcFrame, m_vDetectedTensorTags);
        }

        /////////////////////////////////////////////////////////////////////////////////////
    }

    // Acquire a shared_lock on the detected tags copy queue.
    std::shared_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Check if the detected tag copy queue is empty.
    if (!m_qDetectedArucoTagCopySchedule.empty() || !m_qDetectedTensorflowTagCopySchedule.empty() || !m_qDetectedTagDrawnOverlayFrames.empty())
    {
        size_t siQueueLength = std::max({m_qDetectedArucoTagCopySchedule.size(), m_qDetectedTensorflowTagCopySchedule.size(), m_qDetectedTagDrawnOverlayFrames.size()});
        // Start the thread pool to store multiple copies of the detected tags to the requesting threads
        this->RunDetachedPool(siQueueLength, m_nNumDetectedTagsRetrievalThreads);
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
void TagDetector::PooledLinearCode()
{
    /////////////////////////////
    //  Detection Overlay Frame queue.
    /////////////////////////////
    // Acquire sole writing access to the detectedTagCopySchedule.
    std::unique_lock<std::mutex> lkTagOverlayFrameQueue(m_muFrameCopyMutex);
    // Check if there are unfulfilled requests.
    if (!m_qDetectedTagDrawnOverlayFrames.empty())
    {
        // Get frame container out of queue.
        containers::FrameFetchContainer<cv::Mat> stContainer = m_qDetectedTagDrawnOverlayFrames.front();
        // Pop out of queue.
        m_qDetectedTagDrawnOverlayFrames.pop();
        // Release lock.
        lkTagOverlayFrameQueue.unlock();

        // Check which frame we should copy.
        switch (stContainer.eFrameType)
        {
            case eArucoDetection: *(stContainer.pFrame) = m_cvArucoProcFrame; break;
            default: *(stContainer.pFrame) = m_cvArucoProcFrame;
        }

        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedFrameStatus->set_value(true);
    }

    /////////////////////////////
    //  ArucoTag queue.
    /////////////////////////////
    // Acquire sole writing access to the detectedTagCopySchedule.
    std::unique_lock<std::mutex> lkArucoTagQueue(m_muArucoDataCopyMutex);
    // Check if there are unfulfilled requests.
    if (!m_qDetectedArucoTagCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::DataFetchContainer<std::vector<arucotag::ArucoTag>> stContainer = m_qDetectedArucoTagCopySchedule.front();
        // Pop out of queue.
        m_qDetectedArucoTagCopySchedule.pop();
        // Release lock.
        lkArucoTagQueue.unlock();

        // Copy the detected tags to the target location
        *(stContainer.pData) = m_vDetectedArucoTags;

        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedDataStatus->set_value(true);
    }

    /////////////////////////////
    //  TensorflowTag queue.
    /////////////////////////////
    // Acquire sole writing access to the detectedTagCopySchedule.
    std::unique_lock<std::mutex> lkTensorflowTagQueue(m_muTensorflowDataCopyMutex);
    // Check if there are unfulfilled requests.
    if (!m_qDetectedTensorflowTagCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::DataFetchContainer<std::vector<tensorflowtag::TensorflowTag>> stContainer = m_qDetectedTensorflowTagCopySchedule.front();
        // Pop out of queue.
        m_qDetectedTensorflowTagCopySchedule.pop();
        // Release lock.
        lkTensorflowTagQueue.unlock();

        // Copy the detected tags to the target location
        *(stContainer.pData) = m_vDetectedTensorTags;

        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedDataStatus->set_value(true);
    }
}

/******************************************************************************
 * @brief Request a copy of a frame containing the tag detection overlays from the
 *      aruco and tensorflow library.
 *
 * @param cvFrame - The frame to copy the detection overlay image to.
 * @return std::future<bool> - The future that should be waited on before using the passed in frame.
 *                      Future will be true or false based on whether or not the frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-11
 ******************************************************************************/
std::future<bool> TagDetector::RequestDetectionOverlayFrame(cv::Mat& cvFrame)
{
    // Assemble the DataFetchContainer.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvFrame, eArucoDetection);

    // Acquire lock on pool copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qDetectedTagDrawnOverlayFrames.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Request the most up to date vector of detected tags from OpenCV's Aruco
 *      algorithm.
 *
 * @param vArucoTags - The vector the detected aruco tags will be saved to.
 * @return std::future<bool> - The future that should be waited on before using the passed in tag vector.
 *                      Future will be true or false based on whether or not the tags were successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
std::future<bool> TagDetector::RequestDetectedArucoTags(std::vector<arucotag::ArucoTag>& vArucoTags)
{
    // Assemble the DataFetchContainer.
    containers::DataFetchContainer<std::vector<arucotag::ArucoTag>> stContainer(vArucoTags);

    // Acquire lock on pool copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append detected tag fetch container to the schedule queue.
    m_qDetectedArucoTagCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedDataStatus->get_future();
}

/******************************************************************************
 * @brief Request the most up to date vector of detected tags from our custom tensorflow
 *      model.
 *
 * @param vTensorflowTags - The vector the detected tensorflow tags will be saved to.
 * @return std::future<bool> - The future that should be waited on before using the passed in tag vector.
 *                      Future will be true or false based on whether or not the tags were successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
std::future<bool> TagDetector::RequestDetectedTensorflowTags(std::vector<tensorflowtag::TensorflowTag>& vTensorflowTags)
{
    // Assemble the DataFetchContainer.
    containers::DataFetchContainer<std::vector<tensorflowtag::TensorflowTag>> stContainer(vTensorflowTags);

    // Acquire lock on pool copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append detected tag fetch container to the schedule queue.
    m_qDetectedTensorflowTagCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedDataStatus->get_future();
}

/******************************************************************************
 * @brief Attempt to open the next available TPU hardware and load model at the given
 *      path onto the device.
 *
 * @param szModelPath - The absolute path to the model to open.
 * @param ePerformanceMode - The performance mode to launch the TPU device in.
 * @return true - Model was opened and loaded successfully onto the TPU device.
 * @return false - Something went wrong, model/device not opened.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-31
 ******************************************************************************/
bool TagDetector::InitTensorflowDetection(const std::string szModelPath, yolomodel::tensorflow::TPUInterpreter::PerformanceModes ePerformanceMode)
{
    // Initialize a new YOLOModel object.
    m_pTensorflowDetector = std::make_shared<yolomodel::tensorflow::TPUInterpreter>(szModelPath, ePerformanceMode);
    // Open and load a new YOLOModel from the given path into an EdgeTPU device.
    TfLiteStatus tfReturnStatus = m_pTensorflowDetector->OpenAndLoad();

    // Check if device/model was opened without issue.
    if (tfReturnStatus == TfLiteStatus::kTfLiteOk)
    {
        // Update member variable.
        m_bTensorflowInitialized = true;
        // Return status.
        return true;
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "Unable to initialize Tensorflow detection for TagDetector.");
        // Update member variable.
        m_bTensorflowInitialized = false;
        // Close hardware.
        m_pTensorflowDetector->CloseHardware();
        // Return status.
        return false;
    }
}

/******************************************************************************
 * @brief Turn on tensorflow detection with given parameters.
 *
 * @param fMinObjectConfidence - The lower limit of detection confidence.
 * @param fNMSThreshold - The overlap thresh for NMS algorithm.
 *
 * @note Tensorflow model must be initialized first with the InitTensorflowDetection() method.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-31
 ******************************************************************************/
void TagDetector::EnableTensorflowDetection(const float fMinObjectConfidence, const float fNMSThreshold)
{
    // Update member variables.
    m_fMinObjectConfidence = fMinObjectConfidence;
    m_fNMSThreshold        = fNMSThreshold;

    // Check if tensorflow model has been initialized.
    if (m_bTensorflowInitialized)
    {
        // Update member variable.
        m_bTensorflowEnabled = true;
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Tried to enable tensorflow detection for TagDetector but it has not been initialized yet!");
        // Update member variable.
        m_bTensorflowEnabled = false;
    }
}

/******************************************************************************
 * @brief Set flag to stop tag detection with the tensorflow model.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-31
 ******************************************************************************/
void TagDetector::DisableTensorflowDetection()
{
    // Update member variables.
    m_bTensorflowEnabled = false;
}

/******************************************************************************
 * @brief Updates the detected aruco tags including forgetting tags that haven't been seen for long enough.
 *      If a new tag is spotted: add it to the detected tags vector
 *      If a tag has been spotted again: update the tags distance and angle
 *      If a tag hasn't been seen for a while: remove it from the vector
 *
 * @param vNewlyDetectedTags - Input vector of ArucoTag structs containing the tag info.
 *
 * @author jspencerpittman (jspencerpittman@gmail.com)
 * @date 2023-10-06
 ******************************************************************************/
void TagDetector::UpdateDetectedTags(std::vector<arucotag::ArucoTag>& vNewlyDetectedTags)
{
    // Sort tags from least to greatest.
    std::sort(vNewlyDetectedTags.begin(),
              vNewlyDetectedTags.end(),
              [](const arucotag::ArucoTag& stTag1, const arucotag::ArucoTag& stTag2) { return stTag1.nID < stTag2.nID; });

    // Get the beginning of the new tags and the current tags vector.
    std::vector<arucotag::ArucoTag>::iterator itNewItr = vNewlyDetectedTags.begin();
    std::vector<arucotag::ArucoTag>::iterator itOldItr = m_vDetectedArucoTags.begin();

    // Create vector for storing new tags.
    std::vector<arucotag::ArucoTag> vNewTags;

    // Here we process tags from both the newly detected and previously detected tags in the order of increasing id.
    while (itNewItr != vNewlyDetectedTags.end() || itOldItr != m_vDetectedArucoTags.end())
    {
        // If the id's match then update the previously detected tag.
        if (itNewItr != vNewlyDetectedTags.end() && itOldItr != m_vDetectedArucoTags.end() && itOldItr->nID == itNewItr->nID)
        {
            // Update data for tag.
            itOldItr->dYawAngle             = itNewItr->dYawAngle;
            itOldItr->dStraightLineDistance = itNewItr->dStraightLineDistance;
            itOldItr->CornerTL              = itNewItr->CornerTL;
            itOldItr->CornerTR              = itNewItr->CornerTR;
            itOldItr->CornerBR              = itNewItr->CornerBR;
            itOldItr->CornerBL              = itNewItr->CornerBL;
            itOldItr->vCorners              = {&itOldItr->CornerTL, &itOldItr->CornerTR, &itOldItr->CornerBL, &itOldItr->CornerBR};
            itOldItr->nFramesSinceLastHit   = 0;
            itOldItr->nHits                 = std::max(itOldItr->nHits + 1, constants::ARUCO_VALIDATION_THRESHOLD);

            // Move to next tags.
            itOldItr++;
            itNewItr++;
        }
        // If a previously detected tag wasn't detected in the frame
        else if (itOldItr != m_vDetectedArucoTags.end() && (itNewItr == vNewlyDetectedTags.end() || itOldItr->nID < itNewItr->nID))
        {
            // Increment hit counter.
            itOldItr->nFramesSinceLastHit++;

            // Check if the tag should be removed.
            if ((itOldItr->nHits >= constants::ARUCO_VALIDATION_THRESHOLD && itOldItr->nFramesSinceLastHit >= constants::ARUCO_VALIDATED_TAG_FORGET_THRESHOLD) ||
                !(itOldItr->nHits >= constants::ARUCO_VALIDATION_THRESHOLD && itOldItr->nFramesSinceLastHit >= constants::ARUCO_UNVALIDATED_TAG_FORGET_THRESHOLD))
            {
                // Remove the tag from the detected tags member variable.
                itOldItr = m_vDetectedArucoTags.erase(itOldItr);
            }
            else
            {
                // Decrement the old iterator.
                itOldItr++;
            }
        }
        // A tag was detected for the first time.
        else if (itNewItr != vNewlyDetectedTags.end())
        {
            // Set the new tags attributes for a first detection.
            itNewItr->nHits               = 1;
            itNewItr->nFramesSinceLastHit = 0;

            // Add tag to new tags vector.
            vNewTags.push_back(*itNewItr);

            // Increment the new iterator.
            itNewItr++;
        }
    }

    // Loop through the new tag vector.
    for (arucotag::ArucoTag& stTag : vNewTags)
    {
        // Add the newly detected tags to the member variable list
        m_vDetectedArucoTags.push_back(stTag);
    }
}

/******************************************************************************
 * @brief Mutator for the desired max FPS for this detector.
 *
 * @param nRecordingFPS - The max frames per second to detect tags at.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-22
 ******************************************************************************/
void TagDetector::SetDetectorFPS(const int nRecordingFPS)
{
    // Set the max iterations per second of the recording handler.
    this->SetMainThreadIPSLimit(nRecordingFPS);
}

/******************************************************************************
 * @brief Mutator for the Enable Recording Flag private member
 *
 * @param bEnableRecordingFlag - Whether or not recording should be enabled for this detector.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-31
 ******************************************************************************/
void TagDetector::SetEnableRecordingFlag(const bool bEnableRecordingFlag)
{
    m_bEnableRecordingFlag = bEnableRecordingFlag;
}

/******************************************************************************
 * @brief Accessor for the status of this TagDetector.
 *
 * @return true - The detector is running and detecting tags from the camera.
 * @return false - The detector thread and/or camera is not running/opened.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-04
 ******************************************************************************/
bool TagDetector::GetIsReady()
{
    // Create instance variables.
    bool bDetectorIsReady = false;

    // Check if this detectors thread is currently running.
    if (this->GetThreadState() == eRunning)
    {
        // Check if using ZEDCam or BasicCam.
        if (m_bUsingZedCamera)
        {
            // Check if camera is NOT open.
            if (dynamic_cast<ZEDCam*>(m_pCamera)->GetCameraIsOpen())
            {
                // Set camera opened toggle.
                bDetectorIsReady = true;
            }
        }
        else
        {
            // Check if camera is NOT open.
            if (dynamic_cast<BasicCam*>(m_pCamera)->GetCameraIsOpen())
            {
                // Set camera opened toggle.
                bDetectorIsReady = true;
            }
        }
    }

    // Return if this detector is ready or not.
    return bDetectorIsReady;
}

/******************************************************************************
 * @brief Accessor for the desired max FPS for this detector.
 *
 * @return int - The max frames per second the detector can run at.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-22
 ******************************************************************************/
int TagDetector::GetDetectorFPS() const
{
    // Return member variable value.
    return this->GetMainThreadMaxIPS();
}

/******************************************************************************
 * @brief Accessor for the Enable Recording Flag private member.
 *
 * @return true - Recording for this detector has been requested/flagged.
 * @return false - This detector should not be recorded.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-31
 ******************************************************************************/
bool TagDetector::GetEnableRecordingFlag() const
{
    return m_bEnableRecordingFlag;
}

/******************************************************************************
 * @brief Accessor for the camera name or path that this TagDetector is tied to.
 *
 * @return std::string - The name/path/index of the camera used by this TagDetector.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-01
 ******************************************************************************/
std::string TagDetector::GetCameraName()
{
    return m_szCameraName;
}

/******************************************************************************
 * @brief Accessor for the resolution of the process image used for tag detection.
 *
 * @return cv::Size - The resolution stored in an OpenCV cv::Size.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-01-01
 ******************************************************************************/
cv::Size TagDetector::GetProcessFrameResolution() const
{
    // Check if using a ZED camera.
    if (m_bUsingZedCamera)
    {
        // Concatenate camera model name and serial number.
        return dynamic_cast<ZEDCam*>(m_pCamera)->GetPropResolution();
    }
    else
    {
        // Concatenate camera path or index.
        return dynamic_cast<BasicCam*>(m_pCamera)->GetPropResolution();
    }
}
