// FIXME: File needs file header doc.

#ifndef ARUCO_THREAD_H_
#define ARUCO_THREAD_H_

#include <algorithm>
#include <future>
#include <shared_mutex>
#include <string>
#include <vector>

#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/opencv.hpp>

#include "./../../interfaces/AutonomyThread.hpp"
#include "./../../interfaces/Camera.hpp"
#include "./../../util/vision/ArucoSamplesUtility.hpp"
#include "./../../util/vision/FetchContainers.hpp"
#include "./Aruco.h"

// FIXME: Never #define literals at the top of class files. Instead define the variable within AutonomyGlobals file. I have added a new section for ArUco and made a
// constant identical to this #define as an example.
#define DEFAULT_DICTIONARY cv::aruco::DICT_4X4_50

/******************************************************************************
 * @brief Run's Aruco detection & camera pose estimation in a multithreading environment
 *
 * What are the threads doing?
 * Continous Thread:
 *  In this thread we are constantly getting images and depth maps from the necessary
 *  cameras. We then detect the tags in the image and estimate their location with respect to
 *  the rover.
 * Pooled Threads:
 *  Copy the vector of detected tags to all of the threads requesting it through the
 *  RequestDetectedArucoTags(...) function.
 *
 * @see Aruco.hpp
 * @author jspencerpittman (jspencerpittman@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
class ArucoThread : public AutonomyThread<void>
{
    public:
        /******************************************************************************
         * @brief Construct a new Aruco Thread object.
         *
         * @param cameraHandlerThread - the currently running camera handler    // FIXME: Remove this.
         * @param nNumDetectedTagsRetrievalThreads - the number of threads to use when fullfilling
         *                                           requests for the detected aruco tags
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-30
         ******************************************************************************/
        // FIXME: No need to redeclare the CameraHandlerThread. Just #include "AutonomyGlobals.h" and g_pCameraHandler will be accessible.
        ArucoThread(CameraHandlerThread* cameraHandlerThread, const int nNumDetectedTagsRetrievalThreads);

        /******************************************************************************
         * @brief Request the most up to date vector of detected aruco tags
         *
         * @param arucoTagVec - a reference to the vector the detected aruco tags will be saved to
         * @return std::future<bool> - status of when the request has been fulfilled
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-30
         ******************************************************************************/
        std::future<bool> RequestDetectedArucoTags(std::vector<ArucoTag>& arucoTagVec);    // FIXME: Parameter name should be vArucoTagVec

    private:
        ArucoDetector m_arucoDetector;
        CameraHandlerThread* m_pCameraHandlerThread;    // FIXME: Delete this.
        int m_nnNumDetectedTagsRetrievalThreads;        // FIXME: Remove extra n in m_n

        /* detected tags */
        std::vector<ArucoTag> m_vDetectedTags;
        std::shared_mutex m_muDetectedTagsMutex;

        /* scheduling queue */
        // FIXME: Remove reference wrapper. This will prevent you from fully copying the container out of the queue. Then we you .pop() the container the promise will go
        // out of scope and the program will crash.
        std::queue<std::reference_wrapper<containers::DataFetchContainer<std::vector<ArucoTag>&>>>
            m_qDetectedTagCopySchedule;    // FIXME: Also remove & on ArucoTag, the container already does this for you.
        std::shared_mutex m_muPoolScheduleMatrix;

        /* threading functionality */
        // FIXME: This should be ThreadedContinuousCode(), make sure you are developing within the devcontainer in VSCode as it will alert you to errors like this.
        void ThreadContinousCode() override;
        void PooledLinearCode() override;
};

#endif
