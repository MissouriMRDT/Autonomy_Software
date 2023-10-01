/******************************************************************************
 * @brief provides aruco detection and pose estimation capabilities in a multithreaded
 *          fashion
 *
 * @file ArucoThread.h
 * @author jspencerpittman (jspencerpittman@gmail.com)
 * @date 2023-10-01
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef ARUCO_THREAD_H_
#define ARUCO_THREAD_H_

#include <algorithm>
#include <future>
#include <shared_mutex>
#include <string>
#include <vector>

#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/opencv.hpp>

#include "./../../AutonomyGlobals.h"
#include "./../../interfaces/AutonomyThread.hpp"
#include "./../../interfaces/Camera.hpp"
#include "./../../util/vision/ArucoSamplesUtility.hpp"
#include "./../../util/vision/FetchContainers.hpp"
#include "./Aruco.h"

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
         * @param nNumDetectedTagsRetrievalThreads - the number of threads to use when fullfilling
         *                                           requests for the detected aruco tags
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-30
         ******************************************************************************/
        ArucoThread(const int nNumDetectedTagsRetrievalThreads);

        /******************************************************************************
         * @brief Request the most up to date vector of detected aruco tags
         *
         * @param arucoTagVec - a reference to the vector the detected aruco tags will be saved to
         * @return std::future<bool> - status of when the request has been fulfilled
         *
         * @author jspencerpittman (jspencerpittman@gmail.com)
         * @date 2023-09-30
         ******************************************************************************/
        std::future<bool> RequestDetectedArucoTags(std::vector<ArucoTag>& vArucoTagVec);

    private:
        ArucoDetector m_arucoDetector;
        ZEDCam* m_pZedCam;
        int m_nNumDetectedTagsRetrievalThreads;

        /* detected tags */
        std::vector<ArucoTag> m_vDetectedTags;
        std::shared_mutex m_muDetectedTagsMutex;

        /* scheduling queue */
        std::queue<containers::DataFetchContainer<std::vector<ArucoTag>>> m_qDetectedTagCopySchedule;
        std::mutex m_muPoolScheduleMatrix;

        /* threading functionality */
        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
};

#endif
