/******************************************************************************
 * @brief Implements the ObjectDetector class.
 *
 * @file ObjectDetector.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-24
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include "../../vision/cameras/BasicCam.h"
#include "../../vision/cameras/ZEDCam.h"
#include "./DepthDetection.hpp"
#include "./TensorflowObjectDetection.hpp"

/// \cond
#include <future>
#include <shared_mutex>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief This class implements a modular and easy to use object detector for a single
 *      camera. Given a camera name, this class will detect objects using the depth measure
 *      from a ZED camera and/or inferenced objects from a custom trained model.
 *      This class and it's detections are ran in a different thread.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-24
 ******************************************************************************/
class ObjectDetector : public AutonomyThread<void>
{
    public:
        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////
        ObjectDetector(BasicCam* pBasicCam, const int nNumDetectedObjectsRetrievalThreads = 5, const bool bUsingGpuMats = false);
        ObjectDetector(ZEDCam* pZEDCam, const int nNumDetectedObjectsRetrievalThreads = 5, const bool bUsingGpuMats = false);
        std::future<bool> RequestDepthDetectionOverlayFrame(cv::Mat& cvFrame);
        std::future<bool> RequestTensorflowDetectionOverlayFrame(cv::Mat& cvFrame);
        std::future<bool> RequestDetectedDepthObjects(std::vector<depthobject::DepthObject>& vDepthObjects);
        std::future<bool> RequestDetectedTensorflowObjects(std::vector<tensorflowobject::TensorflowObject>& vTensorflowObjects);
        IPS& GetIPS();

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        // Class member variables.

        Camera<cv::Mat>* m_pCamera;
        bool m_bUsingZedCamera;
        bool m_bUsingGpuMats;
        int m_nNumDetectedObjectsRetrievalThreads;
        IPS m_IPS;

        // Detected objects storage.

        std::vector<depthobject::DepthObject> m_vDetectedDepthObjects;
        std::vector<tensorflowobject::TensorflowObject> m_vDetectedTensorObjects;

        // Create frames for storing images and point clouds.

        cv::Mat m_cvNormalFrame;
        cv::Mat m_cvProcFrame;
        cv::Mat m_cvDepthMeasure;
        cv::cuda::GpuMat m_cvGPUNormalFrame;
        cv::cuda::GpuMat m_cvGPUDepthMeasure;

        // Queues and mutexes for scheduling and copying data to other threads.

        std::queue<containers::FrameFetchContainer<cv::Mat>> m_qDetectedObjectDrawnOverlayFrames;
        std::queue<containers::DataFetchContainer<std::vector<depthobject::DepthObject>>> m_qDetectedDepthObjectCopySchedule;
        std::queue<containers::DataFetchContainer<std::vector<tensorflowobject::TensorflowObject>>> m_qDetectedTensorflowObjectCopySchedule;
        std::shared_mutex m_muPoolScheduleMutex;
        std::mutex m_muFrameCopyMutex;
        std::mutex m_muDepthDataCopyMutex;
        std::mutex m_muTensorflowDataCopyMutex;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////
        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
        void UpdateDetectedObjects(std::vector<depthobject::DepthObject>& vNewlyDetectedObjects);
        void UpdateDetectedObjects(std::vector<tensorflowobject::TensorflowObject>& vNewlyDetectedObjects);
};

#endif
