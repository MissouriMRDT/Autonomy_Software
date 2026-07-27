/******************************************************************************
 * @brief Defines the RecordingHandler class.
 *
 * @file RecordingHandler.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-26
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef RECORDING_HANDLER_H
#define RECORDING_HANDLER_H

#include "../interfaces/BasicCamera.hpp"
#include "../util/threading/Publisher.hpp"
#include "../vision/aruco/TagDetector.h"
#include "../vision/cameras/ZEDCam.h"
#include "../vision/objects/ObjectDetector.h"

/// \cond
#include <opencv2/opencv.hpp>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief The RecordingHandler class serves to enumerate the cameras available from
 *      the CameraHandler and retrieve and write frames from each camera to the filesystem.
 *      The recording of each camera can be disabled through constants and the framerate
 *      of the recording can be adjusted to save CPU-time and resources.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-26
 ******************************************************************************/
class RecordingHandler : public AutonomyThread<void>
{
    public:
        /////////////////////////////////////////
        // Define public enumerators specific to this class.
        /////////////////////////////////////////

        // Enum used to select which mode the recorder should run in.
        enum class RecordingMode
        {
            eCameraHandler,            // Record video feeds from the CameraHandler.
            eTagDetectionHandler,      // Record video feeds from the TagDetectionHandler.
            eObjectDetectionHandler    // Record video feeds from the ObjectDetectionHandler.
        };

        /////////////////////////////////////////
        // Declare public class methods and variables.
        /////////////////////////////////////////

        RecordingHandler(RecordingMode eRecordingMode);
        ~RecordingHandler();

        /////////////////////////////////////////
        // Mutators.
        /////////////////////////////////////////

        void SetRecordingFPS(const int nRecordingFPS);

        /////////////////////////////////////////
        // Accessors.
        /////////////////////////////////////////

        int GetRecordingFPS() const;

    private:
        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
        void UpdateRecordableCameras();
        void RequestAndWriteCameraFrames();
        void UpdateRecordableTagDetectors();
        void RequestAndWriteTagDetectorFrames();
        void UpdateRecordableObjectDetectors();
        void RequestAndWriteObjectDetectorFrames();
        void WriteFrameToVideo(const int nFeedIndex);

        /////////////////////////////////////////
        // Declare private class member variables.
        /////////////////////////////////////////

        int m_nTotalVideoFeeds;
        RecordingMode m_eRecordingMode;
        std::vector<std::shared_ptr<ZEDCamera>> m_vZEDCameras;
        std::vector<std::shared_ptr<BasicCamera>> m_vBasicCameras;
        std::vector<std::shared_ptr<TagDetector>> m_vTagDetectors;
        std::vector<std::shared_ptr<ObjectDetector>> m_vObjectDetectors;
        std::vector<cv::VideoWriter> m_vCameraWriters;
        std::vector<bool> m_vRecordingToggles;
        std::vector<cv::Mat> m_vFrames;
        std::vector<cv::cuda::GpuMat> m_vGPUFrames;

        // Demand handles for the publish-latest channels this handler records from. A camera or
        // detector only produces a data type while at least one Subscription for it is alive, so
        // these are taken when a feed's recording is enabled and released when it is disabled.
        // This replaces the old per-iteration request/future fan-out entirely.
        std::vector<pubsub::Subscription> m_vFrameSubscriptions;
};
#endif
