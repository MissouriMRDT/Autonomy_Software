/******************************************************************************
 * @brief Provides aruco detection and pose estimation capabilities in a multithreaded
 *      fashion using both OpenCV's ArUco library and a custom Torch detection model.
 *
 * @file TagDetector.h
 * @author jspencerpittman (jspencerpittman@gmail.com)
 * @date 2023-10-01
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef TAG_DETECTOR_H
#define TAG_DETECTOR_H

#include "../../interfaces/BasicCamera.hpp"
#include "../../interfaces/ZEDCamera.hpp"
#include "../../util/threading/Publisher.hpp"
#include "../../util/vision/BoundingBoxTracking.h"
#include "../../util/vision/TagDetectionUtilty.hpp"
#include "../../util/vision/YOLOModel.hpp"

/// \cond
#include <future>
#include <memory>
#include <mutex>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief Run's Aruco detection & camera pose estimation in a multithreading environment.
 *      Given a camera, tag detection using OpenCV's ArUco library and a custom trained
 *      model for detecting general tags will be continuously ran on the camera frames.
 *
 * What are the threads doing?
 * Continuous Thread:
 *  In this thread we read the newest published image and point cloud snapshots from the
 *  camera, skipping the pass entirely if the camera has not published a new frame since
 *  last time. We then detect the tags in the image, estimate their location with respect
 *  to the rover, and publish the results for consumers to read on their own schedule.
 * Pooled Threads:
 *  Not used. Result distribution is handled by the publish-latest mechanism, so no
 *  per-consumer fan-out work remains.
 *
 * @author jspencerpittman (jspencerpittman@gmail.com), clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
class TagDetector : public AutonomyThread<void>
{
    public:
        /////////////////////////////////////////
        // Declare public methods.
        /////////////////////////////////////////
        TagDetector(std::shared_ptr<BasicCamera> pBasicCam,
                    const int nArucoCornerRefinementMaxIterations = 30,
                    const int nArucoCornerRefinementMethod        = cv::aruco::CORNER_REFINE_NONE,
                    const int nArucoMarkerBorderBits              = 1,
                    const bool bArucoDetectInvertedMarkers        = false,
                    const bool bUseAruco3Detection                = false,
                    const bool bEnableTracking                    = false,
                    const int nDetectorMaxFPS                     = 30,
                    const bool bEnableRecordingFlag               = false,
                    const int nNumDetectedTagsRetrievalThreads    = 5,
                    const bool bUsingGpuMats                      = false);
        TagDetector(std::shared_ptr<ZEDCamera> pZEDCam,
                    const int nArucoCornerRefinementMaxIterations = 30,
                    const int nArucoCornerRefinementMethod        = cv::aruco::CORNER_REFINE_NONE,
                    const int nArucoMarkerBorderBits              = 1,
                    const bool bArucoDetectInvertedMarkers        = false,
                    const bool bUseAruco3Detection                = false,
                    const bool bEnableTracking                    = false,
                    const int nDetectorMaxFPS                     = 30,
                    const bool bEnableRecordingFlag               = false,
                    const int nNumDetectedTagsRetrievalThreads    = 5,
                    const bool bUsingGpuMats                      = false);
        ~TagDetector();
        bool InitTorchDetection(const std::string& szModelPath,
                                yolomodel::pytorch::PyTorchInterpreter::HardwareDevices eDevice = yolomodel::pytorch::PyTorchInterpreter::HardwareDevices::eCUDA);

        /////////////////////////////////////////
        // Mutators.
        /////////////////////////////////////////

        void EnableTorchDetection(const float fMinObjectConfidence = 0.4f, const float fNMSThreshold = 0.6f);
        void DisableTorchDetection();
        void SetDetectorMaxFPS(const int nRecordingFPS);
        void SetEnableRecordingFlag(const bool bEnableRecordingFlag);

        /////////////////////////////////////////
        // Accessors.
        /////////////////////////////////////////

        bool GetIsReady();
        int GetDetectorMaxFPS() const;
        bool GetEnableRecordingFlag() const;
        std::string GetCameraName();
        cv::Size GetProcessFrameResolution() const;

        /////////////////////////////////////////
        // Publish-latest data channels out. Consumers hold a Reader and Get() the newest
        // immutable snapshot without blocking this detector's loop.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Read handle for the detection-overlay frame channel.
         * @return pubsub::Reader<cv::Mat> - A demand-carrying read handle for the detection overlay channel.
         ******************************************************************************/
        pubsub::Reader<cv::Mat> GetDetectionOverlayReader() { return m_pubDetectionOverlay.CreateReader(); }

        /******************************************************************************
         * @brief Read handle for the last-good detection-overlay frame channel.
         * @return pubsub::Reader<cv::Mat> - A demand-carrying read handle for the last-good overlay channel.
         ******************************************************************************/
        pubsub::Reader<cv::Mat> GetLastGoodOverlayReader() { return m_pubLastGoodOverlay.CreateReader(); }

        /******************************************************************************
         * @brief Read handle for the detected aruco tags channel.
         * @return pubsub::Reader<std::vector<tagdetectutils::ArucoTag>> - A demand-carrying read handle for the tags channel.
         ******************************************************************************/
        pubsub::Reader<std::vector<tagdetectutils::ArucoTag>> GetDetectedTagsReader() { return m_pubDetectedTags.CreateReader(); }

        /******************************************************************************
         * @brief Read the newest published tag list directly, without the caller having to
         *      hold a Reader.
         *
         *      This is safe ONLY because the tags channel is published unconditionally: the
         *      tags are already computed by the detection pass, so publishing them costs a
         *      small vector copy and is never gated on HasReaders(). To keep that consistent
         *      with the demand model rather than merely convention, this detector holds its
         *      own Reader on the channel (m_rdSelfDetectedTags) - so demand genuinely always
         *      exists, and this read genuinely goes through a Reader.
         *
         *      Do NOT copy this pattern to the overlay channels. Those ARE demand gated,
         *      because cloning a frame per iteration is expensive, and a caller that wants
         *      one must hold a real Reader for as long as it intends to read.
         *
         * @return pubsub::SharedSnapshot<std::vector<tagdetectutils::ArucoTag>> - The newest
         *                  immutable tag list, or nullptr if nothing has been published yet.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-09-07
         ******************************************************************************/
        pubsub::SharedSnapshot<std::vector<tagdetectutils::ArucoTag>> GetLatestDetectedTags() const { return m_rdSelfDetectedTags.Get(); }

        /******************************************************************************
         * @brief Accessor for the number of detection passes skipped because the camera
         *      had not published a new frame since the last pass. Used to verify that the
         *      sequence-number short circuit is actually saving work.
         *
         * @return unsigned long long - The cumulative number of skipped detection passes.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        unsigned long long GetSkippedFrameCount() const { return m_ullSkippedFrameCount.load(std::memory_order_relaxed); }

        /******************************************************************************
         * @brief Accessor for the number of detection passes skipped because the frame and
         *      the point cloud came from different camera grabs. Should be near zero; a
         *      steadily rising value means this detector's loop is racing the camera's
         *      publish often enough to be worth publishing the two together instead.
         *
         * @return unsigned long long - The cumulative number of mismatched-grab skips.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-09-07
         ******************************************************************************/
        unsigned long long GetMismatchedGrabCount() const { return m_ullMismatchedGrabCount.load(std::memory_order_relaxed); }

    private:
        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
        void UpdateDetectedTags(std::vector<tagdetectutils::ArucoTag>& vNewlyDetectedTags);
        void EnsureCameraReaders();
        bool LoadLatestCameraFrames();

        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        // Class member variables.

        std::shared_ptr<Camera<cv::Mat>> m_pCamera;
        cv::aruco::ArucoDetector m_cvArucoDetector;
        cv::aruco::DetectorParameters m_cvArucoDetectionParams;
        cv::aruco::Dictionary m_cvTagDictionary;
        std::shared_ptr<yolomodel::pytorch::PyTorchInterpreter> m_pTorchDetector;
        std::atomic<float> m_fTorchMinObjectConfidence;
        std::atomic<float> m_fTorchNMSThreshold;
        std::atomic_bool m_bTorchInitialized;
        std::atomic_bool m_bTorchEnabled;
        std::shared_ptr<tracking::MultiTracker> m_pMultiTracker;
        bool m_bUsingZedCamera;
        bool m_bUsingGpuMats;
        bool m_bCameraIsOpened;

        // Edge-triggered logging of the camera's readiness. This detector idles (rather than
        // stopping itself) whenever its camera is not open, so without this it would log the same
        // "waiting for camera" line every iteration at its full detector FPS.
        bool m_bLastKnownCameraOpenState = true;
        bool m_bEnableTracking;
        int m_nNumDetectedTagsRetrievalThreads;
        std::string m_szCameraName;
        std::atomic_bool m_bEnableRecordingFlag;

        // Detected tags storage.

        std::vector<tagdetectutils::ArucoTag> m_vNewlyDetectedTags;
        std::vector<tagdetectutils::ArucoTag> m_vDetectedArucoTags;

        // Rover position for tag geolocalization.
        geoops::RoverPose m_stRoverPose;

        // Create frames for storing images and point clouds.

        cv::Mat m_cvFrame;
        cv::Mat m_cvPointCloud;

        // The newest overlay that had tags on it, held as its published snapshot instead of a clone, and whether it
        // still has to be copied to the last-good channel.
        pubsub::SharedSnapshot<cv::Mat> m_pLastGoodOverlaySnapshot;
        bool m_bLastGoodOverlayPending = false;

        // Demand for the camera data this detector consumes. Held for this detector's whole
        // lifetime so the camera retrieves and publishes only what is actually being used.

        std::once_flag m_ocCameraReadersOnce;
        // Typed read handles onto the camera channels this detector consumes. Only the pair
        // matching the camera's memory mode is ever active; the other stays default constructed
        // and inactive. Each handle both expresses demand (the camera retrieves nothing without
        // it) and is the only way to read the channel, so the two can never drift apart.
        pubsub::Reader<cv::Mat> m_rdCameraFrameCPU;
        pubsub::Reader<cv::Mat> m_rdCameraPointCloudCPU;
        pubsub::Reader<cv::cuda::GpuMat> m_rdCameraFrameGPU;
        pubsub::Reader<cv::cuda::GpuMat> m_rdCameraPointCloudGPU;

        // Sequence number of the last camera frame this detector actually ran detection on. Used
        // to skip an entire detection pass when the camera has not published a new frame yet.

        unsigned long long m_ullLastProcessedFrameSequence = 0;
        unsigned long long m_ullLastProcessedCloudSequence = 0;         // Last point cloud this detector waited for. ZED cameras only.
        std::atomic<unsigned long long> m_ullMismatchedGrabCount{0};    // Passes skipped because frame and cloud came from different grabs.
        std::atomic<unsigned long long> m_ullSkippedFrameCount{0};

        // Publish-latest data channels out (see accessors above). Each is given an explicit
        // preallocation and growth ceiling so steady state allocates nothing and a consumer that
        // leaks snapshots trips the ceiling and is logged as an error.
        // The two overlay channels are demand gated (a full-frame clone each), while the detected
        // tag channel publishes unconditionally: the tags are already computed by the detection
        // pass, so publishing them costs only a small vector copy and every consumer of this
        // detector wants them.
        pubsub::Publisher<cv::Mat> m_pubDetectionOverlay{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<cv::Mat> m_pubLastGoodOverlay{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};
        pubsub::Publisher<std::vector<tagdetectutils::ArucoTag>> m_pubDetectedTags{constants::PUBLISHER_POOL_PREALLOC, constants::PUBLISHER_POOL_GROWTH_CEILING};

        // This detector's own demand on its tags channel. The tags are published every pass
        // regardless of consumers, and free functions such as statemachine::LoadDetectedTags
        // have nowhere natural to keep a Reader - so the detector holds one itself. That makes
        // "this channel always has demand" a fact about the object rather than a comment, and
        // gives GetLatestDetectedTags() a real Reader to read through.
        pubsub::Reader<std::vector<tagdetectutils::ArucoTag>> m_rdSelfDetectedTags{m_pubDetectedTags.CreateReader()};

        // The camera snapshots this pass is working from. Held (rather than deep copied into
        // member Mats) because a published snapshot is immutable and reference counted for as
        // long as we hold it - copying it again just to read it duplicated a full frame and a
        // full point cloud on every pass.
        pubsub::SharedSnapshot<cv::Mat> m_pFrameSnapshot;
        pubsub::SharedSnapshot<cv::Mat> m_pPointCloudSnapshot;
};

#endif
