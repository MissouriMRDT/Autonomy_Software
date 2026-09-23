/******************************************************************************
 * @brief Defines the TagDetectionHandler class.
 *
 * @file TagDetectionHandler.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef TAG_DETECTION_HANDLER_H
#define TAG_DETECTION_HANDLER_H

#include "../vision/aruco/TagDetector.h"
#include "RecordingHandler.h"

/******************************************************************************
 * @brief The TagDetectionHandler class is responsible for managing all of the
 *      different detectors that Autonomy_Software uses for AR tag detection.
 *      Whether it be for detection using OpenCV's ArUco or detection using a custom
 *      model, the detectors are created and stored here.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 ******************************************************************************/
class TagDetectionHandler
{
    private:
        /////////////////////////////////////////
        // Declare private class member variables.
        /////////////////////////////////////////

        std::shared_ptr<TagDetector> m_pTagDetectorMainCam;
        std::shared_ptr<TagDetector> m_pTagDetectorRearCam;
        std::unique_ptr<RecordingHandler> m_pRecordingHandler;

        // Persistent demand for each detector's overlay channels. A detector only clones and
        // publishes overlay frames while a Reader is alive, so this handler holds one for the
        // lifetime of its detectors - and every read of those channels goes through THESE
        // handles, via the accessors below. Callers elsewhere must not create their own
        // Reader just to read: demand that exists for the duration of one expression is
        // demand the detector never observes, so the channel would simply never publish.
        pubsub::Reader<cv::Mat> m_rdMainCamOverlay;
        pubsub::Reader<cv::Mat> m_rdMainCamLastGoodOverlay;
        pubsub::Reader<cv::Mat> m_rdRearCamOverlay;
        pubsub::Reader<cv::Mat> m_rdRearCamLastGoodOverlay;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        cv::Mat CopyOverlaySnapshot(const pubsub::Reader<cv::Mat>& rdOverlayReader, const std::string& szChannelName);

    public:
        /////////////////////////////////////////
        // Define public enumerators specific to this class.
        /////////////////////////////////////////

        enum class TagDetectors    // Enum for different cameras that detectors are being ran on.
        {
            TAGDETECTOR_START,
            eHeadMainCam,
            eRearCam,
            TAGDETECTOR_END
        };

        /////////////////////////////////////////
        // Declare public class methods and variables.
        /////////////////////////////////////////

        TagDetectionHandler();
        ~TagDetectionHandler();
        void StartAllDetectors();
        void StartRecording();
        void StopAllDetectors();
        void StopRecording();

        /////////////////////////////////////////
        // Accessors.
        /////////////////////////////////////////

        std::shared_ptr<TagDetector> GetTagDetector(TagDetectors eDetectorName);

        cv::Mat GetDetectionOverlayFrame(TagDetectors eDetector = TagDetectors::eHeadMainCam);
        cv::Mat GetLastGoodOverlayFrame(TagDetectors eDetector = TagDetectors::eHeadMainCam);
        cv::Mat GetLastGoodOverlayFrameForDetector(const std::string& szDetectorUUID);
};

#endif
