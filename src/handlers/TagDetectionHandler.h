/******************************************************************************
 * @brief Defines the TagDetectionHandler class.
 *
 * @file TagDetectionHandler.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-07
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef TAG_DETECTION_HANDLER_H
#define TAG_DETECTION_HANDLER_H

#include "../vision/aruco/TagDetector.h"
#include "RecordingHandler.h"

/******************************************************************************
 * @brief The TagDetectionHandler class is responsible for managing all of the
 *      different detectors that Autonomy_Software uses for AR tag detection.
 *      Whether it be for detection using OpenCV's ArUco or detection using a custom
 *      tensorflow model, the detectors are created and stored here.
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

        TagDetector* m_pTagDetectorMainCam;
        TagDetector* m_pTagDetectorLeftCam;
        TagDetector* m_pTagDetectorRightCam;
        TagDetector* m_pTagDetectorGroundCam;
        RecordingHandler* m_pRecordingHandler;

    public:
        /////////////////////////////////////////
        // Define public enumerators specific to this class.
        /////////////////////////////////////////

        enum class TagDetectors    // Enum for different cameras that detectors are being ran on.
        {
            TAGDETECTOR_START,
            eHeadMainCam,
            eFrameLeftCam,
            eFrameRightCam,
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

        TagDetector* GetTagDetector(TagDetectors eDetectorName);
};

#endif
