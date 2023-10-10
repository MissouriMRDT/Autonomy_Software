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

    public:
        /////////////////////////////////////////
        // Define public enumerators specific to this class.
        /////////////////////////////////////////

        enum TagDetectors    // Enum for different cameras that detectors are being ran on.
        {
            eHeadMainCam,
            eHeadLeftArucoEye
        };

        /////////////////////////////////////////
        // Declare public class methods and variables.
        /////////////////////////////////////////

        TagDetectionHandler();
        ~TagDetectionHandler();
        void StartAllDetectors();
        void StopAllDetectors();

        /////////////////////////////////////////
        // Accessors.
        /////////////////////////////////////////

        TagDetector* GetTagDetector(TagDetectors eDetectorName);
};

#endif
