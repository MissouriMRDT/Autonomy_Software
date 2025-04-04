/******************************************************************************
 * @brief Defines the ObjectDetectionHandler class.
 *
 * @file ObjectDetectionHandler.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-23
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef OBJECT_DETECTION_HANDLER_H
#define OBJECT_DETECTION_HANDLER_H

#include "../vision/objects/ObjectDetector.h"

/******************************************************************************
 * @brief The ObjectDetectionHandler class is responsible for managing all of the
 *      different detectors that Autonomy_Software uses for object and obstacle detection. (excluding AR tags)
 *      Whether it be for simple detection using a depth measure and blobs or detection using a custom
 *      tensorflow model, the detectors are created and stored here.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-23
 ******************************************************************************/
class ObjectDetectionHandler
{
    private:
        /////////////////////////////////////////
        // Declare private class member variables.
        /////////////////////////////////////////

        std::shared_ptr<ObjectDetector> m_pObjectDetectorMainCam;
        std::shared_ptr<ObjectDetector> m_pObjectDetectorLeftCam;
        std::shared_ptr<ObjectDetector> m_pObjectDetectorRightCam;

    public:
        /////////////////////////////////////////
        // Define public enumerators specific to this class.
        /////////////////////////////////////////

        enum class ObjectDetectors    // Enum for different cameras that detectors are being ran on.
        {
            eHeadMainCam,
            eFrameLeftCam,
            eFrameRightCam
        };

        /////////////////////////////////////////
        // Declare public class methods and variables.
        /////////////////////////////////////////

        ObjectDetectionHandler();
        ~ObjectDetectionHandler();
        void StartAllDetectors();
        void StopAllDetectors();

        /////////////////////////////////////////
        // Accessors.
        /////////////////////////////////////////

        std::shared_ptr<ObjectDetector> GetObjectDetector(ObjectDetectors eDetectorName);
};

#endif
