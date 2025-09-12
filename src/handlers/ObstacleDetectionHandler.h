#ifndef OBSTACLE_DETECTION_HANDLER_H
#define OBSTACLE_DETECTION_HANDLER_H

#include "../vision/obstacles/ObstacleDetector.h"
#include "RecordingHandler.h"

/******************************************************************************
 * @brief The ObstacleDetectionHandler class is responsible for managing all of the
 *      different detectors that Autonomy_Software uses for obstacle detection.
 *      Whether it be for simple detection using a depth measure and blobs or detection using a custom
 *      tensorflow model, the detectors are created and stored here.
 *
 *
 * @author UhOhDonovan (donovan@balehaus.org)
 * @date 2025-09-03
 ******************************************************************************/
class ObstacleDetectionHandler
{
    private:
        /////////////////////////////////////////
        // Declare private class member variables.
        /////////////////////////////////////////

        std::shared_ptr<ObstacleDetector> m_pObstacleDetectorMainCam;
        std::unique_ptr<RecordingHandler> m_pRecordingHandler;

    public:
        /////////////////////////////////////////
        // Define public enumerators specific to this class.
        /////////////////////////////////////////

        enum class ObstacleDetectors    // Enum for different cameras that detectors are being ran on.
        {
            OBSTACLEDETECTOR_START,
            eHeadMainCam,
            OBSTACLEDETECTOR_END
        };

        /////////////////////////////////////////
        // Declare public class methods and variables.
        /////////////////////////////////////////

        ObstacleDetectionHandler();
        ~ObstacleDetectionHandler();
        void StartAllDetectors();
        void StartRecording();
        void StopAllDetectors();
        void StopRecording();

        /////////////////////////////////////////
        // Accessors.
        /////////////////////////////////////////

        std::shared_ptr<ObstacleDetector> GetObstacleDetector(ObstacleDetectors eDetectorName);
};

#endif
