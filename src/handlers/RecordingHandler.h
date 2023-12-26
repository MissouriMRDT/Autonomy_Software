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

#include "../interfaces/AutonomyThread.hpp"
#include "../interfaces/Camera.hpp"

#include <vector>

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

        /////////////////////////////////////////
        // Declare public class methods and variables.
        /////////////////////////////////////////

        RecordingHandler();
        ~RecordingHandler();
        void StartRecording();
        void StopRecording();

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

        /////////////////////////////////////////
        // Declare private class member variables.
        /////////////////////////////////////////

        int m_nRecordingFPS;
        std::vector<Camera<cv::Mat>> m_vCameras;
};
#endif
