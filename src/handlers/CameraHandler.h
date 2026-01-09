/******************************************************************************
 * @brief Defines the CameraHandler class.
 *
 * @file CameraHandler.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef CAMERA_HANDLER_H
#define CAMERA_HANDLER_H

#include "../interfaces/BasicCamera.hpp"
#include "../interfaces/ZEDCamera.hpp"
#include "RecordingHandler.h"

/// \cond
#include <opencv2/core.hpp>

/// \endcond

/******************************************************************************
 * @brief The CameraHandler class is responsible for managing all of the
 *      camera feeds that Autonomy_Software uses for computer vision. Whether
 *      it be a USB webcam, a MJPEG stream, or a ZED camera, this class is responsible
 *      for initializing that camera and configuring it.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
class CameraHandler
{
    private:
        /////////////////////////////////////////
        // Declare private class member variables.
        /////////////////////////////////////////

        std::shared_ptr<ZEDCamera> m_pMainCam;
        std::shared_ptr<ZEDCamera> m_pRearCam;
        std::unique_ptr<RecordingHandler> m_pRecordingHandler;

    public:
        /////////////////////////////////////////
        // Define public enumerators specific to this class.
        /////////////////////////////////////////

        enum class ZEDCamName    // Enum for different zed cameras.
        {
            ZEDCAM_START,
            eHeadMainCam,
            eRearCam,
            ZEDCAM_END
        };

        enum class BasicCamName    // Enum for different basic cameras.
        {
            BASICCAM_START,
            // eExampleBasicCam,   // Uncomment and change name if basic cam needed.
            BASICCAM_END
        };

        /////////////////////////////////////////
        // Declare public class methods and variables.
        /////////////////////////////////////////

        CameraHandler();
        ~CameraHandler();
        void StartAllCameras();
        void StartRecording();
        void StopAllCameras();
        void StopRecording();

        /////////////////////////////////////////
        // Accessors.
        /////////////////////////////////////////

        std::shared_ptr<ZEDCamera> GetZED(ZEDCamName eCameraName);
        // std::shared_ptr<BasicCamera> GetBasicCam(BasicCamName eCameraName);  // Uncomment if need basic cam.
};

#endif
