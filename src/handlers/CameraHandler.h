/******************************************************************************
 * @brief Defines the CameraHandler class.
 *
 * @file CameraHandler.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef CAMERA_HANDLER_H
#define CAMERA_HANDLER_H

#include "../vision/cameras/BasicCam.h"
#include "../vision/cameras/ZEDCam.h"
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

        ZEDCam* m_pMainCam;
        BasicCam* m_pLeftCam;
        BasicCam* m_pRightCam;
        RecordingHandler* m_pRecordingHandler;

    public:
        /////////////////////////////////////////
        // Define public enumerators specific to this class.
        /////////////////////////////////////////

        enum ZEDCamName    // Enum for different zed cameras.
        {
            ZEDCAM_START,
            eHeadMainCam,
            ZEDCAM_END
        };

        enum BasicCamName    // Enum for different basic cameras.
        {
            BASICCAM_START,
            eHeadLeftArucoEye,
            eHeadRightArucoEye,
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

        ZEDCam* GetZED(ZEDCamName eCameraName);
        BasicCam* GetBasicCam(BasicCamName eCameraName);
};

#endif
