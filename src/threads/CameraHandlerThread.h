/******************************************************************************
 * @brief Defines the CameraHandlerThread class.
 *
 * @file CameraHandlerThread.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef CAMERA_HANDLER_THREAD_H
#define CAMERA_HANDLER_THREAD_H

#include <opencv2/core.hpp>

#include "../interfaces/AutonomyThread.hpp"
#include "../vision/cameras/BasicCam.h"
#include "../vision/cameras/ZEDCam.h"

/******************************************************************************
 * @brief The CameraHandlerThread class is responsible for managing all of the
 *      camera feeds that Autonomy_Software uses for computer vision. Whether
 *      it be a USB webcam, a MJPEG stream, or a ZED camera, this class is responsible
 *      for initializing that camera and retrieving frames from it.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
class CameraHandlerThread
{
    private:
        // Declare private class member variables.
        ZEDCam* m_pMainCam;
        BasicCam* m_pLeftCam;

    public:
        // Define public enumerators specific to this class.
        enum ZEDCamName    // Enum for different zed cameras.
        {
            eHeadMainCam
        };

        enum BasicCamName    // Enum for different basic cameras.
        {
            eHeadLeftArucoEye,
            eHeadRightAcuroEye
        };

        // Declare public class methods and variables.
        CameraHandlerThread();
        ~CameraHandlerThread();
        void StartAllCameras();

        // Accessors.
        ZEDCam* GetZED(ZEDCamName eCameraName);
        BasicCam* GetBasicCam(BasicCamName eCameraName);
};

#endif
