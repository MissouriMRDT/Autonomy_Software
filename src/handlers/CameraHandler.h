/******************************************************************************
 * @brief Defines the CameraHandler class.
 *
 * @file CameraHandler.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef CAMERA_HANDLER__H
#define CAMERA_HANDLER__H

#include <opencv2/core.hpp>

#include "../vision/cameras/BasicCam.h"
#include "../vision/cameras/ZEDCam.h"

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

    public:
        /////////////////////////////////////////
        // Define public enumerators specific to this class.
        /////////////////////////////////////////

        enum ZEDCamName    // Enum for different zed cameras.
        {
            eHeadMainCam
        };

        enum BasicCamName    // Enum for different basic cameras.
        {
            eHeadLeftArucoEye,
            eHeadRightAcuroEye
        };

        /////////////////////////////////////////
        // Declare public class methods and variables.
        /////////////////////////////////////////

        CameraHandler();
        ~CameraHandler();
        void StartAllCameras();
        void StopAllCameras();

        /////////////////////////////////////////
        // Accessors.
        /////////////////////////////////////////

        ZEDCam* GetZED(ZEDCamName eCameraName);
        BasicCam* GetBasicCam(BasicCamName eCameraName);
};

#endif
