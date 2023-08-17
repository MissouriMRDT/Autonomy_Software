/******************************************************************************
 * @brief Defines the CameraHandlerThread class.
 *
 * @file CameraHandlerThread.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include <opencv2/core.hpp>

#include "../interfaces/AutonomyThread.hpp"

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
class CameraHandlerThread : public AutonomyThread<cv::Mat>
{
    private:
        // Declare private class functions and variables.
        void ThreadedContinuousCode();
        cv::Mat PooledLinearCode();

    public:
        // Declare public class methods and variables.
        CameraHandlerThread();
        ~CameraHandlerThread();
};
