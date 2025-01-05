/******************************************************************************
 * @brief Defines and implements the BasicCamera interface class.
 *
 * @file BasicCamera.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-22
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef BASICCAMERA_HPP
#define BASICCAMERA_HPP

#include "Camera.hpp"

/// \cond

/// \endcond

/******************************************************************************
 * @brief This class serves as a middle inheritor between the Camera interface
 *      and the BasicCam class. BasicCam and SIMBasicCam will inherit from this.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-22
 ******************************************************************************/
class BasicCamera : public Camera<cv::Mat>
{
    public:
        /******************************************************************************
         * @brief Construct a new Basic Camera object.
         *
         * @param szCameraPath - The file path to the camera hardware.
         * @param nPropResolutionX - X res of camera.
         * @param nPropResolutionY - Y res of camera.
         * @param nPropFramesPerSecond - FPS camera is running at.
         * @param ePropPixelFormat - The pixel layout/format of the image.
         * @param dPropHorizontalFOV - The horizontal field of view.
         * @param dPropVerticalFOV - The vertical field of view.
         * @param bEnableRecordingFlag - Whether or not this camera should be recorded.
         * @param nNumFrameRetrievalThreads - The number of threads to use for frame queueing and copying.
         ******************************************************************************/
        BasicCamera(const std::string szCameraPath,
                    const int nPropResolutionX,
                    const int nPropResolutionY,
                    const int nPropFramesPerSecond,
                    const PIXEL_FORMATS ePropPixelFormat,
                    const double dPropHorizontalFOV,
                    const double dPropVerticalFOV,
                    const bool bEnableRecordingFlag,
                    const int nNumFrameRetrievalThreads) :
            Camera(nPropResolutionX,
                   nPropResolutionY,
                   nPropFramesPerSecond,
                   ePropPixelFormat,
                   dPropHorizontalFOV,
                   dPropVerticalFOV,
                   bEnableRecordingFlag,
                   nNumFrameRetrievalThreads)
        {
            // Initialize member variables.
            m_nCameraIndex = -1;
            m_szCameraPath = szCameraPath;
        }

        /******************************************************************************
         * @brief Construct a new Basic Camera object. Overloaded for dev/video indexes.
         *
         * @param nCameraIndex - The video index that the camera is connected on.
         * @param nPropResolutionX - X res of camera.
         * @param nPropResolutionY - Y res of camera.
         * @param nPropFramesPerSecond - FPS camera is running at.
         * @param ePropPixelFormat - The pixel layout/format of the image.
         * @param dPropHorizontalFOV - The horizontal field of view.
         * @param dPropVerticalFOV - The vertical field of view.
         * @param bEnableRecordingFlag - Whether or not this camera should be recorded.
         * @param nNumFrameRetrievalThreads - The number of threads to use for frame queueing and copying.
         ******************************************************************************/
        BasicCamera(const int nCameraIndex,
                    const int nPropResolutionX,
                    const int nPropResolutionY,
                    const int nPropFramesPerSecond,
                    const PIXEL_FORMATS ePropPixelFormat,
                    const double dPropHorizontalFOV,
                    const double dPropVerticalFOV,
                    const bool bEnableRecordingFlag,
                    const int nNumFrameRetrievalThreads) :
            Camera(nPropResolutionX,
                   nPropResolutionY,
                   nPropFramesPerSecond,
                   ePropPixelFormat,
                   dPropHorizontalFOV,
                   dPropVerticalFOV,
                   bEnableRecordingFlag,
                   nNumFrameRetrievalThreads)
        {
            // Initialize member variables.
            m_nCameraIndex = nCameraIndex;
            m_szCameraPath = "";
        }

        /******************************************************************************
         * @brief Destroy the Basic Camera object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        virtual ~BasicCamera() {}

        /******************************************************************************
         * @brief The code inside this private method runs in a separate thread, but still
         *      has access to this*. This method continuously get new frames from the OpenCV
         *      VideoCapture object and stores it in a member variable. Then a thread pool is
         *      started and joined once per iteration to mass copy the frames and/or measure
         *      to any other thread waiting in the queues.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        virtual void ThreadedContinuousCode() {}

        /******************************************************************************
         * @brief This method holds the code that is ran in the thread pool started by
         *      the ThreadedLinearCode() method. It copies the data from the different
         *      data objects to references of the same type stored in a vector queued up by the
         *      Grab methods.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        virtual void PooledLinearCode() {}

        /******************************************************************************
         * @brief Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
         *      Remember, this code will be ran in whatever, class/thread calls it.
         *
         * @param cvFrame - A reference to the cv::Mat to store the frame in.
         * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
         *                          Value will be true if frame was successfully retrieved.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        std::future<bool> RequestFrameCopy(cv::Mat& cvFrame) override = 0;

        /******************************************************************************
         * @brief Accessor for the cameras path or video index.
         *
         * @return std::string - The path or index of the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-22
         ******************************************************************************/
        virtual std::string GetCameraLocation() const { return ""; }

    protected:
        // Declare protected methods and member variables.
        int m_nCameraIndex;
        std::string m_szCameraPath;

    private:
        // Declare private methods and member variables.
};

#endif    // BASIC_CAMERA_HPP
