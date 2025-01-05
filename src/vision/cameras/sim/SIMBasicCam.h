/******************************************************************************
 * @brief Defines the SIMBasicCam class.
 *
 * @file SIMBasicCam.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef SIMBASICCAM_H
#define SIMBASICCAM_H

#include "../../../interfaces/AutonomyThread.hpp"
#include "../../../interfaces/BasicCamera.hpp"

/// \cond
#include <opencv2/opencv.hpp>

/// \endcond

/******************************************************************************
 * @brief This class implements and interfaces with the SIM cameras and data.
 *  It is designed in such a way that multiple other classes/threads
 *  can safely call any method of an object of this class withing resource corruption
 *  or slowdown of the camera.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
class SIMBasicCam : public BasicCamera
{
    public:
        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////
        SIMBasicCam(const std::string szCameraPath,
                    const int nPropResolutionX,
                    const int nPropResolutionY,
                    const int nPropFramesPerSecond,
                    const PIXEL_FORMATS ePropPixelFormat,
                    const double dPropHorizontalFOV,
                    const double dPropVerticalFOV,
                    const bool bEnableRecordingFlag,
                    const int nNumFrameRetrievalThreads = 10);
        ~SIMBasicCam();
        std::future<bool> RequestFrameCopy(cv::Mat& cvFrame) override;

        /////////////////////////////////////////
        // Getters.
        /////////////////////////////////////////

        bool GetCameraIsOpen() override;
        std::string GetCameraLocation() const override;

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        // Basic Camera specific.

        cv::VideoCapture m_cvCamera;

        // Mats for storing frames.

        cv::Mat m_cvFrame;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////
        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
};
#endif
