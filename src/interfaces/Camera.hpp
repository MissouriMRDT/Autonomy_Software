/******************************************************************************
 * @brief Defines the Camera base interface class.
 *
 * @file Camera.hpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "../util/vision/FetchContainers.hpp"
#include "./AutonomyThread.hpp"

/// \cond
#include <atomic>
#include <future>
#include <shared_mutex>
#include <tracy/Tracy.hpp>

/// \endcond

/******************************************************************************
 * @brief This interface class serves as a base for all other classes that will
 *      implement and interface with a type of camera.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
template<class T>
class Camera : public AutonomyThread<void>
{
    public:
        /////////////////////////////////////////
        // Declare public structs specific to this class.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Defines a simple struct to hold pose data.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-01-09
         ******************************************************************************/
        struct Pose
        {
            public:
                double dPosX;    // Position X in meters.
                double dPosY;    // Position Y in meters.
                double dPosZ;    // Position Z in meters.
                double dQX;      // Quaternion X.
                double dQY;      // Quaternion Y.
                double dQZ;      // Quaternion Z.
                double dQW;      // Quaternion W.
        };

        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////
        /******************************************************************************
         * @brief Construct a new Camera object.
         *
         * @param nPropResolutionX - X res of camera.
         * @param nPropResolutionY - Y res of camera.
         * @param nPropFramesPerSecond - FPS camera is running at.
         * @param ePropPixelFormat - The pixel layout/format of the image.
         * @param dPropHorizontalFOV - The horizontal field of view.
         * @param dPropVerticalFOV - The vertical field of view.
         * @param bEnableRecordingFlag - Whether or not this camera should be recorded.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-08-18
         ******************************************************************************/
        Camera(const int nPropResolutionX,
               const int nPropResolutionY,
               const int nPropFramesPerSecond,
               const PIXEL_FORMATS ePropPixelFormat,
               const double dPropHorizontalFOV,
               const double dPropVerticalFOV,
               const bool bEnableRecordingFlag,
               const int nNumFrameRetrievalThreads = 5)

        {
            // Initialize member variables.
            m_nPropResolutionX          = nPropResolutionX;
            m_nPropResolutionY          = nPropResolutionY;
            m_nPropFramesPerSecond      = nPropFramesPerSecond;
            m_ePropPixelFormat          = ePropPixelFormat;
            m_dPropHorizontalFOV        = dPropHorizontalFOV;
            m_dPropVerticalFOV          = dPropVerticalFOV;
            m_bEnableRecordingFlag      = bEnableRecordingFlag;
            m_nNumFrameRetrievalThreads = nNumFrameRetrievalThreads;
            m_stCameraPoseOffset        = Pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
        }

        /******************************************************************************
         * @brief Destroy the Camera object.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-08-18
         ******************************************************************************/
        virtual ~Camera() {}

        /******************************************************************************
         * @brief Mutator for the Enable Recording Flag private member
         *
         * @param bEnableRecordingFlag - Whether or not recording should be enabled for this camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-12-26
         ******************************************************************************/
        void SetEnableRecordingFlag(const bool bEnableRecordingFlag) { m_bEnableRecordingFlag = bEnableRecordingFlag; }

        /******************************************************************************
         * @brief Mutator for the Camera Pose Offset private member.
         *
         * @param stPoseOffset - The pose offset of the camera from the rover center.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-01-09
         ******************************************************************************/
        void SetCameraPoseOffset(const Pose& stPoseOffset) { m_stCameraPoseOffset = stPoseOffset; }

        /******************************************************************************
         * @brief Mutator for the Camera Pose Offset private member
         *
         * @param dPosX - The X position of the camera offset.
         * @param dPosY - The Y position of the camera offset.
         * @param dPosZ - The Z position of the camera offset.
         * @param dQX - The X component of the quaternion rotation.
         * @param dQY - The Y component of the quaternion rotation.
         * @param dQZ - The Z component of the quaternion rotation.
         * @param dQW - The W component of the quaternion rotation.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-01-09
         ******************************************************************************/
        void SetCameraPoseOffset(const double dPosX, const double dPosY, const double dPosZ, const double dQX, const double dQY, const double dQZ, const double dQW)
        {
            // Update member variable.
            m_stCameraPoseOffset.dPosX = dPosX;
            m_stCameraPoseOffset.dPosY = dPosY;
            m_stCameraPoseOffset.dPosZ = dPosZ;
            m_stCameraPoseOffset.dQX   = dQX;
            m_stCameraPoseOffset.dQY   = dQY;
            m_stCameraPoseOffset.dQZ   = dQZ;
            m_stCameraPoseOffset.dQW   = dQW;
        }

        /******************************************************************************
         * @brief Accessor for the Prop Resolution private member.
         *
         * @return cv::Size - The resolution of the camera stored in OpenCV's cv::Size.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-08-19
         ******************************************************************************/
        cv::Size GetPropResolution() const { return cv::Size(m_nPropResolutionX, m_nPropResolutionY); }

        /******************************************************************************
         * @brief Accessor for the Prop Frames Per Second private member.
         *
         * @return int - The FPS of the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-08-19
         ******************************************************************************/
        int GetPropFramesPerSecond() const { return m_nPropFramesPerSecond; }

        /******************************************************************************
         * @brief Accessor for the Prop Pixel Format private member.
         *
         * @return PIXEL_FORMATS - The layout/pixel format of the image returned from
         *                      the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-08-19
         ******************************************************************************/
        PIXEL_FORMATS GetPropPixelFormat() const { return m_ePropPixelFormat; }

        /******************************************************************************
         * @brief Accessor for the Prop Horizontal F O V private member.
         *
         * @return double - The horizontal field of view of the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-08-19
         ******************************************************************************/
        double GetPropHorizontalFOV() const { return m_dPropHorizontalFOV; }

        /******************************************************************************
         * @brief Accessor for the Prop Vertical F O V private member.
         *
         * @return double - The vertical field of view of the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-08-19
         ******************************************************************************/
        double GetPropVerticalFOV() const { return m_dPropVerticalFOV; }

        /******************************************************************************
         * @brief Accessor for the Enable Recording Flag private member.
         *
         * @return true - Recording for this camera has been requested/flagged.
         * @return false - This camera should not be recorded.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-12-26
         ******************************************************************************/
        bool GetEnableRecordingFlag() const { return m_bEnableRecordingFlag; }

        /******************************************************************************
         * @brief Accessor for the Camera Pose Offset private member.
         *
         * @return Pose - The pose offset of the camera from the rover center.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-01-09
         ******************************************************************************/
        Pose GetCameraPoseOffset() const { return m_stCameraPoseOffset; }

        /******************************************************************************
         * @brief Accessor for the Camera Is Open private member.
         *
         * @return true - The camera is currently open.
         * @return false - The camera is not currently open.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-12-25
         ******************************************************************************/
        virtual bool GetCameraIsOpen() = 0;    // This is where the code to check if the camera is currently open goes.

    protected:
        // Declare protected methods and member variables.
        int m_nPropResolutionX;
        int m_nPropResolutionY;
        int m_nPropFramesPerSecond;
        int m_nNumFrameRetrievalThreads;
        PIXEL_FORMATS m_ePropPixelFormat;
        double m_dPropHorizontalFOV;
        double m_dPropVerticalFOV;
        Pose m_stCameraPoseOffset;
        std::atomic_bool m_bEnableRecordingFlag;

        // NOTE: Frame delivery is handled entirely by each concrete camera's publish-latest
        // channel (see BasicCamera::GetFramePublisher / ZEDCamera::GetFrameCPUPublisher, etc.).
        // Consumers Subscribe() to express demand and Get() the newest immutable snapshot, so
        // this interface declares no frame-request method at all.

    private:
        // Declare private methods and member variables.
};
#endif
