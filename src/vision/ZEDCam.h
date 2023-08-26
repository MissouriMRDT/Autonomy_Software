/******************************************************************************
 * @brief Defines the ZedCam class.
 *
 * @file ZedCam.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-25
 *
 * @copyright Copyright MRDT 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef ZEDCAM_H
#define ZEDCAM_H

#include <future>
#include <sl/Camera.hpp>

#include "../interfaces/Camera.hpp"

class ZedCam : public Camera<sl::Mat>
{
    private:
        // Declare private methods and functions variables.
        sl::Camera m_slCamera;
        sl::Mat m_slFrame;

    public:
        // Declare public methods and member variables.
        ZedCam(const int nPropResolutionX, const int nPropResolutionY, const int nPropFramesPerSecond, const double dPropHorizontalFOV, const double dPropVerticalFOV);
        ~ZedCam();
        sl::Mat GrabFrame(const bool bGrabRaw = false) override;
        sl::Mat GrabDepth();
        sl::Mat GrabPointCloud();
        void EnablePositionalTracking();
        sl::Pose GetPosition();
        void ResetPositionalTracking();
        void DisablePositionalTracking();
        sl::SensorsData GetSensorsData();
        void EnableSpatialMapping();
        std::future<sl::FusedPointCloud> ExtractSpatialMapAsyc();

        // Getters.
        template<typename T>
        T GetCameraLocation();
        bool GetCameraIsOpen() override;
};
#endif
