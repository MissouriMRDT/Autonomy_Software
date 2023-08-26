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
#include <vector>

#include "../AutonomyConstants.h"
#include "../interfaces/Camera.hpp"

class ZedCam : public Camera<sl::Mat>
{
    private:
        // Declare private methods and functions variables.
        sl::Camera m_slCamera;
        sl::Mat m_slFrame;
        sl::Mat m_slDepth;
        sl::Mat m_slPointCloud;

    public:
        // Declare public methods and member variables.
        ZedCam(const int nPropResolutionX, const int nPropResolutionY, const int nPropFramesPerSecond, const double dPropHorizontalFOV, const double dPropVerticalFOV);
        ~ZedCam();
        sl::Mat GrabFrame(const bool bGrabRaw = false) override;
        sl::Mat GrabDepth(const bool bGrabRaw = false);
        sl::Mat GrabPointCloud(const bool bGrabRaw = false);
        sl::ERROR_CODE ResetPositionalTracking();
        sl::ERROR_CODE IngestCustomBoxObjects(std::vector<sl::CustomBoxObjectData> vCustomObjects);
        sl::ERROR_CODE RebootCamera();

        // Setters for class member variables.
        sl::ERROR_CODE EnablePositionalTracking();
        void DisablePositionalTracking();
        sl::ERROR_CODE SetPositionalPose();
        sl::ERROR_CODE EnableSpatialMapping();
        void DisableSpatialMapping();
        sl::ERROR_CODE EnableObjectDetection();
        void DisableObjectDetection();

        // Accessors for class member variables.
        bool GetCameraIsOpen() const override;
        sl::Pose GetPositionalPose() const;
        bool GetPositionalTrackingEnabled() const;
        sl::SensorsData GetSensorsData() const;
        sl::SPATIAL_MAPPING_STATE GetSpatialMappingState() const;
        bool GetObjectDetectionEnabled() const;
        sl::ERROR_CODE GetDetectedAndTrackedObjects() const;
        std::future<sl::FusedPointCloud> ExtractSpatialMapAsync() const;
};
#endif
