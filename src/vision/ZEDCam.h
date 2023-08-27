/******************************************************************************
 * @brief Defines the ZEDCam class.
 *
 * @file ZEDCam.h
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

class ZEDCam : public Camera<sl::Mat>
{
    private:
        // Declare private methods and functions variables.
        sl::Camera m_slCamera;
        sl::Pose m_slCameraPose;
        sl::SensorsData m_slSensorData;
        sl::Objects m_slTrackedObjects;
        sl::ObjectDetectionParameters m_slTrackedObjectParameters;
        sl::Mat m_slFrame;
        sl::Mat m_slDepth;
        sl::Mat m_slPointCloud;

    public:
        // Declare public methods and member variables.
        ZEDCam(const int nPropResolutionX, const int nPropResolutionY, const int nPropFramesPerSecond, const double dPropHorizontalFOV, const double dPropVerticalFOV);
        ~ZEDCam();
        sl::Mat GrabFrame(const bool bGrabRaw = false) override;
        sl::Mat GrabDepth(const bool bGrabRaw = false);
        sl::Mat GrabPointCloud(const bool bGrabRaw = false);
        sl::ERROR_CODE ResetPositionalTracking();
        sl::ERROR_CODE TrackCustomBoxObjects(std::vector<sl::CustomBoxObjectData> vCustomObjects);
        sl::ERROR_CODE RebootCamera();

        // Setters for class member variables.
        sl::ERROR_CODE EnablePositionalTracking();
        void DisablePositionalTracking();
        sl::ERROR_CODE SetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO);
        sl::ERROR_CODE EnableSpatialMapping();
        void DisableSpatialMapping();
        sl::ERROR_CODE EnableObjectTracking();
        void DisableObjectTracking();

        // Accessors for class member variables.
        bool GetCameraIsOpen() override;
        sl::Pose GetPositionalPose(const sl::REFERENCE_FRAME slPositionReference = sl::REFERENCE_FRAME::WORLD);
        bool GetPositionalTrackingEnabled();
        std::vector<double> GetIMUData();
        sl::SPATIAL_MAPPING_STATE GetSpatialMappingState();
        std::future<sl::FusedPointCloud> ExtractSpatialMapAsync();
        bool GetObjectTrackingEnabled();
        std::vector<sl::ObjectData> GetTrackedObjects();
};
#endif
