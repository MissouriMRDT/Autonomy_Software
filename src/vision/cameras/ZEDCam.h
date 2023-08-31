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

#include "../../AutonomyConstants.h"
#include "../../interfaces/Camera.hpp"

class ZEDCam : public Camera<sl::Mat>
{
    private:
        // Declare private methods and functions variables.
        sl::Camera m_slCamera;
        sl::InitParameters m_slCameraParams;
        sl::RuntimeParameters m_slRuntimeParams;
        sl::SensorsData m_slSensorData;
        sl::PositionalTrackingParameters m_slPoseTrackingParams;
        sl::Pose m_slCameraPose;
        sl::SpatialMappingParameters m_slSpatialMappingParams;
        sl::ObjectDetectionParameters m_slObjectDetectionParams;
        sl::BatchParameters m_slObjectDetectionBatchParams;
        sl::Objects m_slDetectedObjects;
        sl::MEM m_slMemoryType;
        sl::Mat m_slFrame;
        sl::Mat m_slDepth;
        sl::Mat m_slPointCloud;

        // Create iteration per second object pointers.
        IPS* m_pIPSDepth      = new IPS();
        IPS* m_pIPSPointCloud = new IPS();

    public:
        // Declare public structs that are specific to and used within this class.
        struct ZedObjectData;

        // Define public enum for IPS selection.
        enum IPS_TYPE
        {
            eFRAME,
            eDEPTH,
            ePOINTCLOUD
        };

        // Declare public methods and member variables.
        ZEDCam(const int nPropResolutionX,
               const int nPropResolutionY,
               const int nPropFramesPerSecond,
               const double dPropHorizontalFOV,
               const double dPropVerticalFOV,
               const float fMinSenseDistance           = constants::ZED_DEFAULT_MINIMUM_DISTANCE,
               const float fMaxSenseDistance           = constants::ZED_DEFAULT_MAXIMUM_DISTANCE,
               const bool bMemTypeGPU                  = false,
               const unsigned int unCameraSerialNumber = 0);
        ~ZEDCam();
        sl::Mat GrabFrame(const bool bResize = true) override;
        sl::Mat GrabDepth(const bool bRetrieveMeasure, const bool bResize = true, const bool bHalfPrecision = false);
        sl::Mat GrabPointCloud(const bool bResize = true, const bool bIncludeColor = false);
        sl::ERROR_CODE ResetPositionalTracking();
        sl::ERROR_CODE TrackCustomBoxObjects(std::vector<ZedObjectData>& vCustomObjects);
        sl::ERROR_CODE RebootCamera();

        // Setters for class member variables.
        sl::ERROR_CODE EnablePositionalTracking();
        void DisablePositionalTracking();
        sl::ERROR_CODE SetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO);
        sl::ERROR_CODE EnableSpatialMapping();
        void DisableSpatialMapping();
        sl::ERROR_CODE EnableObjectDetection(const bool bEnableBatching = false);
        void DisableObjectDetection();

        // Accessors for class member variables.
        bool GetCameraIsOpen() override;
        IPS* GetIPS(const IPS_TYPE eIPSType);
        std::string GetCameraModel();
        unsigned int GetCameraSerial();
        sl::Pose GetPositionalPose(const sl::REFERENCE_FRAME slPositionReference = sl::REFERENCE_FRAME::WORLD);
        bool GetPositionalTrackingEnabled();
        std::vector<double> GetIMUData();
        sl::SPATIAL_MAPPING_STATE GetSpatialMappingState();
        sl::SPATIAL_MAPPING_STATE ExtractSpatialMapAsync(std::future<sl::FusedPointCloud>& fuPointCoudFuture);
        bool GetObjectDetectionEnabled();
        std::vector<sl::ObjectData> GetObjects();
        std::vector<sl::ObjectsBatch> GetBatchedObjects();
};
#endif
