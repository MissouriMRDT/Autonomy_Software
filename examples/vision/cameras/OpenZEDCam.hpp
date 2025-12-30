/******************************************************************************
 * @brief Example file that demonstrates opening and using multiple different
 *      features of the ZED camera.
 *
 * @file OpenZEDCam.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-16
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../src/AutonomyConstants.h"
#include "../../../src/AutonomyGlobals.h"
#include "../../../src/AutonomyLogging.h"
#include "../../../src/AutonomyNetworking.h"
#include "../../../src/util/ExampleChecker.h"
#include "../../../src/util/vision/ImageOperations.hpp"

/// \cond
#include <chrono>
/// \endcond

// Declare file constants.
const bool ENABLE_SPATIAL_MAPPING = false;

/******************************************************************************
 * @brief Constructs a printable string out of a given Matrix3f data
 *      structure from the stereolabs library.
 *
 * @param slMatrix - The matrix to print.
 *
 * @author MissouriMRDT (mrdt.autonomy@gmail.com)
 * @date 2025-11-17
 ******************************************************************************/
std::string PrintMatrix3f(const sl::Matrix3f& slMatrix)
{
    std::ostringstream stdOSS;
    stdOSS << std::fixed << std::setprecision(4);

    for (int nIter = 0; nIter < 3; ++nIter)
    {
        stdOSS << "[ ";
        for (int nJter = 0; nJter < 3; ++nJter)
        {
            // Access element at row i, column j
            stdOSS << slMatrix.r[nIter * 3 + nJter];
            if (nJter < 2)
            {
                stdOSS << ", ";
            }
        }
        stdOSS << " ]" << std::endl;
    }

    return stdOSS.str();
}

/******************************************************************************
 * @brief This example demonstrates the proper way to interact with the CameraHandler.
 *      A pointer to a ZEDCam is retrieved and then a couple of local cv::Mat are created
 *      for storing frames. Then, the frames are passed to the RequestFrameCopy function of the
 *      camera and a future is IMMEDIATELY returned. The method call doesn't wait for frame to be
 *      retrieved/copied before returning. This allows you to request multiple frames/data from the
 *      camera non-sequentially.
 *
 *      Inside the camera thread, the cv::Mat pointer that points to the cv::Mat within THIS class
 *      is written to and an std::promise is set to TRUE. The future that was return now contains this
 *      TRUE value. When the get() method is called on the returned future, the code will block until
 *      the promise is fulfilled (set to TRUE). Once the get() method returns, the cv::Mat within
 *      this class now contains a complete frame and can be display or used in other computer vision
 *      things.
 *
 *      The same exact process happens for the positional tracking pose that is retrieved from the camera.
 *      Multiple other methods of the ZEDCam class work this way as it allows this thread and other threads
 *      to get multiple pieces of from the camera without slowing it down to an unusable speed.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-22
 ******************************************************************************/
void RunExample()
{
    // Initialize and start handlers.
    globals::g_pCameraHandler = new CameraHandler();

    // Get pointer to camera.
    std::shared_ptr<ZEDCamera> ExampleZEDCam1 = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam);
    // Start ZED cam.
    ExampleZEDCam1->Start();

    // Turn on ZED features.
    ExampleZEDCam1->EnablePositionalTracking();
    // Check if we should turn on spatial mapping.
    if (ENABLE_SPATIAL_MAPPING)
    {
        // Enable spatial mapping.
        ExampleZEDCam1->EnableSpatialMapping();
    }

    // Declare mats to store images in.
    cv::Mat cvNormalFrame1;
    cv::Mat cvDepthFrame1;
    cv::Mat cvPointCloud1;
    cv::Mat cvPointCloudColor1;
    cv::cuda::GpuMat cvGPUNormalFrame1;
    cv::cuda::GpuMat cvGPUDepthFrame1;
    cv::cuda::GpuMat cvGPUPointCloud1;
    // Declare other data types to store data in.
    ZEDCam::Pose stPose;
    sl::SensorsData slSensors;

    // Declare FPS counter.
    IPS FPS = IPS();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Create instance variables.
        std::future<bool> fuFrameCopyStatus;
        std::future<bool> fuDepthCopyStatus;
        std::future<bool> fuPointCloudCopyStatus;

        // Check if the camera is setup to use CPU or GPU mats.
        if (ExampleZEDCam1->GetUsingGPUMem())
        {
            // Grab frames from camera.
            fuFrameCopyStatus      = ExampleZEDCam1->RequestFrameCopy(cvGPUNormalFrame1);
            fuDepthCopyStatus      = ExampleZEDCam1->RequestDepthCopy(cvGPUDepthFrame1, false);
            fuPointCloudCopyStatus = ExampleZEDCam1->RequestPointCloudCopy(cvGPUPointCloud1);
        }
        else
        {
            // Grab frames from camera.
            fuFrameCopyStatus      = ExampleZEDCam1->RequestFrameCopy(cvNormalFrame1);
            fuDepthCopyStatus      = ExampleZEDCam1->RequestDepthCopy(cvDepthFrame1, false);
            fuPointCloudCopyStatus = ExampleZEDCam1->RequestPointCloudCopy(cvPointCloud1);
        }
        // Grab other info from camera.
        std::future<bool> fuPoseCopyStatus    = ExampleZEDCam1->RequestPositionalPoseCopy(stPose);
        std::future<bool> fuSensorsCopyStatus = ExampleZEDCam1->RequestSensorsCopy(slSensors);

        // Wait for the frames to be copied.
        if (fuFrameCopyStatus.get() && fuDepthCopyStatus.get() && fuPointCloudCopyStatus.get())
        {
            // Check if the camera is setup to use CPU or GPU mats.
            if (ExampleZEDCam1->GetUsingGPUMem())
            {
                // Download memory from gpu mats if necessary.
                cvGPUNormalFrame1.download(cvNormalFrame1);
                cvGPUDepthFrame1.download(cvDepthFrame1);
                cvGPUPointCloud1.download(cvPointCloud1);
            }

            // Put FPS on normal frame.
            cv::putText(cvNormalFrame1,
                        std::to_string(ExampleZEDCam1->GetIPS().GetExactIPS()),
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,
                        1,
                        cv::Scalar(255, 255, 255));

            // Put FPS on depth frame.
            cv::putText(cvDepthFrame1, std::to_string(ExampleZEDCam1->GetIPS().GetExactIPS()), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));

            // Split color from point cloud.
            imgops::SplitPointCloudColors(cvPointCloud1, cvPointCloudColor1);

            // Wait for the other info to be copied.
            if (fuPoseCopyStatus.get())
            {
                LOG_INFO(logging::g_qConsoleLogger,
                         "Positional Tracking: X: {} | Y: {} | Z: {}",
                         stPose.stTranslation.dX,
                         stPose.stTranslation.dY,
                         stPose.stTranslation.dZ);
                LOG_INFO(logging::g_qConsoleLogger,
                         "Positional Orientation: Roll: {} | Pitch: {} | Yaw:{}",
                         stPose.stEulerAngles.dXO,
                         stPose.stEulerAngles.dYO,
                         stPose.stEulerAngles.dZO);
            }

            // Wait for sensors data to be copied.
            if (fuSensorsCopyStatus.get())
            {
                // Unpack sensors data.
                double dRelativeAltitude = slSensors.barometer.relative_altitude;
                float fTemperature       = 0.0;
                slSensors.temperature.get(sl::SensorsData::TemperatureData::SENSOR_LOCATION::IMU, fTemperature);
                float fMagHeading             = slSensors.magnetometer.magnetic_heading;
                float fIMUVelocityX           = slSensors.imu.angular_velocity.x;
                float fIMUVelocityY           = slSensors.imu.angular_velocity.y;
                float fIMUVelocityZ           = slSensors.imu.angular_velocity.z;
                sl::Matrix3f slIMUVelocityCov = slSensors.imu.angular_velocity_covariance.r;
                float fAccelX                 = slSensors.imu.linear_acceleration.x;
                float fAccelY                 = slSensors.imu.linear_acceleration.y;
                float fAccelZ                 = slSensors.imu.linear_acceleration.z;
                sl::Matrix3f slAccelCov       = slSensors.imu.angular_velocity_covariance.r;

                LOG_INFO(logging::g_qConsoleLogger,
                         "Sensors Data: Altitude: {} | Temperature: {} | Mag Heading: {}\nIMU VelX: {}\nIMU VelY: {}\nIMU VelZ: {}\nIMU AccelX: {}\nIMU AccelY: {}\n"
                         "IMU AccelZ: {}\nIMU VelCov: {}\nIMU AccelCov: {}",
                         dRelativeAltitude,
                         fTemperature,
                         fMagHeading,
                         fIMUVelocityX,
                         fIMUVelocityY,
                         fIMUVelocityZ,
                         fAccelX,
                         fAccelY,
                         fAccelZ,
                         PrintMatrix3f(slIMUVelocityCov),
                         PrintMatrix3f(slAccelCov));
            }

            // Print info.
            LOG_INFO(logging::g_qConsoleLogger, "ZED Getter FPS: {} | 1% Low: {}", ExampleZEDCam1->GetIPS().GetAverageIPS(), ExampleZEDCam1->GetIPS().Get1PercentLow());
            // Check if spatial mapping is enabled.
            if (ENABLE_SPATIAL_MAPPING)
            {
                LOG_INFO(logging::g_qConsoleLogger, "Spatial Mapping State: {}", sl::toString(ExampleZEDCam1->GetSpatialMappingState()).get());
            }

            // Display frames.
            cv::imshow("FRAME1", cvNormalFrame1);
            cv::imshow("DEPTH1", cvDepthFrame1);
            cv::imshow("POINT CLOUD COLOR 1", cvPointCloudColor1);
        }

        // Tick FPS counter.
        FPS.Tick();
        // Print FPS of main loop.
        LOG_INFO(logging::g_qConsoleLogger, "Main FPS: {}", FPS.GetAverageIPS());

        char chKey = cv::waitKey(1);
        if (chKey == 27)    // Press 'Esc' key to exit
            break;
    }

    // Close all OpenCV windows.
    cv::destroyAllWindows();

    // Check if spatial mapping is enabled.
    if (ENABLE_SPATIAL_MAPPING)
    {
        // Extract spatial map.
        std::future<sl::Mesh> fuSpatialMap;
        ExampleZEDCam1->ExtractSpatialMapAsync(fuSpatialMap);
        sl::Mesh slSpatialMap = fuSpatialMap.get();
        slSpatialMap.save("test.obj", sl::MESH_FILE_FORMAT::PLY);
    }

    /////////////////////////////////////////
    // Cleanup.
    /////////////////////////////////////////
    // Stop RoveComm quill logging or quill will segfault if trying to output logs to RoveComm.
    network::g_bRoveCommUDPStatus = false;
    network::g_bRoveCommTCPStatus = false;

    // Stop camera threads.
    globals::g_pCameraHandler->StopAllCameras();
}
