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
 * @brief This example demonstrates the proper way to consume data from a ZEDCam.
 *
 *      The ZED exposes one publish-latest channel per data type: BGRA frame, depth
 *      measure, depth image, point cloud, pose, floor plane, sensors, detected objects,
 *      batched objects, and camera status. Each has both a CPU (cv::Mat) and a GPU
 *      (cv::cuda::GpuMat) variant where it makes sense; use the one matching the memory
 *      mode the camera was configured with (GetUsingGPUMem()).
 *
 *      Consuming any of them is the same two steps:
 *
 *      1. Take a Reader once and hold it. The camera calls into the
 *         ZED SDK for a data type ONLY while something is subscribed to it, so this is
 *         what turns retrieval on. This is a real performance lever: never subscribe to
 *         the point cloud if you only need the frame.
 *
 *      2. Get() the newest snapshot. This never blocks on the camera loop, so this loop
 *         and the camera's run at completely independent rates. It returns nullptr until
 *         the first publish, and after a failed grab the last good snapshot stays valid
 *         (compare ullSequence / tmPublished to detect staleness).
 *
 *      Load each snapshot ONCE into a local and work from that local. Snapshots are
 *      immutable and shared with every other consumer, so clone before modifying.
 *
 *      Note the ordering of the feature-enable calls below: configuring the camera BEFORE
 *      Start() runs those commands inline on this thread, which is both simpler and
 *      faster. Calling them after Start() also works, but they are then posted to the
 *      camera thread and this thread blocks until it drains them (up to one frame period).
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

    // Turn on ZED features BEFORE starting the camera thread so these run inline instead of
    // being posted to the camera thread and waited on.
    ExampleZEDCam1->EnablePositionalTracking();
    // Check if we should turn on spatial mapping.
    if (ENABLE_SPATIAL_MAPPING)
    {
        // Enable spatial mapping.
        ExampleZEDCam1->EnableSpatialMapping();
    }

    // Start ZED cam.
    ExampleZEDCam1->Start();

    // Whether this camera hands out GPU or CPU mats. Decides which channels we subscribe to.
    const bool bUsingGPUMem = ExampleZEDCam1->GetUsingGPUMem();

    // Register demand for every data type we intend to read. The camera retrieves NOTHING for a
    // channel with no subscribers, so these handles are what actually turn each retrieval on.
    // Hold them for as long as we want the data; letting them fall out of scope withdraws demand.
    // Only the handle matching the camera's memory mode is taken; the other stays
    // default constructed and inactive.
    pubsub::Reader<cv::Mat> rdFrameCPU;
    pubsub::Reader<cv::cuda::GpuMat> rdFrameGPU;
    if (bUsingGPUMem)
    {
        // Take the GPU channel handle.
        rdFrameGPU = ExampleZEDCam1->GetFrameGPUReader();
    }
    else
    {
        // Take the CPU channel handle.
        rdFrameCPU = ExampleZEDCam1->GetFrameCPUReader();
    }
    // Only the handle matching the camera's memory mode is taken; the other stays
    // default constructed and inactive.
    pubsub::Reader<cv::Mat> rdDepthImageCPU;
    pubsub::Reader<cv::cuda::GpuMat> rdDepthImageGPU;
    if (bUsingGPUMem)
    {
        // Take the GPU channel handle.
        rdDepthImageGPU = ExampleZEDCam1->GetDepthImageGPUReader();
    }
    else
    {
        // Take the CPU channel handle.
        rdDepthImageCPU = ExampleZEDCam1->GetDepthImageCPUReader();
    }
    // Only the handle matching the camera's memory mode is taken; the other stays
    // default constructed and inactive.
    pubsub::Reader<cv::Mat> rdPointCloudCPU;
    pubsub::Reader<cv::cuda::GpuMat> rdPointCloudGPU;
    if (bUsingGPUMem)
    {
        // Take the GPU channel handle.
        rdPointCloudGPU = ExampleZEDCam1->GetPointCloudGPUReader();
    }
    else
    {
        // Take the CPU channel handle.
        rdPointCloudCPU = ExampleZEDCam1->GetPointCloudCPUReader();
    }
    pubsub::Reader<ZEDCamera::Pose> rdPose       = ExampleZEDCam1->GetPoseReader();
    pubsub::Reader<sl::SensorsData> rdSensors    = ExampleZEDCam1->GetSensorsReader();

    // Declare mats to store our own working copies in.
    cv::Mat cvNormalFrame1;
    cv::Mat cvDepthFrame1;
    cv::Mat cvPointCloud1;
    cv::Mat cvPointCloudColor1;

    // Track which frame we last processed so we can skip iterations with nothing new.
    unsigned long long ullLastProcessedSequence = 0;

    // Declare FPS counter.
    IPS FPS = IPS();

    // Loop forever, or until user hits ESC.
    while (true)
    {
        // Load the newest snapshot of each channel ONCE into a local. These are lock-free,
        // non-blocking reads; a null result simply means nothing has been published yet.
        bool bHaveNewImagery                        = false;
        unsigned long long ullThisFrameSequence     = 0;

        // Check if the camera is setup to use CPU or GPU mats.
        if (bUsingGPUMem)
        {
            // Load the GPU snapshots.
            pubsub::SharedSnapshot<cv::cuda::GpuMat> pFrame      = rdFrameGPU.Get();
            pubsub::SharedSnapshot<cv::cuda::GpuMat> pDepth      = rdDepthImageGPU.Get();
            pubsub::SharedSnapshot<cv::cuda::GpuMat> pPointCloud = rdPointCloudGPU.Get();

            // Only process once every channel has produced something new.
            if (pFrame != nullptr && pDepth != nullptr && pPointCloud != nullptr && pFrame->ullSequence != ullLastProcessedSequence)
            {
                // Record which frame this is and that we have work to do.
                ullThisFrameSequence = pFrame->ullSequence;
                bHaveNewImagery      = true;
                // Download from GPU memory onto our own mats. Done here, on this thread, off the
                // camera's critical path.
                pFrame->tData.download(cvNormalFrame1);
                pDepth->tData.download(cvDepthFrame1);
                pPointCloud->tData.download(cvPointCloud1);
            }
        }
        else
        {
            // Load the CPU snapshots.
            pubsub::SharedSnapshot<cv::Mat> pFrame      = rdFrameCPU.Get();
            pubsub::SharedSnapshot<cv::Mat> pDepth      = rdDepthImageCPU.Get();
            pubsub::SharedSnapshot<cv::Mat> pPointCloud = rdPointCloudCPU.Get();

            // Only process once every channel has produced something new.
            if (pFrame != nullptr && pDepth != nullptr && pPointCloud != nullptr && pFrame->ullSequence != ullLastProcessedSequence)
            {
                // Record which frame this is and that we have work to do.
                ullThisFrameSequence = pFrame->ullSequence;
                bHaveNewImagery      = true;
                // Snapshots are immutable and shared, and we draw on these below, so take our own
                // deep copies rather than aliasing the published buffers.
                pFrame->tData.copyTo(cvNormalFrame1);
                pDepth->tData.copyTo(cvDepthFrame1);
                pPointCloud->tData.copyTo(cvPointCloud1);
            }
        }

        // Only do the display work when the camera actually produced a new frame.
        if (bHaveNewImagery)
        {
            // Remember which frame we processed so a repeat of it is skipped next time.
            ullLastProcessedSequence = ullThisFrameSequence;

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

            // Read the newest pose snapshot. Null until positional tracking has published one.
            pubsub::SharedSnapshot<ZEDCamera::Pose> pPose = rdPose.Get();
            if (pPose != nullptr)
            {
                // Work from the immutable snapshot's data.
                const ZEDCamera::Pose& stPose = pPose->tData;
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

            // Read the newest sensors snapshot. Null until the camera has published one.
            pubsub::SharedSnapshot<sl::SensorsData> pSensors = rdSensors.Get();
            if (pSensors != nullptr)
            {
                // Copy the sensors data out of the snapshot. A const reference will not work here:
                // several ZED SDK accessors (temperature.get(), the covariance members) are not
                // const qualified. Copying also guarantees we never mutate shared snapshot data.
                sl::SensorsData slSensors = pSensors->tData;
                double dRelativeAltitude  = slSensors.barometer.relative_altitude;
                float fTemperature        = 0.0;
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
    // Withdraw all of our demand so the camera stops retrieving data nobody is reading. This also
    // happens automatically when these handles go out of scope.
    rdFrameCPU.Release();
    rdFrameGPU.Release();
    rdDepthImageCPU.Release();
    rdDepthImageGPU.Release();
    rdPointCloudCPU.Release();
    rdPointCloudGPU.Release();
    rdPose.Release();
    rdSensors.Release();

    // Stop RoveComm quill logging or quill will segfault if trying to output logs to RoveComm.
    network::g_bRoveCommUDPStatus = false;
    network::g_bRoveCommTCPStatus = false;

    // Stop camera threads.
    globals::g_pCameraHandler->StopAllCameras();
}
