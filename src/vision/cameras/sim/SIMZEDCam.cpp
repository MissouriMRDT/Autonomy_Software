/******************************************************************************
 * @brief Implements the SIMZEDCam class.
 *
 * @file SIMZEDCam.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "SIMZEDCam.h"

#include "../../../AutonomyConstants.h"
#include "../../../AutonomyGlobals.h"
#include "../../../AutonomyLogging.h"

/// \cond
#include "../../../util/NumberOperations.hpp"
#include <nlohmann/json.hpp>

/// \endcond

/******************************************************************************
 * @brief Construct a new SIM Cam:: SIM Cam object.
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
 * @param unCameraSerialNumber - The serial number of the camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
SIMZEDCam::SIMZEDCam(const std::string szCameraPath,
                     const int nPropResolutionX,
                     const int nPropResolutionY,
                     const int nPropFramesPerSecond,
                     const double dPropHorizontalFOV,
                     const double dPropVerticalFOV,
                     const bool bEnableRecordingFlag,
                     const int nNumFrameRetrievalThreads,
                     const unsigned int unCameraSerialNumber) :
    ZEDCamera(nPropResolutionX,
              nPropResolutionY,
              nPropFramesPerSecond,
              dPropHorizontalFOV,
              dPropVerticalFOV,
              bEnableRecordingFlag,
              false,
              false,
              false,
              nNumFrameRetrievalThreads,
              unCameraSerialNumber)
{
    // Assign member variables.
    m_szCameraPath              = szCameraPath;
    m_nNumFrameRetrievalThreads = nNumFrameRetrievalThreads;

    // Initialize OpenCV mats to a black/empty image the size of the camera resolution.
    m_cvFrame        = cv::Mat::zeros(nPropResolutionY, nPropResolutionX, CV_8UC4);
    m_cvDepthImage   = cv::Mat::zeros(nPropResolutionY, nPropResolutionX, CV_8UC3);
    m_cvDepthMeasure = cv::Mat::zeros(nPropResolutionY, nPropResolutionX, CV_32FC1);
    m_cvDepthBuffer  = cv::Mat::zeros(nPropResolutionY, nPropResolutionX, CV_8UC3);
    m_cvPointCloud   = cv::Mat::zeros(nPropResolutionY, nPropResolutionX, CV_32FC4);

    // Construct camera stream objects. Append proper camera path arguments to each URL camera path.
    m_pRGBStream        = std::make_unique<WebRTC>(szCameraPath, "ZEDFrontRGB");
    m_pDepthImageStream = std::make_unique<WebRTC>(szCameraPath, "ZEDFrontDepthImage");
    // m_pDepthMeasureStream = std::make_unique<WebRTC>(szCameraPath, "ZEDFrontDepthMeasure");

    // Set callbacks for the WebRTC connections.
    this->SetCallbacks();

    // Set max FPS of the ThreadedContinuousCode method.
    this->SetMainThreadIPSLimit(nPropFramesPerSecond);
}

/******************************************************************************
 * @brief Destroy the SIM Cam:: SIM Cam object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
SIMZEDCam::~SIMZEDCam()
{
    // Destroy the WebRTC connections.
    m_pRGBStream.reset();
    m_pDepthImageStream.reset();
    m_pDepthMeasureStream.reset();

    // Stop threaded code.
    this->RequestStop();
    this->Join();
}

/******************************************************************************
 * @brief This method sets the callbacks for the WebRTC connections.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
void SIMZEDCam::SetCallbacks()
{
    // Set the frame callbacks.
    m_pRGBStream->SetOnFrameReceivedCallback(
        [this](cv::Mat& cvFrame)
        {
            // Acquire a lock on the webRTC copy mutex.
            std::unique_lock<std::shared_mutex> lkWebRTC(m_muWebRTCRGBImageCopyMutex);
            // Deep copy the frame.
            m_cvFrame = cvFrame.clone();
        });
    m_pDepthImageStream->SetOnFrameReceivedCallback(
        [this](cv::Mat& cvFrame)
        {
            // Acquire a lock on the webRTC copy mutex.
            std::unique_lock<std::shared_mutex> lkWebRTC(m_muWebRTCDepthImageCopyMutex);
            // Deep copy the frame.
            m_cvDepthImage = cvFrame.clone();
        });
    // m_pDepthMeasureStream->SetOnFrameReceivedCallback(
    //     [this](cv::Mat& cvFrame)
    //     {
    //         // Acquire a lock on the webRTC copy mutex.
    //         std::unique_lock<std::shared_mutex> lkWebRTC(m_muWebRTCDepthMeasureCopyMutex);
    //         // Deep copy the frame.
    //         m_cvDepthBuffer = cvFrame.clone();
    //     });
}

/******************************************************************************
 * @brief This method decodes the encoded depth measure data from the simulator.
 *      We receive the depth measure from an H264 stream so it has to be encoded
 *      and packed in a special way to ensure things like compression and
 *      transmission are efficient.
 *
 * @param cvDepthBuffer - The encoded depth buffer.
 * @param cvDepthMeasure - The decoded depth measure that will be written to.
 *
 * @note http://reality.cs.ucl.ac.uk/projects/depth-streaming/depth-streaming.pdf
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-04
 ******************************************************************************/
void SIMZEDCam::DecodeDepthMeasure(const cv::Mat& cvDepthBuffer, cv::Mat& cvDepthMeasure)
{
    // Declare instance variables.
    float fW  = 65536.0;
    float fNP = 512.0;

    // Iterate over each pixel in the cvDepthMeasure image
    for (int nY = 0; nY < cvDepthBuffer.rows; ++nY)
    {
        for (int nX = 0; nX < cvDepthBuffer.cols; ++nX)
        {
            // Extract the encoded depth values
            cv::Vec3b cvEncodedDepth = cvDepthBuffer.at<cv::Vec3b>(nY, nX);

            // Extract encoded values
            float fL  = cvEncodedDepth[2] / 255.0;
            float fHa = cvEncodedDepth[1] / 255.0;
            float fHb = cvEncodedDepth[0] / 255.0;

            // Period for triangle waves
            float fP = fNP / fW;

            // Determine offset and fine-grain correction
            int fM       = fmod((4.0 * (fL / fP)) - 0.5, 4.0);
            float fL0    = fL - fmod(fL - (fP / 8.0), fP) + ((fP / 4.0) * fM) - (fP / 8.0);

            float fDelta = 0.0f;
            if (fM == 0)
                fDelta = (fP / 2.0) * fHa;
            else if (fM == 1)
                fDelta = (fP / 2.0) * fHb;
            else if (fM == 2)
                fDelta = (fP / 2.0) * (1.0 - fHa);
            else if (fM == 3)
                fDelta = (fP / 2.0) * (1.0 - fHb);

            // Combine to compute the original depth
            float fDepth = fW * (fL0 + fDelta);

            // Check if the depth is within the bounds of the depth image
            if (fDepth < 0.0)
                fDepth = 0.0;
            else if (fDepth > 65535.0)
                fDepth = 65535.0;

            // Check if nY and nX are within the bounds of the depth image
            if (nY < cvDepthMeasure.rows && nX < cvDepthMeasure.cols)
            {
                // Store the decoded depth in the new cv::Mat. Convert cm to m.
                cvDepthMeasure.at<float>(nY, nX) = static_cast<float>(fDepth / 100.0);
            }
        }
    }
}

/******************************************************************************
 * @brief This method calculates a point cloud from the decoded depth measure
 *      use some simple trig and the camera FOV.
 *
 * @param cvDepthMeasure - The decoded depth measure.
 * @param cvPointCloud - The point cloud that will be written to.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-04
 ******************************************************************************/
void SIMZEDCam::CalculatePointCloud(const cv::Mat& cvDepthMeasure, cv::Mat& cvPointCloud)
{
    // Use the decoded depth measure to create a point cloud.
    for (int nY = 0; nY < cvDepthMeasure.rows; ++nY)
    {
        for (int nX = 0; nX < cvDepthMeasure.cols; ++nX)
        {
            // Get the depth value.
            float fDepth = cvDepthMeasure.at<float>(nY, nX);

            // Get the horizontal and vertical angles.
            double dHorizontalAngle = (nX - cvDepthMeasure.cols / 2.0) * m_dPropHorizontalFOV / cvDepthMeasure.cols;
            double dVerticalAngle   = (nY - cvDepthMeasure.rows / 2.0) * m_dPropVerticalFOV / cvDepthMeasure.rows;

            // Convert angles to radians.
            double dHorizontalAngleRad = dHorizontalAngle * M_PI / 180.0;
            double dVerticalAngleRad   = dVerticalAngle * M_PI / 180.0;

            // Calculate the Cartesian coordinates.
            float fX = fDepth * sin(dHorizontalAngleRad);
            float fY = fDepth * sin(dVerticalAngleRad);
            float fZ = fDepth * cos(dHorizontalAngleRad) * cos(dVerticalAngleRad);

            // Store the decoded depth in the new cv::Mat
            m_cvPointCloud.at<cv::Vec4f>(nY, nX) = cv::Vec4f(fX, fY, fZ, 1.0);
        }
    }
}

/******************************************************************************
 * @brief The code inside this private method runs in a separate thread, but still
 *      has access to this*. This method continuously get new frames from the OpenCV
 *      VideoCapture object and stores it in a member variable. Then a thread pool is
 *      started and joined once per iteration to mass copy the frames and/or measure
 *      to any other thread waiting in the queues.
 *
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
void SIMZEDCam::ThreadedContinuousCode()
{
    // Acquire a lock on the rover pose mutex.
    std::unique_lock<std::shared_mutex> lkRoverPoseLock(m_muCurrentRoverPoseMutex);
    // Check if the NavBoard pointer is valid.
    if (globals::g_pNavigationBoard != nullptr)
    {
        // Get the current rover pose from the NavBoard.
        m_stCurrentRoverPose = geoops::RoverPose(globals::g_pNavigationBoard->GetGPSData(), globals::g_pNavigationBoard->GetHeading());
    }
    {
        // Get the current rover pose from the NavBoard.
        m_stCurrentRoverPose = geoops::RoverPose(globals::g_pNavigationBoard->GetGPSData(), globals::g_pNavigationBoard->GetHeading());
    }
    // Release lock.
    lkRoverPoseLock.unlock();

    // Acquire a lock on the WebRTC mutex.
    std::shared_lock<std::shared_mutex> lkWebRTC3(m_muWebRTCDepthMeasureCopyMutex);
    // Decode the depth measure.
    this->DecodeDepthMeasure(m_cvDepthBuffer, m_cvDepthMeasure);
    // Release lock.
    lkWebRTC3.unlock();

    // Calculate the point cloud from the decoded depth measure.
    this->CalculatePointCloud(m_cvDepthMeasure, m_cvPointCloud);

    // Acquire a shared_lock on the frame copy queue.
    std::shared_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Check if the frame copy queue is empty.
    if (!m_qFrameCopySchedule.empty() || !m_qPoseCopySchedule.empty() || !m_qGeoPoseCopySchedule.empty())
    {
        // Add the length of all queues together to determine the number of tasks to create.
        size_t siTotalQueueLength = m_qFrameCopySchedule.size() + m_qPoseCopySchedule.size() + m_qGeoPoseCopySchedule.size();

        // Acquire shared lock on the WebRTC mutex, so that the WebRTC connection doesn't try to write to the Mats while they are being copied in the thread pool.
        std::shared_lock<std::shared_mutex> lkWebRTC(m_muWebRTCRGBImageCopyMutex);
        std::shared_lock<std::shared_mutex> lkWebRTC2(m_muWebRTCDepthImageCopyMutex);
        std::shared_lock<std::shared_mutex> lkWebRTC3(m_muWebRTCDepthMeasureCopyMutex);

        // Start the thread pool to store multiple copies of the sl::Mat into the given cv::Mats.
        this->RunDetachedPool(siTotalQueueLength, m_nNumFrameRetrievalThreads);

        // Static bool for keeping track of if the thread pool has been started.
        static bool bQueueTogglesAlreadyReset = false;
        // Get current time.
        std::chrono::_V2::system_clock::duration tmCurrentTime = std::chrono::high_resolution_clock::now().time_since_epoch();
        // Only reset once every couple seconds.
        if (std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime).count() % 31 == 0 && !bQueueTogglesAlreadyReset)
        {
            // Reset queue counters.
            m_bPosesQueued.store(false, ATOMIC_MEMORY_ORDER_METHOD);
            m_bGeoPosesQueued.store(false, ATOMIC_MEMORY_ORDER_METHOD);

            // Set reset toggle.
            bQueueTogglesAlreadyReset = true;
        }
        // Crucial for toggle action. If time is not evenly devisable and toggles have previously been set, reset queue reset boolean.
        else if (bQueueTogglesAlreadyReset)
        {
            // Reset reset toggle.
            bQueueTogglesAlreadyReset = false;
        }

        // Wait for thread pool to finish.
        this->JoinPool();
    }

    // Release lock on frame copy queue.
    lkSchedulers.unlock();
}

/******************************************************************************
 * @brief This method holds the code that is ran in the thread pool started by
 *      the ThreadedLinearCode() method. It copies the data from the different
 *      data objects to references of the same type stored in a vector queued up by the
 *      Grab methods.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
void SIMZEDCam::PooledLinearCode()
{
    /////////////////////////////
    //  Frame queue.
    /////////////////////////////

    // Acquire mutex for getting frames out of the queue.
    std::unique_lock<std::shared_mutex> lkFrameQueue(m_muFrameCopyMutex);
    // Check if the queue is empty.
    if (!m_qFrameCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::FrameFetchContainer<cv::Mat> stContainer = m_qFrameCopySchedule.front();
        // Pop out of queue.
        m_qFrameCopySchedule.pop();
        // Release lock.
        lkFrameQueue.unlock();

        // Determine which frame should be copied.
        switch (stContainer.eFrameType)
        {
            case PIXEL_FORMATS::eBGRA: *(stContainer.pFrame) = m_cvFrame.clone(); break;
            case PIXEL_FORMATS::eDepthImage: *(stContainer.pFrame) = m_cvDepthImage.clone(); break;
            case PIXEL_FORMATS::eDepthMeasure: *(stContainer.pFrame) = m_cvDepthMeasure.clone(); break;
            case PIXEL_FORMATS::eXYZ: *(stContainer.pFrame) = m_cvPointCloud.clone(); break;
            default: *(stContainer.pFrame) = m_cvFrame.clone(); break;
        }

        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedFrameStatus->set_value(true);
    }

    /////////////////////////////
    //  Pose queue.
    /////////////////////////////
    // Acquire mutex for getting data out of the pose queue.
    std::unique_lock<std::shared_mutex> lkPoseQueue(m_muPoseCopyMutex);
    // Check if the queue is empty.
    if (!m_qPoseCopySchedule.empty())
    {
        // Get pose container out of queue.
        containers::DataFetchContainer<Pose> stContainer = m_qPoseCopySchedule.front();
        // Pop out of queue.
        m_qPoseCopySchedule.pop();
        // Release lock.
        lkPoseQueue.unlock();

        // Get angle realignments.
        double dNewYO = numops::InputAngleModulus<double>(m_stCurrentRoverPose.GetCompassHeading() + m_dPoseOffsetYO, 0.0, 360.0);
        // Repack values into pose.
        Pose stPose(m_stCurrentRoverPose.GetUTMCoordinate().dEasting + m_dPoseOffsetX,
                    m_stCurrentRoverPose.GetUTMCoordinate().dAltitude + m_dPoseOffsetY,
                    m_stCurrentRoverPose.GetUTMCoordinate().dNorthing + m_dPoseOffsetZ,
                    m_dPoseOffsetXO,
                    dNewYO,
                    m_dPoseOffsetZO);

        // ISSUE NOTE: Might be in the future if we ever change our coordinate system on the ZED. This can be used to fix the directions of the Pose's coordinate system.
        // // Check ZED coordinate system.
        // switch (m_slCameraParams.coordinate_system)
        // {
        //     case sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP:
        //     {
        //         // Realign based in the signedness of this coordinate system. Z is backwards.
        //         stPose.stTranslation.dZ *= -1;
        //         break;
        //     }
        //     default:
        //     {
        //         // No need to flip signs for other coordinate systems.
        //         break;
        //     }
        // }

        // Copy pose.
        *(stContainer.pData) = stPose;

        // Signal future that the data has been successfully retrieved.
        stContainer.pCopiedDataStatus->set_value(true);
    }
    else
    {
        // Release lock.
        lkPoseQueue.unlock();
    }

    /////////////////////////////
    //  GeoPose queue.
    /////////////////////////////
    // Acquire mutex for getting data out of the pose queue.
    std::unique_lock<std::shared_mutex> lkGeoPoseQueue(m_muGeoPoseCopyMutex);
    // Check if the queue is empty.
    if (!m_qGeoPoseCopySchedule.empty())
    {
        // Get pose container out of queue.
        containers::DataFetchContainer<sl::GeoPose> stContainer = m_qGeoPoseCopySchedule.front();
        // Pop out of queue.
        m_qGeoPoseCopySchedule.pop();
        // Release lock.
        lkGeoPoseQueue.unlock();

        // Get rover's current UTM position.
        geoops::UTMCoordinate stCurrentUTM = m_stCurrentRoverPose.GetUTMCoordinate();
        // Add offsets to the current UTM position.
        stCurrentUTM.dEasting += m_dPoseOffsetX;
        stCurrentUTM.dAltitude += m_dPoseOffsetY;
        stCurrentUTM.dNorthing += m_dPoseOffsetZ;
        // Create a GPS coordinate from the UTM position.
        geoops::GPSCoordinate stCurrentGPS = geoops::ConvertUTMToGPS(stCurrentUTM);

        // Create new GeoPose.
        sl::GeoPose slGeoPose;
        slGeoPose.heading = numops::InputAngleModulus<double>(m_stCurrentRoverPose.GetCompassHeading() + m_dPoseOffsetYO, 0.0, 360.0);
        slGeoPose.latlng_coordinates.setCoordinates(stCurrentGPS.dLatitude, stCurrentGPS.dLongitude, stCurrentGPS.dAltitude, false);

        // Copy pose.
        *(stContainer.pData) = slGeoPose;

        // Signal future that the data has been successfully retrieved.
        stContainer.pCopiedDataStatus->set_value(true);
    }
    else
    {
        // Release lock.
        lkGeoPoseQueue.unlock();
    }
}

/******************************************************************************
 * @brief Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
 *      Remember, this code will be ran in whatever, class/thread calls it.
 *
 * @param cvFrame - A reference to the cv::Mat to store the frame in.
 * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
std::future<bool> SIMZEDCam::RequestFrameCopy(cv::Mat& cvFrame)
{
    // Assemble the FrameFetchContainer.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvFrame, m_ePropPixelFormat);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Requests a depth measure or image from the camera.
 *      Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
 *      This image has the same shape as a grayscale image, but the values represent the depth in
 *      MILLIMETERS. The ZEDSDK will always return this measure in MILLIMETERS.
 *
 * @param cvDepth - A reference to the cv::Mat to copy the depth frame to.
 * @param bRetrieveMeasure - False to get depth IMAGE instead of MEASURE. Do not use the 8-bit grayscale depth image
 *                  purposes other than displaying depth.
 * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
std::future<bool> SIMZEDCam::RequestDepthCopy(cv::Mat& cvDepth, const bool bRetrieveMeasure)
{
    // Create instance variables.
    PIXEL_FORMATS eFrameType;

    // Check if the container should be set to retrieve an image or a measure.
    bRetrieveMeasure ? eFrameType = PIXEL_FORMATS::eDepthMeasure : eFrameType = PIXEL_FORMATS::eDepthImage;
    // Assemble container.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvDepth, eFrameType);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkSchedulers.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Requests a point cloud image from the camera. This image has the same resolution as a normal
 *      image but with three XYZ values replacing the old color values in the 3rd dimension.
 *      The units and sign of the XYZ values are determined by ZED_MEASURE_UNITS and ZED_COORD_SYSTEM
 *      constants set in AutonomyConstants.h.
 *
 *      Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
 *
 * @param cvPointCloud - A reference to the cv::Mat to copy the point cloud frame to.
 * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-08-26
 ******************************************************************************/
std::future<bool> SIMZEDCam::RequestPointCloudCopy(cv::Mat& cvPointCloud)
{
    // Assemble the FrameFetchContainer.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvPointCloud, PIXEL_FORMATS::eXYZ);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkSchedulers.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief This method is used to reset the positional tracking of the camera.
 *      Because this is a simulation camera, and then ZEDSDK is not available,
 *      this method will just reset the offsets to zero.
 *
 * @return sl::ERROR_CODE - The error code returned by the ZED SDK. In this case, it will always be SUCCESS.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
sl::ERROR_CODE SIMZEDCam::ResetPositionalTracking()
{
    // Reset offsets back to zero.
    m_dPoseOffsetX  = 0.0;
    m_dPoseOffsetY  = 0.0;
    m_dPoseOffsetZ  = 0.0;
    m_dPoseOffsetXO = 0.0;
    m_dPoseOffsetYO = 0.0;
    m_dPoseOffsetZO = 0.0;

    return sl::ERROR_CODE::SUCCESS;
}

/******************************************************************************
 * @brief This method is used to reboot the camera. This method will stop the camera thread,
 *      join the camera thread, destroy the camera stream objects, reconstruct the camera stream
 *      objects, set the frame callbacks, and restart the camera thread. This simulates a camera
 *      reboot since the ZED SDK is not available.
 *
 * @return sl::ERROR_CODE - The error code returned by the ZED SDK. In this case, it will always be SUCCESS.
 *              Even if the streams are not successfully reconnected, the camera will still be considered open.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
sl::ERROR_CODE SIMZEDCam::RebootCamera()
{
    // Stop the camera thread.
    this->RequestStop();
    // Join the camera thread.
    this->Join();

    // Destroy the camera stream objects.
    m_pRGBStream.reset();
    m_pDepthImageStream.reset();
    m_pDepthMeasureStream.reset();
    m_pPointCloudStream.reset();
    // Reconstruct camera stream objects. Append proper camera path arguments to each URL camera path.
    m_pRGBStream          = std::make_unique<WebRTC>(m_szCameraPath, "ZEDFrontRGB");
    m_pDepthImageStream   = std::make_unique<WebRTC>(m_szCameraPath, "ZEDFrontDepthImage");
    m_pDepthMeasureStream = std::make_unique<WebRTC>(m_szCameraPath, "ZEDFrontDepthMeasure");
    m_pPointCloudStream   = std::make_unique<WebRTC>(m_szCameraPath, "ZEDFrontPointCloud");

    // Set the frame callbacks.
    this->SetCallbacks();

    // Restart the camera thread.
    this->Start();

    return sl::ERROR_CODE::SUCCESS;
}

/******************************************************************************
 * @brief This method is used to subscribe the sl::Fusion object to the camera with the given UUID.
 *      Given that this is a simulation camera, and the ZED SDK is not available, this method will
 *      do nothing.
 *
 * @param slCameraUUID - The UUID of the camera to subscribe to.
 * @return sl::FUSION_ERROR_CODE - The error code returned by the ZED SDK. In this case, it will always be SUCCESS.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
sl::FUSION_ERROR_CODE SIMZEDCam::SubscribeFusionToCameraUUID(sl::CameraIdentifier& slCameraUUID)
{
    // Unused parameter.
    (void) slCameraUUID;

    return sl::FUSION_ERROR_CODE::SUCCESS;
}

/******************************************************************************
 * @brief This method is used to publish the camera to the sl::Fusion object.
 *      Since this is a simulation camera this will do nothing.
 *
 * @return sl::CameraIdentifier - The identifier of the camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
sl::CameraIdentifier SIMZEDCam::PublishCameraToFusion()
{
    return sl::CameraIdentifier(m_unCameraSerialNumber);
}

/******************************************************************************
 * @brief This method is used to enable positional tracking on the camera.
 *      Since this is a simulation camera, this method will just set the member variable
 *      and then use the NavBoard to simulate the camera's position.
 *
 * @param fExpectedCameraHeightFromFloorTolerance - The expected camera height from the floor tolerance.
 * @return sl::ERROR_CODE - The error code returned by the ZED SDK. In this case, it will always be SUCCESS.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
sl::ERROR_CODE SIMZEDCam::EnablePositionalTracking(const float fExpectedCameraHeightFromFloorTolerance)
{
    // Unused parameter.
    (void) fExpectedCameraHeightFromFloorTolerance;
    // Update member variables.
    m_bCameraPositionalTrackingEnabled = true;

    return sl::ERROR_CODE::SUCCESS;
}

/******************************************************************************
 * @brief This method is used to disable positional tracking on the camera.
 *      Since this is a simulation camera, this method will just set the member variable.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
void SIMZEDCam::DisablePositionalTracking()
{
    // Update member variables.
    m_bCameraPositionalTrackingEnabled = false;
}

/******************************************************************************
 * @brief This method is used to set the positional pose of the camera.
 *      Since this is a simulation camera, this method will just set the offset member variables.
 *
 * @param dX - The new X position of the camera in ZED_MEASURE_UNITS.
 * @param dY - The new Y position of the camera in ZED_MEASURE_UNITS.
 * @param dZ - The new Z position of the camera in ZED_MEASURE_UNITS.
 * @param dXO - The new tilt of the camera around the X axis in degrees.
 * @param dYO - The new tilt of the camera around the Y axis in degrees.
 * @param dZO - The new tilt of the camera around the Z axis in degrees.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
void SIMZEDCam::SetPositionalPose(const double dX, const double dY, const double dZ, const double dXO, const double dYO, const double dZO)
{
    // Acquire lock on the current rover pose mutex.
    std::unique_lock<std::shared_mutex> lkPose(m_muCurrentRoverPoseMutex);

    // Update offset member variables.
    m_dPoseOffsetX  = dX - m_stCurrentRoverPose.GetUTMCoordinate().dEasting;
    m_dPoseOffsetY  = dY - m_stCurrentRoverPose.GetUTMCoordinate().dAltitude;
    m_dPoseOffsetZ  = dZ - m_stCurrentRoverPose.GetUTMCoordinate().dNorthing;
    m_dPoseOffsetXO = dXO;
    m_dPoseOffsetYO = dYO - m_stCurrentRoverPose.GetCompassHeading();
    m_dPoseOffsetZO = dZO;
}

/******************************************************************************
 * @brief Accessor for the camera open status.
 *
 * @return true - The camera has been successfully opened.
 * @return false - The camera has not been successfully opened.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
bool SIMZEDCam::GetCameraIsOpen()
{
    return m_pRGBStream->GetIsConnected();    //&& m_pDepthImageStream->GetIsConnected() && m_pDepthMeasureStream->GetIsConnected();
}

/******************************************************************************
 * @brief Returns if the camera is using GPU memory. This is a simulation camera,
 *       so this method will always return false.
 *
 * @return true - We are using GPU memory.
 * @return false - We are not using GPU memory.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
bool SIMZEDCam::GetUsingGPUMem() const
{
    return false;
}

/******************************************************************************
 * @brief Accessor for the name of this model of camera.
 *
 * @return std::string - The model of the camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
std::string SIMZEDCam::GetCameraModel()
{
    return "SIMZED2i";
}

/******************************************************************************
 * @brief Puts a sl::GeoPose pointer into a queue so a copy of a GeoPose from the camera can be written to it.
 *
 * @param stPose - A reference to the sl::GeoPose to store the GeoPose in.
 * @return std::future<bool> - A future that should be waited on before the passed in GeoPose is used.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
std::future<bool> SIMZEDCam::RequestPositionalPoseCopy(ZEDCamera::Pose& stPose)
{
    // Check if positional tracking is enabled.
    if (m_bCameraPositionalTrackingEnabled)
    {
        // Assemble the data container.
        containers::DataFetchContainer<Pose> stContainer(stPose);

        // Acquire lock on pose copy queue.
        std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
        // Append pose fetch container to the schedule queue.
        m_qPoseCopySchedule.push(stContainer);
        // Release lock on the pose schedule queue.
        lkSchedulers.unlock();

        // Check if pose queue toggle has already been set.
        if (!m_bPosesQueued.load(ATOMIC_MEMORY_ORDER_METHOD))
        {
            // Signify that the pose queue is not empty.
            m_bPosesQueued.store(true, ATOMIC_MEMORY_ORDER_METHOD);
        }

        // Return the future from the promise stored in the container.
        return stContainer.pCopiedDataStatus->get_future();
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "Attempted to get ZED positional pose but positional tracking is not enabled or is still initializing!");

        // Create dummy promise to return the future.
        std::promise<bool> pmDummyPromise;
        std::future<bool> fuDummyFuture = pmDummyPromise.get_future();
        // Set future value.
        pmDummyPromise.set_value(false);

        // Return unsuccessful.
        return fuDummyFuture;
    }
}

/******************************************************************************
 * @brief Puts a sl::GeoPose pointer into a queue so a copy of a GeoPose from the camera can be written to it.
 *
 * @param slGeoPose - A reference to the sl::GeoPose to store the GeoPose in.
 * @return std::future<bool> - A future that should be waited on before the passed in GeoPose is used.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
std::future<bool> SIMZEDCam::RequestFusionGeoPoseCopy(sl::GeoPose& slGeoPose)
{
    // Check if positional tracking has been enabled.
    if (m_bCameraIsFusionMaster && m_bCameraPositionalTrackingEnabled)
    {
        // Assemble the data container.
        containers::DataFetchContainer<sl::GeoPose> stContainer(slGeoPose);

        // Acquire lock on frame copy queue.
        std::unique_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
        // Append frame fetch container to the schedule queue.
        m_qGeoPoseCopySchedule.push(stContainer);
        // Release lock on the frame schedule queue.
        lkSchedulers.unlock();

        // Check if pose queue toggle has already been set.
        if (!m_bGeoPosesQueued.load(ATOMIC_MEMORY_ORDER_METHOD))
        {
            // Signify that the pose queue is not empty.
            m_bGeoPosesQueued.store(true, ATOMIC_MEMORY_ORDER_METHOD);
        }

        // Return the future from the promise stored in the container.
        return stContainer.pCopiedDataStatus->get_future();
    }
    else
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger,
                    "Attempted to get ZED FUSION geo pose but positional tracking is not enabled and/or this camera was not initialized as a Fusion Master!");

        // Create dummy promise to return the future.
        std::promise<bool> pmDummyPromise;
        std::future<bool> fuDummyFuture = pmDummyPromise.get_future();
        // Set future value.
        pmDummyPromise.set_value(false);

        // Return unsuccessful.
        return fuDummyFuture;
    }
}

/******************************************************************************
 * @brief Accessor for the if the camera's positional tracking is enabled.
 *      Since this is a simulation camera, this method will just return the member variable.
 *
 * @return true - Positional tracking is enabled.
 * @return false - Positional tracking is disabled.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-12-26
 ******************************************************************************/
bool SIMZEDCam::GetPositionalTrackingEnabled()
{
    return m_bCameraPositionalTrackingEnabled && this->GetCameraIsOpen();
}
