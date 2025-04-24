/******************************************************************************
 * @brief This file implements the H264 encoding and decoding of depth measures.
 *
 * @file DepthMeasureEncodeDecode.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-21
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef DEPTHMEASUREENCODEDECODE_HPP
#define DEPTHMEASUREENCODEDECODE_HPP

#include "../../../src/util/ExampleChecker.h"

/// \cond
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

/// \endcond

/******************************************************************************
 * @brief This is a mouse callback function that will be called when the user
 *      clicks on the original depth image. It will print the original depth value
 *      at the clicked pixel location.
 *
 * @param nEvent - The type of mouse event (e.g., left button click).
 * @param nX - The x-coordinate of the mouse event.
 * @param nY - The y-coordinate of the mouse event.
 * @param nFlags - The flags associated with the mouse event.
 * @param pUserData - Pointer to user data (in this case, the original depth image).
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-22
 ******************************************************************************/
void OriginalDepthMouseCallback(int nEvent, int nX, int nY, int nFlags, void* pUserData)
{
    (void) nFlags;    // Unused parameter
    if (nEvent != cv::EVENT_LBUTTONDOWN)
        return;

    cv::Mat* cvDepthImage = static_cast<cv::Mat*>(pUserData);
    if (nX >= 0 && nY >= 0 && nX < cvDepthImage->cols && nY < cvDepthImage->rows)
    {
        uint16_t unDepthValue = cvDepthImage->at<uint16_t>(nY, nX);
        std::cout << "Original depth at (" << nX << ", " << nY << "): " << unDepthValue << " units" << std::endl;
    }
}

/******************************************************************************
 * @brief This is a mouse callback function that will be called when the user
 *      clicks on the decoded depth image. It will print the decoded depth value
 *      at the clicked pixel location.
 *
 * @param nEvent - The type of mouse event (e.g., left button click).
 * @param nX - The x-coordinate of the mouse event.
 * @param nY - The y-coordinate of the mouse event.
 * @param nFlags - The flags associated with the mouse event.
 * @param pUserData - Pointer to user data (in this case, the decoded depth image).
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-22
 ******************************************************************************/
void DecodedDepthMouseCallback(int nEvent, int nX, int nY, int nFlags, void* pUserData)
{
    (void) nFlags;    // Unused parameter
    if (nEvent != cv::EVENT_LBUTTONDOWN)
        return;

    cv::Mat* cvDepthImage = static_cast<cv::Mat*>(pUserData);
    if (nX >= 0 && nY >= 0 && nX < cvDepthImage->cols && nY < cvDepthImage->rows)
    {
        uint16_t unDepthValue = cvDepthImage->at<uint16_t>(nY, nX);
        std::cout << "Decoded depth at (" << nX << ", " << nY << "): " << unDepthValue << " units" << std::endl;
    }
}

/******************************************************************************
 * @brief This will implement the H264 encoding and decoding of depth measures
 *      discussed in this paper:
 *          2011-Adapting-Standard-Video-Codecs-for-Depth-Streaming
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-21
 ******************************************************************************/
void RunExample()
{
    // Encoding constants.
    const float fW                  = 65536.0f;    // Maximum depth value.
    const float fNP                 = 512.0f;      // Number of depth levels.
    const int nH264CompressionLevel = 10;          // H264 compression level. 0-51 Lower is better quality

    ///////////////////////////////////////////////////////
    // Create a fake depth measure.
    ///////////////////////////////////////////////////////
    // Create a cv::Mat to store the 16-bit depth measure.
    cv::Mat cvDepthMeasure(720, 1280, CV_16UC1, cv::Scalar(0));
    // Fill the depth measure with values of fW - 1 at each pixel.
    for (int nY = 0; nY < cvDepthMeasure.rows; ++nY)
    {
        for (int nX = 0; nX < cvDepthMeasure.cols; ++nX)
        {
            cvDepthMeasure.at<uint16_t>(nY, nX) = static_cast<uint16_t>(fW - 1);
        }
    }

    // Add shapes at different depths.
    cv::RNG RNG(12345);    // Random number generator with seed.
    // Add 10 circles.
    for (int nI = 0; nI < 10; ++nI)
    {
        int nCenterX     = RNG.uniform(50, cvDepthMeasure.cols - 50);
        int nCenterY     = RNG.uniform(50, cvDepthMeasure.rows - 50);
        int nRadius      = RNG.uniform(30, 100);
        uint16_t unDepth = static_cast<uint16_t>(RNG.uniform(5000, 60000));

        for (int nY = nCenterY - nRadius; nY <= nCenterY + nRadius; ++nY)
        {
            if (nY < 0 || nY >= cvDepthMeasure.rows)
                continue;

            for (int nX = nCenterX - nRadius; nX <= nCenterX + nRadius; ++nX)
            {
                if (nX < 0 || nX >= cvDepthMeasure.cols)
                    continue;

                if ((nX - nCenterX) * (nX - nCenterX) + (nY - nCenterY) * (nY - nCenterY) <= nRadius * nRadius)
                {
                    cvDepthMeasure.at<uint16_t>(nY, nX) = unDepth;
                }
            }
        }
    }
    // Add 8 squares.
    for (int nI = 0; nI < 8; ++nI)
    {
        int nCenterX     = RNG.uniform(50, cvDepthMeasure.cols - 50);
        int nCenterY     = RNG.uniform(50, cvDepthMeasure.rows - 50);
        int nSize        = RNG.uniform(30, 100);
        uint16_t unDepth = static_cast<uint16_t>(RNG.uniform(5000, 60000));

        for (int nY = nCenterY - nSize / 2; nY <= nCenterY + nSize / 2; ++nY)
        {
            if (nY < 0 || nY >= cvDepthMeasure.rows)
                continue;

            for (int nX = nCenterX - nSize / 2; nX <= nCenterX + nSize / 2; ++nX)
            {
                if (nX < 0 || nX >= cvDepthMeasure.cols)
                    continue;
                cvDepthMeasure.at<uint16_t>(nY, nX) = unDepth;
            }
        }
    }
    // Add 6 triangles.
    for (int nI = 0; nI < 6; ++nI)
    {
        int nCenterX     = RNG.uniform(50, cvDepthMeasure.cols - 50);
        int nCenterY     = RNG.uniform(50, cvDepthMeasure.rows - 50);
        int nSize        = RNG.uniform(40, 120);
        uint16_t unDepth = static_cast<uint16_t>(RNG.uniform(5000, 60000));

        // Define triangle vertices.
        cv::Point vertices[3] = {cv::Point(nCenterX, nCenterY - nSize / 2),
                                 cv::Point(nCenterX - nSize / 2, nCenterY + nSize / 2),
                                 cv::Point(nCenterX + nSize / 2, nCenterY + nSize / 2)};

        // Fill the triangle.
        cv::fillConvexPoly(cvDepthMeasure, vertices, 3, cv::Scalar(unDepth));
    }

    ///////////////////////////////////////////////////////
    // Encode the depth measure.
    ///////////////////////////////////////////////////////
    // Create a image to store the encoded depth measure.
    cv::Mat cvEncodedDepth(cvDepthMeasure.rows, cvDepthMeasure.cols, CV_8UC3, cv::Scalar(0));
    // Encode each pixel in the depth measure.
    for (int nY = 0; nY < cvDepthMeasure.rows; ++nY)
    {
        for (int nX = 0; nX < cvDepthMeasure.cols; ++nX)
        {
            // Get the depth value.
            // uint16_t unDepthVal = cvDepthMeasure.at<uint16_t>(nY, nX);
            uint16_t unDepthVal = 2000;

            // Normalize the depth value.
            float fNormalizedDepth = (unDepthVal + 0.5f) / fW;

            // Period for triangle waves.
            float fP = fNP / fW;

            // L(d): Linear mapping for low-resolution depth.
            float fL = fNormalizedDepth;

            // H_a(d): Triangle wave function 1.
            float fHa = fmod(fL / (fP / 2.0f), 2.0f);
            fHa       = fHa <= 1.0f ? fHa : 2.0f - fHa;

            // H_b(d): Triangle wave function 2 (phase-shifted by π/4).
            float fHb = fmod((fL - (fP / 4.0f)) / (fP / 2.0f), 2.0f);
            fHb       = fHb <= 1.0f ? fHb : 2.0f - fHb;

            // Scale values to 0-255 range for 8-bit storage
            uchar unY = static_cast<uchar>(fL * 255.0f);
            uchar unU = static_cast<uchar>(fHa * 255.0f);
            uchar unV = static_cast<uchar>(fHb * 255.0f);

            // Assign encoded values to output image.
            cvEncodedDepth.at<cv::Vec3b>(nY, nX) = cv::Vec3b(unY, unU, unV);
        }
    }

    // Use ffmpeg to emulate the H264 encoding.
    // Save the encoded depth to a temporary file
    std::string szInputFilename  = "temp_encoded_depth.png";
    std::string szH264Filename   = "temp_h264.mp4";
    std::string szOutputFilename = "temp_decoded_depth.png";

    // Save original encoded depth image.
    if (!cv::imwrite(szInputFilename, cvEncodedDepth))
    {
        std::cerr << "Failed to save encoded depth image to " << szInputFilename << std::endl;
    }
    std::cout << "Simulating H264 encoding/decoding with ffmpeg..." << std::endl;

    // Keep a copy of the pre-H264 encoded depth for comparison.
    cv::Mat cvPreH264Depth = cvEncodedDepth.clone();

    // Build the ffmpeg commands with appropriate options for depth data.
    // Using higher quality (lower CRF) to preserve depth information. CRF range is 0-51, where lower is better quality.
    std::string szEncoderCMD = "ffmpeg -y -i " + szInputFilename + " -c:v libx264 -preset fast -tune zerolatency -crf " + std::to_string(nH264CompressionLevel) + " " +
                               szH264Filename + " 2>/dev/null";

    std::string szDecoderCMD = "ffmpeg -y -i " + szH264Filename + " -frames:v 1 " + szOutputFilename + " 2>/dev/null";

    // Run encoder and decoder.
    int nEncoderResult = system(szEncoderCMD.c_str());
    if (nEncoderResult != 0)
    {
        std::cerr << "Error running ffmpeg encoder. Make sure ffmpeg is installed." << std::endl;
    }

    int nDecoderResult = system(szDecoderCMD.c_str());
    if (nDecoderResult != 0)
    {
        std::cerr << "Error running ffmpeg decoder." << std::endl;
    }

    // Read the H264-processed image back.
    cv::Mat cvH264ProcessedDepth = cv::imread(szOutputFilename);
    if (cvH264ProcessedDepth.empty())
    {
        std::cerr << "Error reading H264-processed depth image" << std::endl;
    }
    else
    {
        // Display both versions for comparison.
        cv::imshow("Pre-H264 Encoded Depth", cvPreH264Depth);
        cv::imshow("Post-H264 Encoded Depth", cvH264ProcessedDepth);
        cv::waitKey(500);    // Short pause to ensure images are displayed.

        // Replace the original encoded depth with the H264-processed version.
        cvEncodedDepth = cvH264ProcessedDepth;

        std::cout << "H264 encoding/decoding simulation complete" << std::endl;
    }

// Clean up temporary files.
#ifdef _WIN32
    system(("del " + szInputFilename + " " + szH264Filename + " " + szOutputFilename).c_str());
#else
    system(("rm -f " + szInputFilename + " " + szH264Filename + " " + szOutputFilename).c_str());
#endif

    ///////////////////////////////////////////////////////
    // Decode the depth measure.
    ///////////////////////////////////////////////////////
    // Create a cv::Mat to store the decoded depth measure.
    cv::Mat cvDecodedDepth(cvEncodedDepth.rows, cvEncodedDepth.cols, CV_16UC1, cv::Scalar(0));
    // Decode each pixel in the encoded depth image.
    for (int nY = 0; nY < cvEncodedDepth.rows; ++nY)
    {
        for (int nX = 0; nX < cvEncodedDepth.cols; ++nX)
        {
            // Extract the encoded depth values.
            cv::Vec3b cvPixel = cvEncodedDepth.at<cv::Vec3b>(nY, nX);

            // Extract encoded values.
            float fL  = cvPixel[0] / 255.0f;
            float fHa = cvPixel[1] / 255.0f;
            float fHb = cvPixel[2] / 255.0f;

            // Period for triangle waves.
            float fP = fNP / fW;

            // Determine offset and fine-grain correction.
            int fM       = static_cast<int>(std::floor((4.0f * (fL / fP)) - 0.5f)) % 4;
            float fL0    = fL - fmod(fL - (fP / 8.0f), fP) + ((fP / 4.0f) * fM) - (fP / 8.0f);

            float fDelta = 0.0f;
            if (fM == 0)
                fDelta = (fP / 2.0f) * fHa;
            else if (fM == 1)
                fDelta = (fP / 2.0f) * fHb;
            else if (fM == 2)
                fDelta = (fP / 2.0f) * (1.0f - fHa);
            else if (fM == 3)
                fDelta = (fP / 2.0f) * (1.0f - fHb);

            // Combine to compute the original depth.
            float fDepth = fW * (fL0 + fDelta);

            // Clamp the depth value to valid range.
            if (fDepth < 0.0f)
                fDepth = 0.0f;
            else if (fDepth > fW)
                fDepth = fW;

            // Store the decoded depth.
            cvDecodedDepth.at<uint16_t>(nY, nX) = static_cast<uint16_t>(fDepth);
        }
    }

    // Display the original and decoded depth measures.
    cv::imshow("Original Depth Measure", cvDepthMeasure);
    cv::imshow("Decoded Depth Measure", cvDecodedDepth);
    // Set mouse callbacks for original and decoded depth images.
    cv::setMouseCallback("Original Depth Measure", OriginalDepthMouseCallback, &cvDepthMeasure);
    cv::setMouseCallback("Decoded Depth Measure", DecodedDepthMouseCallback, &cvDecodedDepth);
    // Print instructions for the user.
    std::cout << "Click on images to see depth values. Press 'q' or ESC to exit." << std::endl;

    // Wait for 'q' or ESC key to be pressed.
    int nKey;
    while (true)
    {
        nKey = cv::waitKey(0);            // Wait indefinitely for a key press.
        if (nKey == 'q' || nKey == 27)    // 'q' or ESC.
            break;
    }
    cv::destroyAllWindows();

    // Use the decoded depth measure to calculate a point cloud.
    cv::Mat cvPointCloud(cvDecodedDepth.rows, cvDecodedDepth.cols, CV_32FC4, cv::Scalar(0));
    // Camera parameters.
    double dHorizontalFOV = 90.0;    // Horizontal field of view in degrees.
    double dVerticalFOV   = 60.0;    // Vertical field of view in degrees.
    // Calculate focal lengths from FOV.
    const double dRadPerDeg = M_PI / 180.0;
    const double dFx        = (cvDecodedDepth.cols / 2.0) / tan(dHorizontalFOV * dRadPerDeg / 2.0);
    const double dFy        = (cvDecodedDepth.rows / 2.0) / tan(dVerticalFOV * dRadPerDeg / 2.0);

    // Image center. (principal point)
    const double dCx = cvDecodedDepth.cols / 2.0;
    const double dCy = cvDecodedDepth.rows / 2.0;

    // Calculate 3D points from depth values.
    for (int nY = 0; nY < cvDecodedDepth.rows; ++nY)
    {
        for (int nX = 0; nX < cvDecodedDepth.cols; ++nX)
        {
            // Get depth value.
            float fDepth = static_cast<float>(cvDecodedDepth.at<uint16_t>(nY, nX));

            // Skip invalid depth values.
            if (fDepth <= 0)
            {
                cvPointCloud.at<cv::Vec4f>(nY, nX) = cv::Vec4f(0, 0, 0, 0);
                continue;
            }

            // Convert from pixel coordinates to 3D coordinates.
            float fX = static_cast<float>((nX - dCx) * fDepth / dFx);
            float fY = static_cast<float>((nY - dCy) * fDepth / dFy);
            float fZ = fDepth;

            // Store point. (XYZ + intensity, using Y channel for intensity)
            cvPointCloud.at<cv::Vec4f>(nY, nX) = cv::Vec4f(fX, fY, fZ, 255);
        }
    }
    std::cout << "Point cloud generated with " << (cvDecodedDepth.rows * cvDecodedDepth.cols) << " points." << std::endl;

    // Use PCL to visualize the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Convert the OpenCV point cloud to PCL format.
    pclCloud->width    = cvPointCloud.cols;
    pclCloud->height   = cvPointCloud.rows;
    pclCloud->is_dense = false;
    pclCloud->points.resize(pclCloud->width * pclCloud->height);

    // Copy points from OpenCV format to PCL format.
    for (int nY = 0; nY < cvPointCloud.rows; ++nY)
    {
        for (int nX = 0; nX < cvPointCloud.cols; ++nX)
        {
            cv::Vec4f point = cvPointCloud.at<cv::Vec4f>(nY, nX);
            size_t idx      = nY * cvPointCloud.cols + nX;

            // Only add points with valid depth.
            if (point[2] > 0)
            {
                pclCloud->points[idx].x = point[0];
                pclCloud->points[idx].y = point[1];
                pclCloud->points[idx].z = point[2];
            }
            else
            {
                // For invalid points, set to NaN.
                pclCloud->points[idx].x = std::numeric_limits<float>::quiet_NaN();
                pclCloud->points[idx].y = std::numeric_limits<float>::quiet_NaN();
                pclCloud->points[idx].z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }

    // Remove NaN points to clean up the cloud..
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclFilteredPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> vIndices;
    pcl::removeNaNFromPointCloud(*pclCloud, *pclFilteredPointCloud, vIndices);
    std::cout << "Point cloud filtered: " << pclFilteredPointCloud->points.size() << " valid points." << std::endl;

    // Downsample the point cloud for better visualization performance.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclDownsampledPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> pclVoxelGrid;
    pclVoxelGrid.setInputCloud(pclFilteredPointCloud);
    pclVoxelGrid.setLeafSize(5.0f, 5.0f, 5.0f);
    pclVoxelGrid.filter(*pclDownsampledPointCloud);
    std::cout << "Point cloud downsampled to " << pclDownsampledPointCloud->points.size() << " points." << std::endl;

    // Create a PCL visualizer.
    pcl::visualization::PCLVisualizer::Ptr pclViewer(new pcl::visualization::PCLVisualizer("Depth Point Cloud"));
    pclViewer->setBackgroundColor(0, 0, 0);
    pclViewer->addPointCloud<pcl::PointXYZ>(pclDownsampledPointCloud, "depth_cloud");
    pclViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "depth_cloud");
    pclViewer->addCoordinateSystem(1000.0);
    pclViewer->initCameraParameters();

    std::cout << "Visualizing point cloud. Close the pclViewer window to continue." << std::endl;

    // Wait for the pclViewer to close.
    while (!pclViewer->wasStopped())
    {
        pclViewer->spinOnce(100);
    }
}

#endif
