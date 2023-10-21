// /******************************************************************************
//  * @brief Defines and implements functions related to object detection operations using
//  *      the depth measure from a ZED camera. All functions are defined within the depthobject namespace.
//  *
//  * @file DepthDetection.h
//  * @author clayjay3 (claytonraycowen@gmail.com)
//  * @date 2023-10-20
//  *
//  * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
//  ******************************************************************************/

// #ifndef DEPTH_DETECTION_HPP
// #define DEPTH_DETECTION_HPP

// #include <vector>

// #include <opencv2/opencv.hpp>

// #include "../../AutonomyConstants.h"
// #include "../../AutonomyLogging.h"
// #include "../../util/GeospatialOperations.hpp"
// #include "../../util/vision/ImageOperations.hpp"

// /******************************************************************************
//  * @brief Namespace containing functions related to object detection operations
//  *      using depth measures and images.
//  *
//  *
//  * @author clayjay3 (claytonraycowen@gmail.com)
//  * @date 2023-10-07
//  ******************************************************************************/
// namespace depthobject
// {
//     /******************************************************************************
//      * @brief Represents a single depth object. Stores all information about a specific
//      *      object detection.
//      *
//      *
//      * @author clayjay3 (claytonraycowen@gmail.com)
//      * @date 2023-10-21
//      ******************************************************************************/
//     struct DepthObject
//     {
//         public:
//             // Declare public struct member attributes.
//             cv::Point2f CornerTL;                                                                 // The top left corner of the bounding box.
//             cv::Point2f CornerTR;                                                                 // The top right corner of the bounding box.
//             cv::Point2f CornerBL;                                                                 // The bottom left corner of the bounding box.
//             cv::Point2f CornerBR;                                                                 // The bottom right corner of bounding box.
//             std::vector<cv::Point2f*> vCorners = {&CornerTL, &CornerTR, &CornerBL, &CornerBR};    // Provide an easy method for getting all corners.
//             geoops::UTMCoordinate stLocation;                                                     // The absolute position of the object stored in a UTM coordinate.
//             double dStraightLineDistance;                                                         // Distance between the tag and the camera.
//             double dYawAngle;                                                                     // This is the yaw angle so roll and pitch are ignored.
//     };

//     /******************************************************************************
//      * @brief Given an DepthObject struct find the center point of the corners.
//      *
//      * @param stTag - The object to find the center of.
//      * @return cv::Point2f - The resultant center point within the image.
//      *
//      * @author jspencerpittman (jspencerpittman@gmail.com)
//      * @date 2023-10-20
//      ******************************************************************************/
//     inline cv::Point2f FindTagCenter(const DepthObject& stTag)
//     {
//         // Average of the four corners.
//         cv::Point2f cvCenter(0, 0);

//         // Loop through each corner of the tag.
//         for (cv::Point2f* cvCorner : stTag.vCorners)
//         {
//             // Add each tag x, y to the center x, y.
//             cvCenter.x += cvCorner->x;
//             cvCenter.y += cvCorner->y;
//         }
//         // Divide by number of corners.
//         cvCenter.x /= 4;
//         cvCenter.y /= 4;

//         // Return a copy of the center point of the tag.
//         return cvCenter;
//     }

//     inline std::vector<DepthObject> Detect(const cv::Mat& cvDepthMeasure, )
//     {
//         /// Create instance variables.
//         std::vector<int> vIDs;
//         std::vector<std::vector<cv::Point2f>> cvMarkerCorners, cvRejectedCandidates;

//         // Run Aruco detection algorithm.
//         cvArucoDetector.detectMarkers(cvFrame, cvMarkerCorners, vIDs, cvRejectedCandidates);

//         // Store all of the detected tags as DepthObject.
//         std::vector<DepthObject> vDetectedTags;
//         vDetectedTags.reserve(vIDs.size());

//         // Loop through each detection and build tag for it.
//         for (long unsigned int unIter = 0; unIter < vIDs.size(); unIter++)
//         {
//             // Create and initialize new tag.
//             DepthObject stDetectedTag;
//             stDetectedTag.nID = vIDs[unIter];
//             // Copy corners.
//             stDetectedTag.CornerTL = cvMarkerCorners[unIter][0];
//             stDetectedTag.CornerTR = cvMarkerCorners[unIter][1];
//             stDetectedTag.CornerBR = cvMarkerCorners[unIter][2];
//             stDetectedTag.CornerBL = cvMarkerCorners[unIter][3];

//             // Add new tag to detected tags vector.
//             vDetectedTags.push_back(stDetectedTag);
//         }

//         // Return the detected tags.
//         return vDetectedTags
//     }

//     /******************************************************************************
//      * @brief Given a vector of DepthObject structs draw each tag corner and ID onto the given image.
//      *      Image must be a 1 or 3 channel image and image must match dimensions of image when used for
//      *      detection of the given tags.
//      *
//      * @param cvDetectionsFrame - The frame to draw overlay onto.
//      * @param vDetectedTags - The vector of DepthObject struct used to draw tag corners and IDs onto image.
//      *
//      * @author clayjay3 (claytonraycowen@gmail.com)
//      * @date 2023-10-19
//      ******************************************************************************/
//     inline void DrawDetections(cv::Mat& cvDetectionsFrame, std::vector<DepthObject> vDetectedTags)
//     {
//         // Create instance variables.
//         std::vector<int> vIDs;
//         std::vector<std::vector<cv::Point2f>> cvMarkers;

//         // Loop through each of the given AR tags and repackage them so that the draw function can read them.
//         for (long unsigned int nIter = 0; nIter < vDetectedTags.size(); ++nIter)
//         {
//             // Append tag ID.
//             vIDs.emplace_back(vDetectedTags[nIter].nID);

//             // Assemble vector of marker corners.
//             std::vector<cv::Point2f> cvMarkerCorners;
//             cvMarkerCorners.emplace_back(vDetectedTags[nIter].CornerTL);
//             cvMarkerCorners.emplace_back(vDetectedTags[nIter].CornerTR);
//             cvMarkerCorners.emplace_back(vDetectedTags[nIter].CornerBR);
//             cvMarkerCorners.emplace_back(vDetectedTags[nIter].CornerBL);
//             // Append vector of marker corners.
//             cvMarkers.emplace_back(cvMarkerCorners);
//         }

//         // Check if the given frame is a 1 or 3 channel image. (not BGRA)
//         if (!cvDetectionsFrame.empty() && (cvDetectionsFrame.channels() == 1 || cvDetectionsFrame.channels() == 3))
//         {
//             // Draw markers onto normal given image.
//             cv::aruco::drawDetectedMarkers(cvDetectionsFrame, cvMarkers, vIDs, cv::Scalar(0, 0, 0));
//         }
//         else
//         {
//             // Submit logger message.
//             LOG_ERROR(logging::g_qSharedLogger,
//                       "ArucoDetect: Unable to draw markers on image because it is empty or because it has {} channels. (Should be 1 or 3)",
//                       cvDetectionsFrame.channels());
//         }
//     }
// }    // namespace depthobject

// #endif
