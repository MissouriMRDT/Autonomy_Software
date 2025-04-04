/******************************************************************************
 * @brief Implements functions related to tag detection. Tag detection runs on multiple cameras,
 *      under two modes of operations: Tensorflow or standard OpenCV. These functions make it more
 *      convenient to aggregate detected tags from all cameras for both OpenCV and Tensorflow.
 *
 * @file TagDetectionUtility.hpp
 * @author Jason Pittman (jspencerpittman@gmail.com)
 * @date 2024-10-07
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef TAG_DETECTION_UTILITY_HPP
#define TAG_DETECTION_UTILITY_HPP

/// \cond
#include <type_traits>

/// \endcond

///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * @brief Namespace containing functions to assist in tag detection.
 *
 *
 * @author Jason Pittman (jspencerpittman@gmail.com)
 * @date 2024-10-07
 ******************************************************************************/
namespace tagdetectutils
{
    /******************************************************************************
     * @brief Enum class to define the different tag detection methods available.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-04-03
     ******************************************************************************/
    enum class TagDetectionMethod
    {
        eUnknown,      // Unknown detection method.
        eOpenCV,       // Standard OpenCV detection using the ArUco library.
        eTorch,        // Torch detection using a YOLO model.
        eTensorflow    // Tensorflow detection using a YOLO model.
    };

    /******************************************************************************
     * @brief Represents a single ArUco tag. Combines attributes from TorchTag,
     *        TensorflowTag, and the original ArucoTag structs.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-04-03
     ******************************************************************************/
    struct ArucoTag
    {
        public:
            // Declare public struct member attributes.
            std::shared_ptr<cv::Rect2d> cvBoundingBox;                                              // The bounding box of the detected tag.
            double dConfidence           = 0.0;                                                     // The detection confidence of the tag (from Torch/Tensorflow models).
            double dStraightLineDistance = 0.0;                                                     // Distance between the tag and the camera.
            double dYawAngle             = 0.0;                                                     // This is the yaw angle so roll and pitch are ignored.
            int nID                      = -1;                                                      // The ID of the tag. This is set to -1 if the tag is not detected.
            std::string szClassName;                                                                // The class name of the tag (used in Torch/Tensorflow models).
            std::chrono::system_clock::time_point tmLastDetected;                                   // The time the tag was last detected.
            std::chrono::system_clock::time_point tmCreation = std::chrono::system_clock::now();    // Set the time detected to the current time.
            TagDetectionMethod eDetectionMethod              = TagDetectionMethod::eUnknown;        // The detection method used to detect the tag.

            /******************************************************************************
             * @brief Overload the equality operator for the ArucoTag struct.
             *
             * @param stOther - The other ArucoTag struct to compare to.
             * @return true - The two ArucoTag structs are equal.
             * @return false - The two ArucoTag structs are not equal.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-04-03
             ******************************************************************************/
            bool operator==(const ArucoTag& stOther) const
            {
                return cvBoundingBox == stOther.cvBoundingBox && dConfidence == stOther.dConfidence && dStraightLineDistance == stOther.dStraightLineDistance &&
                       dYawAngle == stOther.dYawAngle && nID == stOther.nID && szClassName == stOther.szClassName && tmLastDetected == stOther.tmLastDetected &&
                       tmCreation == stOther.tmCreation && eDetectionMethod == stOther.eDetectionMethod;
            }

            /******************************************************************************
             * @brief Overload the inequality operator for the ArucoTag struct.
             *
             * @param stOther - The other ArucoTag struct to compare to.
             * @return true - The two ArucoTag structs are not equal.
             * @return false - The two ArucoTag structs are equal.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-04-03
             ******************************************************************************/
            bool operator!=(const ArucoTag& stOther) const { return !(*this == stOther); }
    };
}    // namespace tagdetectutils

#endif
