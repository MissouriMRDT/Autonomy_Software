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

#include "../../vision/aruco/ArucoDetection.hpp"
#include "../../vision/aruco/TagDetector.h"
#include "../../vision/aruco/TensorflowTagDetection.hpp"

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
     * @brief Aggregates all detected tags from each provided tag detector for both OpenCV and Tensorflow detection.
     *
     * @note When using bUnique, if you wish to prioritize one tag detector's detections over another put that tag detector earlier in the vTagDetectors.
     *
     * @param vDetectedArucoTags - Reference vector that will hold all of the aggregated detected Aruco tags.
     * @param vDetectedTensorflowTags - Reference vector that will hold all of the aggregated detected Tensorflow tags.
     * @param vTagDetectors - Vector of pointers to tag detectors that will be used to request their detected tags.
     * @param bUnique - Ensure vDetectedArucoTags is a unique list of tags (unique by ID).
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-03-07
     ******************************************************************************/
    inline void LoadDetectedTags(std::vector<arucotag::ArucoTag>& vDetectedArucoTags,
                                 std::vector<tensorflowtag::TensorflowTag>& vDetectedTensorflowTags,
                                 const std::vector<TagDetector*>& vTagDetectors,
                                 bool bUnique = false)
    {
        // Number of tag detectors.
        size_t siNumTagDetectors = vTagDetectors.size();

        // Initialize vectors to store detected tags temporarily.
        std::vector<std::vector<arucotag::ArucoTag>> vDetectedArucoTagBuffers(siNumTagDetectors);
        std::vector<std::vector<tensorflowtag::TensorflowTag>> vDetectedTensorflowTagBuffers(siNumTagDetectors);

        // Initialize vectors to store detected tags futures.
        std::vector<std::future<bool>> vDetectedArucoTagsFuture;
        std::vector<std::future<bool>> vDetectedTensorflowTagsFuture;

        // Request tags from each detector.
        for (size_t siIdx = 0; siIdx < siNumTagDetectors; ++siIdx)
        {
            // Check if this tag detector is ready.
            if (vTagDetectors[siIdx]->GetIsReady())
            {
                // Request detected Aruco tags from detector.
                vDetectedArucoTagsFuture.emplace_back(vTagDetectors[siIdx]->RequestDetectedArucoTags(vDetectedArucoTagBuffers[siIdx]));
                // Request detected Tensorflow tags from detector.
                vDetectedTensorflowTagsFuture.emplace_back(vTagDetectors[siIdx]->RequestDetectedTensorflowTags(vDetectedTensorflowTagBuffers[siIdx]));
            }
        }

        // Ensure all requests have been fulfilled.
        // Then transfer tags from the buffer to vDetectedArucoTags and vDetectedTensorflowTags for the user to access.
        for (size_t siIdx = 0; siIdx < vDetectedArucoTagsFuture.size(); ++siIdx)
        {
            // Wait for the request to be fulfilled.
            vDetectedArucoTagsFuture[siIdx].get();
            vDetectedTensorflowTagsFuture[siIdx].get();

            // Loop through the detected Aruco tags and add them to the vDetectedArucoTags vector.
            for (const arucotag::ArucoTag& tTag : vDetectedArucoTagBuffers[siIdx])
            {
                vDetectedArucoTags.emplace_back(tTag);
            }

            // Loop through the detected Tensorflow tags and add them to the vDetectedTensorflowTags vector.
            for (const tensorflowtag::TensorflowTag& tTag : vDetectedTensorflowTagBuffers[siIdx])
            {
                vDetectedTensorflowTags.emplace_back(tTag);
            }
        }

        if (bUnique)
        {
            // Remove all Aruco tags with a duplicate ID.
            std::set<int> setIds;
            size_t szIdx = 0;
            while (szIdx < vDetectedArucoTags.size())
            {
                // Tag was detected by another tag detector.
                if (setIds.count(vDetectedArucoTags[szIdx].nID))
                {
                    vDetectedArucoTags.erase(vDetectedArucoTags.begin() + szIdx);
                }
                else
                {
                    setIds.insert(vDetectedArucoTags[szIdx].nID);
                    ++szIdx;
                }
            }
        }
    }

    /******************************************************************************
     * @brief Find a tag in the rover's vision with the specified ID, using OpenCV detection.
     *
     * @param nID - The ID of the tag being looked for.
     * @param stIdentifiedArucoTag - Reference to save the identified tag.
     * @param vTagDetectors - Vector of pointers to tag detectors that will be used to request their detected tags.
     * @return true - The tag was found.
     * @return false - The tag was not found.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-03-08
     ******************************************************************************/
    inline bool FindArucoTagByID(int nID, arucotag::ArucoTag& stIdentifiedArucoTag, const std::vector<TagDetector*>& vTagDetectors)
    {
        // Load all detected tags in the rover's vision.
        std::vector<arucotag::ArucoTag> vDetectedArucoTags;
        std::vector<tensorflowtag::TensorflowTag> vDetectedTensorflowTags;
        LoadDetectedTags(vDetectedArucoTags, vDetectedTensorflowTags, vTagDetectors, true);

        // Find the tag with the corresponding id.
        for (const arucotag::ArucoTag& tTag : vDetectedArucoTags)
        {
            // Is this the tag being searched for.
            if (tTag.nID == nID)
            {
                stIdentifiedArucoTag = tTag;
                return true;
            }
        }

        // The tag was not found by the tag detectors.
        return false;
    }

    // LEAD: Commented this out since TensorflowTag has no ID. Can this be removed or reimplemented in another way?
    // /******************************************************************************
    //  * @brief Find a tag in the rover's vision with the specified ID, using Tensorflow detection.
    //  *
    //  * @param nID - The ID of the tag being looked for.
    //  * @param tIdentifiedTag - Reference to save the identified tag.
    //  * @param vTagDetectors - Vector of pointers to tag detectors that will be used to request their detected tags.
    //  * @return true - The tag was found.
    //  * @return false - The tag was not found.
    //  *
    //  * @author JSpencerPittman (jspencerpittman@gmail.com)
    //  * @date 2024-03-08
    //  ******************************************************************************/
    // inline bool FindTensorflowTagByID(int nID, tensorflowtag::TensorflowTag& stIdentifiedTag, const std::vector<TagDetector*>& vTagDetectors)
    // {
    //     // Load all detected tags in the rover's vision.
    //     std::vector<tensorflowtag::TensorflowTag> vDetectedTags;
    //     LoadDetectedTensorflowTags(vDetectedTags, vTagDetectors, true);

    //     // Find the tag with the corresponding id.
    //     for (const tensorflowtag::TensorflowTag& tTag : vDetectedTags)
    //     {
    //         // Is this the tag being searched for.
    //         if (tTag.nID == nID)
    //         {
    //             stIdentifiedTag = tTag;
    //             return true;
    //         }
    //     }

    //     // The tag was not found by the tag detectors.
    //     return false;
    // }
}    // namespace tagdetectutils

#endif
