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
     * @brief Aggregates all detected tags from each provided tag detector for OpenCV detection.
     *
     * @note When using bUnique, if you wish to prioritize one tag detector's detections over another put that tag detector earlier in the vTagDetectors.
     *
     * @param vDetectedTags - Reference vector that will hold all of the aggregated detected tags.
     * @param vTagDetectors - Vector of pointers to tag detectors that will be used to request their detected tags.
     * @param bUnique - Ensure vDetectedTags is a unique list of tags (unique by ID).
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-03-07
     ******************************************************************************/
    inline void LoadDetectedArucoTags(std::vector<arucotag::ArucoTag>& vDetectedTags, const std::vector<TagDetector*>& vTagDetectors, bool bUnique = false)
    {
        // Number of tag detectors.
        size_t siNumTagDetectors = vTagDetectors.size();

        // Initialize vectors to store detected tags temporarily.
        // Using pointers the interference between vectors being updated across different threads should be minimal.
        std::vector<std::vector<arucotag::ArucoTag>> vDetectedTagBuffers;
        vDetectedTagBuffers.resize(siNumTagDetectors);

        // Initialize vectors to stored detected tags
        std::vector<std::future<bool>> vDetectedTagsFuture;

        // Request tags from each detector.
        for (size_t siIdx = 0; siIdx < siNumTagDetectors; ++siIdx)
        {
            // Check if this tag detector is ready.
            if (vTagDetectors[siIdx]->GetIsReady())
            {
                // Request detected tags from detector.
                vDetectedTagsFuture.emplace_back(vTagDetectors[siIdx]->RequestDetectedArucoTags(vDetectedTagBuffers[siIdx]));
            }
        }

        // Ensure all requests have been fulfilled.
        // Then transfer tags from the buffer to vDetectedTags for the user to access.
        for (size_t siIdx = 0; siIdx < vDetectedTagsFuture.size(); ++siIdx)
        {
            vDetectedTagsFuture[siIdx].get();
            vDetectedTags.insert(vDetectedTags.end(), vDetectedTagBuffers[siIdx].begin(), vDetectedTagBuffers[siIdx].end());
        }

        if (bUnique)
        {
            // Remove all tags with a duplicate ID.
            // This is done in ascending order. This means that if a user wishes to prioritize tags detected from
            //  specific tag detectors they should be first in the vector.
            std::set<int> setIds;
            size_t szIdx = 0;
            while (szIdx < vDetectedTags.size())
            {
                // Tag was detected by another tag detector.
                if (setIds.count(vDetectedTags[szIdx].nID))
                {
                    vDetectedTags.erase(vDetectedTags.begin() + szIdx);
                }
                else
                {
                    ++szIdx;
                }
            }
        }
    }

    /******************************************************************************
     * @brief Aggregates all detected tags from each provided tag detector for Tensorflow detection.
     *
     * @note When using bUnique, if you wish to prioritize one tag detector's detections over another put that tag detector earlier in the vTagDetectors.
     *
     * @param vDetectedTags - Reference vector that will hold all of the aggregated detected tags.
     * @param vTagDetectors - Vector of pointers to tag detectors that will be used to request their detected tags.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-03-07
     ******************************************************************************/
    inline void LoadDetectedTensorflowTags(std::vector<tensorflowtag::TensorflowTag>& vDetectedTags, const std::vector<TagDetector*>& vTagDetectors)
    {
        // Number of tag detectors.
        size_t siNumTagDetectors = vTagDetectors.size();

        // Initialize vectors to store detected tags temporarily.
        // Using pointers the interference between vectors being updated across different threads should be minimal.
        std::vector<std::vector<tensorflowtag::TensorflowTag>> vDetectedTagBuffers;
        vDetectedTagBuffers.resize(siNumTagDetectors);

        // Initialize vectors to stored detected tags
        std::vector<std::future<bool>> vDetectedTagsFuture;

        // Request tags from each detector.
        for (size_t siIdx = 0; siIdx < siNumTagDetectors; ++siIdx)
        {
            // Check if this tag detector is ready.
            if (vTagDetectors[siIdx]->GetIsReady())
            {
                // Request detected tags from detector.
                vDetectedTagsFuture.emplace_back(vTagDetectors[siIdx]->RequestDetectedTensorflowTags(vDetectedTagBuffers[siIdx]));
            }
        }

        // Ensure all requests have been fulfilled.
        // Then transfer tags from the buffer to vDetectedTags for the user to access.
        for (size_t siIdx = 0; siIdx < vDetectedTagsFuture.size(); ++siIdx)
        {
            vDetectedTagsFuture[siIdx].get();
            vDetectedTags.insert(vDetectedTags.end(), vDetectedTagBuffers[siIdx].begin(), vDetectedTagBuffers[siIdx].end());
        }

        // LEAD: Commented out since TensorflowTag no longer has ID.
        // if (bUnique)
        // {
        //     // Remove all tags with a duplicate ID.
        //     // This is done in ascending order. This means that if a user wishes to prioritize tags detected from
        //     //  specific tag detectors they should be first in the vector.
        //     std::set<int> setIds;
        //     size_t szIdx = 0;
        //     while (szIdx < vDetectedTags.size())
        //     {
        //         // Tag was detected by another tag detector.
        //         if (setIds.count(vDetectedTags[szIdx].nID))
        //         {
        //             vDetectedTags.erase(vDetectedTags.begin() + szIdx);
        //         }
        //         else
        //         {
        //             ++szIdx;
        //         }
        //     }
        // }
    }

    /******************************************************************************
     * @brief Find a tag in the rover's vision with the specified ID, using OpenCV detection.
     *
     * @param nID - The ID of the tag being looked for.
     * @param tIdentifiedTag - Reference to save the identified tag.
     * @param vTagDetectors - Vector of pointers to tag detectors that will be used to request their detected tags.
     * @return true - The tag was found.
     * @return false - The tag was not found.
     *
     * @author JSpencerPittman (jspencerpittman@gmail.com)
     * @date 2024-03-08
     ******************************************************************************/
    inline bool FindArucoTagByID(int nID, arucotag::ArucoTag& stIdentifiedTag, const std::vector<TagDetector*>& vTagDetectors)
    {
        // Load all detected tags in the rover's vision.
        std::vector<arucotag::ArucoTag> vDetectedTags;
        LoadDetectedArucoTags(vDetectedTags, vTagDetectors, true);

        // Find the tag with the corresponding id.
        for (const arucotag::ArucoTag& tTag : vDetectedTags)
        {
            // Is this the tag being searched for.
            if (tTag.nID == nID)
            {
                stIdentifiedTag = tTag;
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
