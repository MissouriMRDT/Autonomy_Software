/******************************************************************************
 * @brief TagDetectionChecker class is responsible for checking if the rover is
 *       has detected a tag and if it meets the requirements to be considered a valid tag.
 *
 * @file TagDetectionChecker.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-04
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef TAG_DETECTION_CHECKER_HPP
#define TAG_DETECTION_CHECKER_HPP

#include "../../AutonomyGlobals.h"
#include "../../vision/aruco/TagDetector.h"

/// \cond

/// \endcond

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-04
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief Aggregates all detected tags from each provided tag detector for both OpenCV and Tensorflow detection.
     *
     * @param vDetectedArucoTags - Reference vector that will hold all of the aggregated detected Aruco tags.
     * @param vTagDetectors - Vector of pointers to tag detectors that will be used to request their detected tags.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-04-04
     ******************************************************************************/
    inline void LoadDetectedTags(std::vector<tagdetectutils::ArucoTag>& vDetectedArucoTags, const std::vector<std::shared_ptr<TagDetector>>& vTagDetectors)
    {
        // Number of tag detectors.
        size_t siNumTagDetectors = vTagDetectors.size();

        // Initialize vectors to store detected tags temporarily.
        std::vector<std::vector<tagdetectutils::ArucoTag>> vDetectedArucoTagBuffers(siNumTagDetectors);

        // Initialize vectors to store detected tags futures.
        std::vector<std::future<bool>> vDetectedArucoTagsFuture;

        // Request tags from each detector.
        for (size_t siIdx = 0; siIdx < siNumTagDetectors; ++siIdx)
        {
            // Check if this tag detector is ready.
            if (vTagDetectors[siIdx]->GetIsReady())
            {
                // Request detected Aruco tags from detector.
                vDetectedArucoTagsFuture.emplace_back(vTagDetectors[siIdx]->RequestDetectedArucoTags(vDetectedArucoTagBuffers[siIdx]));
            }
        }

        // Ensure all requests have been fulfilled.
        // Then transfer tags from the buffer to vDetectedArucoTags and vDetectedTensorflowTags for the user to access.
        for (size_t siIdx = 0; siIdx < vDetectedArucoTagsFuture.size(); ++siIdx)
        {
            // Wait for the request to be fulfilled.
            vDetectedArucoTagsFuture[siIdx].get();

            // Loop through the detected Aruco tags and add them to the vDetectedArucoTags vector.
            for (const tagdetectutils::ArucoTag& tTag : vDetectedArucoTagBuffers[siIdx])
            {
                vDetectedArucoTags.emplace_back(tTag);
            }
        }
    }

    /******************************************************************************
     * @brief Identify a target marker in the rover's vision, using OpenCV detection.
     *
     * @note If multiple markers are detected the closest one will be chosen as the target.
     *
     * @param vTagDetectors - The vector of tag detectors to use for detection.
     * @param stArucoTarget - The detected target marker from OpenCV.
     * @param stTorchTarget - The detected target marker from Torch.
     * @param nTargetTagID - The ID of the target tag to identify. If -1, the closest tag will be chosen.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-04-04
     ******************************************************************************/
    inline void IdentifyTargetMarker(const std::vector<std::shared_ptr<TagDetector>>& vTagDetectors,
                                     tagdetectutils::ArucoTag& stArucoTarget,
                                     tagdetectutils::ArucoTag& stTorchTarget,
                                     const int nTargetTagID = -1)
    {
        // Create instance variables.
        std::vector<tagdetectutils::ArucoTag> vDetectedArucoTags;
        tagdetectutils::ArucoTag stArucoBestTag;
        tagdetectutils::ArucoTag stTorchBestTag;
        std::string szIdentifiedTags = "";

        // Get the current time
        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();

        // Load all detected tags in the rover's vision.
        LoadDetectedTags(vDetectedArucoTags, vTagDetectors);
        // Find the best tag from the Aruco tags.
        for (const tagdetectutils::ArucoTag& stCandidate : vDetectedArucoTags)
        {
            // Calculate the total age of the tag.
            double dTagTotalAge = std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - stCandidate.tmCreation).count();
            // Calculate the total tag area.
            double dArea = stCandidate.pBoundingBox->area();

            // Check the tag detection method type.
            if (stCandidate.eDetectionMethod == tagdetectutils::TagDetectionMethod::eOpenCV)
            {
                // Assemble the identified tags string.
                szIdentifiedTags += "\tArUco ID: " + std::to_string(stCandidate.nID) + " Tag Age: " + std::to_string(dTagTotalAge) + "s\n";
                // Check if the tag is best.
                if (stCandidate.nID == nTargetTagID || nTargetTagID == -1)
                {
                    // Check other tag requirements.
                    if (dArea > stArucoBestTag.pBoundingBox->area() && dTagTotalAge > constants::ARUCO_MIN_LIFETIME_THRESHOLD)
                    {
                        // Set the target tag to the detected tag.
                        stArucoBestTag = stCandidate;
                    }
                }
            }
            else if (stCandidate.eDetectionMethod == tagdetectutils::TagDetectionMethod::eTorch)
            {
                // Assemble the identified tags string.
                szIdentifiedTags += "\tTorch Class: " + stCandidate.szClassName + " Tag Age: " + std::to_string(dTagTotalAge) + "s\n";
                // Check if the tag is best.
                if (dArea > stTorchBestTag.pBoundingBox->area() && dTagTotalAge > constants::ARUCO_MIN_LIFETIME_THRESHOLD)
                {
                    // Set the target tag to the detected tag.
                    stTorchBestTag = stCandidate;
                }
            }
        }

        // Set the target tag to the best tag.
        stArucoTarget = stArucoBestTag;
        stTorchTarget = stTorchBestTag;
    }
}    // namespace statemachine
#endif
