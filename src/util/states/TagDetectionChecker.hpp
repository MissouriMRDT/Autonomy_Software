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
        int nFutureIdx = 0;
        for (size_t siIdx = 0; siIdx < siNumTagDetectors; ++siIdx)
        {
            // Only check the buffer if the detector was ready and actually spawned a future
            if (vTagDetectors[siIdx]->GetIsReady())
            {
                // Wait for the correct future to finish
                vDetectedArucoTagsFuture[nFutureIdx].get();
                nFutureIdx++;

                // Loop through the detected tags using the correct buffer index (siIdx)
                for (const tagdetectutils::ArucoTag& tTag : vDetectedArucoTagBuffers[siIdx])
                {
                    vDetectedArucoTags.emplace_back(tTag);
                }
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
     * @return int - The total number of tags currently detected.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-04-04
     ******************************************************************************/
    inline int IdentifyTargetMarker(const std::vector<std::shared_ptr<TagDetector>>& vTagDetectors,
                                    tagdetectutils::ArucoTag& stArucoTarget,
                                    tagdetectutils::ArucoTag& stTorchTarget,
                                    const int nTargetTagID = static_cast<int>(manifest::Autonomy::AUTONOMYWAYPOINTTYPES::ANY))
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
            double dTagTotalAge = std::fabs(std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - stCandidate.tmCreation).count() / 1000.0);
            // Calculate the total tag area.
            double dArea = stCandidate.pBoundingBox->area();
            // Calculate what percentage of the screen the tag takes up.
            double dAreaPercentage = (dArea / (stCandidate.cvImageResolution.width * stCandidate.cvImageResolution.height)) * 100.0;

            // If the distance of the tag is not greater than 0, skip it.
            if (stCandidate.dStraightLineDistance <= 0.0)
            {
                continue;
            }

            // Check the tag detection method type.
            if (stCandidate.eDetectionMethod == tagdetectutils::TagDetectionMethod::eOpenCV)
            {
                // Assemble the identified tags string.
                szIdentifiedTags += "\tArUco ID: " + std::to_string(stCandidate.nID) + " Tag Age: " + std::to_string(dTagTotalAge) +
                                    "s Tag Screen Percentage: " + std::to_string(dAreaPercentage) + "%\n";
                // Check if the tag is best.
                if (stCandidate.nID == nTargetTagID || static_cast<int>(manifest::Autonomy::AUTONOMYWAYPOINTTYPES::ANY))
                {
                    // Check if the tag meets the requirements.
                    if (dAreaPercentage < constants::BBOX_MIN_SCREEN_PERCENTAGE || dTagTotalAge < constants::BBOX_MIN_LIFETIME_THRESHOLD)
                    {
                        continue;
                    }

                    // Check other tag requirements.
                    if (dArea > stArucoBestTag.pBoundingBox->area())
                    {
                        // Set the target tag to the detected tag.
                        stArucoBestTag = stCandidate;
                    }
                }
            }
            else if (stCandidate.eDetectionMethod == tagdetectutils::TagDetectionMethod::eTorch)
            {
                // Assemble the identified tags string.
                szIdentifiedTags += "\tTorch Class: " + stCandidate.szClassName + " Tag Age: " + std::to_string(dTagTotalAge) +
                                    "s Tag Screen Percentage: " + std::to_string(dAreaPercentage) + "%\n";
                // Check if the tag meets the requirements.
                if (dAreaPercentage < constants::BBOX_MIN_SCREEN_PERCENTAGE || dTagTotalAge < constants::BBOX_MIN_LIFETIME_THRESHOLD)
                {
                    continue;
                }

                // Check other tag requirements.
                if (dArea > stTorchBestTag.pBoundingBox->area())
                {
                    // Set the target tag to the detected tag.
                    stTorchBestTag = stCandidate;
                }
            }
        }

        // Only print the identified tags if there are any.
        if (stArucoBestTag.nID != -1 || stTorchBestTag.dConfidence != 0.0)
        {
            // Submit logger message.
            LOG_DEBUG(logging::g_qSharedLogger, "TagDetectionChecker: Identified tags:\n{}", szIdentifiedTags);
        }

        // Set the target tag to the best tag.
        stArucoTarget = stArucoBestTag;
        stTorchTarget = stTorchBestTag;

        return static_cast<int>(vDetectedArucoTags.size());
    }
}    // namespace statemachine
#endif
