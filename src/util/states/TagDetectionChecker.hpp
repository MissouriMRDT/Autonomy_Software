/******************************************************************************
 * @brief TagDetectionChecker class is responsible for checking if the rover is
 * has detected a tag and if it meets the requirements to be considered a valid tag.
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
#include <tracy/Tracy.hpp>

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
     * @brief Aggregates all detected tags from each provided tag detector for both OpenCV and YOLO detection.
     *
     *      Each detector's tags are read as a single lock-free load of its newest immutable
     *      snapshot, so this never blocks on a detector's loop. Detectors publish their detected
     *      tags unconditionally (the tags are already computed by the detection pass), so no
     *      Reader is required only to keep it published, not to read it.
     *
     * @param vDetectedArucoTags - Reference vector that will hold all of the aggregated detected Aruco tags.
     * @param vTagDetectors - Vector of pointers to tag detectors that will be read from.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-04-04
     ******************************************************************************/
    inline void LoadDetectedTags(std::vector<tagdetectutils::ArucoTag>& vDetectedArucoTags, const std::vector<std::shared_ptr<TagDetector>>& vTagDetectors)
    {
        ZoneScopedC(tracy::Color::MediumPurple);
        // Read the newest published tags from each detector.
        for (const std::shared_ptr<TagDetector>& pTagDetector : vTagDetectors)
        {
            // Skip detectors that are not up and running.
            if (pTagDetector == nullptr || !pTagDetector->GetIsReady())
            {
                // Nothing to aggregate from this detector.
                continue;
            }

            // Load the newest snapshot once into a local so it cannot change while we read it.
            pubsub::SharedSnapshot<std::vector<tagdetectutils::ArucoTag>> pSnapshot = pTagDetector->GetDetectedTagsReader().Get();
            // Nothing has been published yet.
            if (pSnapshot == nullptr)
            {
                // Skip this detector for now.
                continue;
            }

            // Loop through this detector's tags and aggregate them.
            for (const tagdetectutils::ArucoTag& stTag : pSnapshot->tData)
            {
                // Copy the tag out, then give the caller its own bounding box instance so nothing
                // downstream can mutate the shared, immutable snapshot.
                tagdetectutils::ArucoTag stTagCopy = stTag;
                stTagCopy.pBoundingBox             = std::make_shared<cv::Rect2d>(*stTag.pBoundingBox);
                vDetectedArucoTags.emplace_back(stTagCopy);
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
        ZoneScopedC(tracy::Color::MediumPurple);
        // Create instance variables.
        std::vector<tagdetectutils::ArucoTag> vDetectedArucoTags;
        tagdetectutils::ArucoTag stArucoBestTag;
        tagdetectutils::ArucoTag stTorchBestTag;
        std::string szIdentifiedTags = "";

        // Initialize best percentages to 0 so the first valid tag always wins
        double dBestArucoAreaPercentage = 0.0;
        double dBestTorchAreaPercentage = 0.0;

        // Get the current time
        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();

        // Load all detected tags in the rover's vision.
        LoadDetectedTags(vDetectedArucoTags, vTagDetectors);

        // Find the best tag from the Aruco tags.
        for (const tagdetectutils::ArucoTag& stCandidate : vDetectedArucoTags)
        {
            // Calculate the total age of the tag.
            double dTagTotalAge = std::fabs(std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - stCandidate.tmCreation).count() / 1000.0);

            // Safety check for bounding box pointer
            if (stCandidate.pBoundingBox == nullptr)
            {
                continue;
            }

            // Calculate what percentage of the screen the tag takes up.
            double dArea           = stCandidate.pBoundingBox->area();
            double dAreaPercentage = (dArea / (stCandidate.cvImageResolution.width * stCandidate.cvImageResolution.height)) * 100.0;

            // Check if the tag meets the minimum thresholds.
            if (dAreaPercentage < constants::BBOX_MIN_SCREEN_PERCENTAGE || dTagTotalAge < constants::BBOX_MIN_LIFETIME_THRESHOLD)
            {
                continue;
            }

            // --- OpenCV Tag Logic ---
            if (stCandidate.eDetectionMethod == tagdetectutils::TagDetectionMethod::eOpenCV)
            {
                szIdentifiedTags += "\tArUco ID: " + std::to_string(stCandidate.nID) + " Tag Age: " + std::to_string(dTagTotalAge) +
                                    "s Tag Screen Percentage: " + std::to_string(dAreaPercentage) + "%\n";

                // Ensure it matches the requested ID (or we accept ANY ID)
                if (stCandidate.nID == nTargetTagID || nTargetTagID == static_cast<int>(manifest::Autonomy::AUTONOMYWAYPOINTTYPES::ANY))
                {
                    // Prioritize the tag that takes up the most screen area
                    if (dAreaPercentage > dBestArucoAreaPercentage)
                    {
                        stArucoBestTag           = stCandidate;
                        dBestArucoAreaPercentage = dAreaPercentage;
                    }
                }
            }
            // --- Torch Tag Logic ---
            else if (stCandidate.eDetectionMethod == tagdetectutils::TagDetectionMethod::eTorch)
            {
                szIdentifiedTags += "\tTorch Class: " + stCandidate.szClassName + " Tag Age: " + std::to_string(dTagTotalAge) +
                                    "s Tag Screen Percentage: " + std::to_string(dAreaPercentage) + "%\n";

                // Prioritize the tag that takes up the most screen area
                if (dAreaPercentage > dBestTorchAreaPercentage)
                {
                    stTorchBestTag           = stCandidate;
                    dBestTorchAreaPercentage = dAreaPercentage;
                }
            }
        }

        // Only print the identified tags if there are any.
        if (stArucoBestTag.nID != -1 || stTorchBestTag.dConfidence != 0.0)
        {
            LOG_DEBUG(logging::g_qSharedLogger, "TagDetectionChecker: Identified tags:\n{}", szIdentifiedTags);
        }

        // Set the target tag to the best tag.
        stArucoTarget = stArucoBestTag;
        stTorchTarget = stTorchBestTag;

        return static_cast<int>(vDetectedArucoTags.size());
    }
}    // namespace statemachine
#endif
