/******************************************************************************
 * @brief ObjectDetectionChecker class is responsible for checking if the rover is
 * has detected an object and if it meets the requirements to be considered a valid object.
 *
 * @file ObjectDetectionChecker.hpp
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2025-05-08
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef OBJECT_DETECTION_CHECKER_HPP
#define OBJECT_DETECTION_CHECKER_HPP

#include "../../AutonomyGlobals.h"
#include "../../vision/objects/ObjectDetector.h"

/// \cond
#include <tracy/Tracy.hpp>

/// \endcond

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 *
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2025-05-08
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief Aggregates all detected objects from each provided object detector.
     *
     * @param vDetectedObjects - Reference vector that will hold all of the aggregated detected objects.
     * @param vObjectDetectors - Vector of pointers to object detectors that will be used to request their detected objects.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-05-08
     ******************************************************************************/
    inline void LoadDetectedObjects(std::vector<objectdetectutils::Object>& vDetectedObjects, const std::vector<std::shared_ptr<ObjectDetector>>& vObjectDetectors)
    {
        ZoneScopedC(tracy::Color::PowderBlue);
        // Number of object detectors.
        size_t siNumObjectDetectors = vObjectDetectors.size();

        // Initialize vectors to store detected objects temporarily.
        std::vector<std::vector<objectdetectutils::Object>> vDetectedObjectBuffers(siNumObjectDetectors);

        // Initialize vectors to store detected objects futures.
        std::vector<std::future<bool>> vDetectedObjectsFuture;

        // Track exactly which cameras successfully spawned a future to prevent vector crashes.
        std::vector<bool> vSpawnedFuture(siNumObjectDetectors, false);

        // Request objects from each detector.
        for (size_t siIdx = 0; siIdx < siNumObjectDetectors; ++siIdx)
        {
            // Check if this object detector is ready.
            if (vObjectDetectors[siIdx]->GetIsReady())
            {
                // Request detected objects from detector.
                vDetectedObjectsFuture.emplace_back(vObjectDetectors[siIdx]->RequestDetectedObjects(vDetectedObjectBuffers[siIdx]));
                vSpawnedFuture[siIdx] = true;
            }
        }

        // Ensure all requests have been fulfilled.
        // Then transfer objects from the buffer to vDetectedObjects for the user to access.
        int nFutureIdx = 0;
        // Iterate through the total number of detectors to match the buffer and spawned tracking sizes.
        for (size_t siIdx = 0; siIdx < siNumObjectDetectors; ++siIdx)
        {
            // Only check the buffer if the detector was ready and actually spawned a future
            if (vSpawnedFuture[siIdx])
            {
                // Wait for the correct future to finish
                vDetectedObjectsFuture[nFutureIdx].get();
                nFutureIdx++;

                // Loop through the detected objects and add them to the vDetectedObjects vector.
                for (const objectdetectutils::Object& tObject : vDetectedObjectBuffers[siIdx])
                {
                    vDetectedObjects.emplace_back(tObject);
                }
            }
        }
    }

    /******************************************************************************
     * @brief Identify a target object in the rover's vision, using Torch detection.
     *
     * @note If multiple objects are detected the closest one will be chosen as the target.
     *
     * @param vObjectDetectors - The vector of object detectors to use for detection.
     * @param stObjectTarget - The detected object marker from Torch.
     * @param eDesiredDetectionType - The desired detection type to check for.
     * @return int - The total number of objects currently detected.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-05-09
     ******************************************************************************/
    inline int IdentifyTargetObject(const std::vector<std::shared_ptr<ObjectDetector>>& vObjectDetectors,
                                    objectdetectutils::Object& stObjectTarget,
                                    const geoops::WaypointType& eDesiredDetectionType = geoops::WaypointType::eUNKNOWN)
    {
        ZoneScopedC(tracy::Color::PowderBlue);
        // Create instance variables.
        std::vector<objectdetectutils::Object> vDetectedObjects;
        objectdetectutils::Object stBestObject;
        std::string szIdentifiedObjects = "";

        // Initialize best percentage to 0 so the first valid object always wins
        double dBestAreaPercentage = 0.0;

        // Get the current time
        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();

        // Load all detected objects in the rover's vision.
        LoadDetectedObjects(vDetectedObjects, vObjectDetectors);

        // Find the best object.
        for (const objectdetectutils::Object& stCandidate : vDetectedObjects)
        {
            // Calculate the total age of the object.
            double dObjectTotalAge = std::fabs(std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - stCandidate.tmCreation).count() / 1000.0);

            // Null pointer safety check for the bounding box
            if (stCandidate.pBoundingBox == nullptr)
            {
                continue;
            }

            // Calculate the total object area and percentage of the screen the object takes up.
            double dArea           = stCandidate.pBoundingBox->area();
            double dAreaPercentage = (dArea / (stCandidate.cvImageResolution.width * stCandidate.cvImageResolution.height)) * 100.0;

            // Determine the desired detection type.
            switch (eDesiredDetectionType)
            {
                case geoops::WaypointType::eMalletWaypoint:
                {
                    if (stCandidate.eDetectionType != objectdetectutils::ObjectDetectionType::eMallet)
                        continue;
                    break;
                }
                case geoops::WaypointType::eWaterBottleWaypoint:
                {
                    if (stCandidate.eDetectionType != objectdetectutils::ObjectDetectionType::eWaterBottle)
                        continue;
                    break;
                }
                case geoops::WaypointType::eRockPickWaypoint:
                {
                    if (stCandidate.eDetectionType != objectdetectutils::ObjectDetectionType::eRockPick)
                        continue;
                    break;
                }
                case geoops::WaypointType::eObjectWaypoint:
                {
                    if (stCandidate.eDetectionType != objectdetectutils::ObjectDetectionType::eMallet &&
                        stCandidate.eDetectionType != objectdetectutils::ObjectDetectionType::eWaterBottle &&
                        stCandidate.eDetectionType != objectdetectutils::ObjectDetectionType::eRockPick)
                    {
                        continue;
                    }
                    break;
                }
                default:
                {
                    break;
                }
            }

            // Check the object detection method type.
            if (stCandidate.eDetectionMethod == objectdetectutils::ObjectDetectionMethod::eTorch)
            {
                // Assemble the identified objects string.
                szIdentifiedObjects += "\tObject Class: " + stCandidate.szClassName + " Object Age: " + std::to_string(dObjectTotalAge) +
                                       "s Object Screen Percentage: " + std::to_string(dAreaPercentage) + "%\n";

                // Check if the object meets the threshold requirements.
                if (dAreaPercentage < constants::BBOX_MIN_SCREEN_PERCENTAGE || dObjectTotalAge < constants::BBOX_MIN_LIFETIME_THRESHOLD)
                {
                    continue;
                }

                // Prioritize the object that takes up the most screen area
                if (dAreaPercentage > dBestAreaPercentage)
                {
                    // Set the target object to the detected object.
                    stBestObject        = stCandidate;
                    dBestAreaPercentage = dAreaPercentage;
                }
            }
        }

        // Only print the identified objects if there are any.
        if (stBestObject.dConfidence != 0.0)
        {
            // Submit logger message.
            LOG_DEBUG(logging::g_qSharedLogger, "ObjectDetectionChecker: Identified objects:\n{}", szIdentifiedObjects);
        }

        // Set the target object to the best object.
        stObjectTarget = stBestObject;

        return static_cast<int>(vDetectedObjects.size());
    }
}    // namespace statemachine
#endif
