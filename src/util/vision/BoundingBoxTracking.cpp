/******************************************************************************
 * @brief Header file for the BoundingBoxTracking class, which is used to track
 * bounding boxes in images using OpenCV.
 *
 * @file BoundingBoxTracking.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-03-14
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "BoundingBoxTracking.h"
#include "../../AutonomyLogging.h"

/// \cond

/// \endcond

/******************************************************************************
 * @brief Namespace containing classes and functions for tracking bounding boxes in images.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-03-14
 ******************************************************************************/
namespace tracking
{
    /******************************************************************************
     * @brief Construct a new Multi Tracker object.
     *
     * @param eTrackerType - The type of tracker to use (default is KCF).
     * @param dTrackingLostThreshold - The time in seconds after which a tracker is considered lost (default is 1.0).
     * @param dIOUThreshold - The minimum Intersection over Union (IoU) required to associate a new detection with an existing tracker (default is 0.3).
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-15
     ******************************************************************************/
    MultiTracker::MultiTracker(const TrackerType eTrackerType, const double dTrackingLostThreshold, const double dIOUThreshold)
    {
        // Initialize member variables.
        m_eTrackerType           = eTrackerType;
        m_dTrackingLostThreshold = dTrackingLostThreshold;
        m_dIOUThreshold          = dIOUThreshold;
        m_nNextId                = 0;

        // Set the tracker factory function based on the specified tracker type.
        m_fnTrackerFactory = [](const TrackerType eType) -> cv::Ptr<cv::Tracker>
        {
            switch (eType)
            {
                case TrackerType::eMIL: return cv::TrackerMIL::create();
                case TrackerType::eKCF: return cv::TrackerKCF::create();
                case TrackerType::eGOTURN: return cv::TrackerGOTURN::create();
                case TrackerType::eCSRT: return cv::TrackerCSRT::create();
                default:
                {
                    // Submit a warning message if the tracker type is unknown.
                    LOG_WARNING(logging::g_qSharedLogger, "Unknown tracker type specified. Defaulting to KCF.");
                    // Return a default tracker (KCF) if the type is unknown.
                    return cv::TrackerKCF::create();
                }
            }
        };
    }

    /******************************************************************************
     * @brief Destroy the Multi Tracker object.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-15
     ******************************************************************************/
    MultiTracker::~MultiTracker()
    {
        // Nothing to do yet.
    }

    /******************************************************************************
     * @brief Add a new tracker for a given frame and bounding box.
     *
     * @param cvFrame - The frame to initialize the tracker with.
     * @param cvBoundingBox - The bounding box (cv::Rect2d) for the object to track.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-15
     ******************************************************************************/
    void MultiTracker::AddTracker(const cv::Mat& cvFrame, const std::shared_ptr<cv::Rect2d> cvBoundingBox)
    {
        // Create a new tracker and initialize it with the given frame and bounding box.
        cv::Ptr<cv::Tracker> cvTracker = m_fnTrackerFactory(m_eTrackerType);
        cvTracker->init(cvFrame, *cvBoundingBox);
        // Add the new tracker to the maps.
        m_mTrackers[m_nNextId]       = cvTracker;
        m_mBoundingBoxes[m_nNextId]  = cvBoundingBox;
        m_mLastUpdateTime[m_nNextId] = std::chrono::steady_clock::now();
        m_nNextId++;
    }

    /******************************************************************************
     * @brief Initialize the MultiTracker with a new frame and a vector of bounding boxes.
     *
     * @param cvFrame - The initial frame to track objects in.
     * @param vDetections - A vector of bounding boxes (cv::Rect2d) representing the initial detections.
     * @param bNewGroundTruthDetections - A flag indicating that the bounding boxes in the given vector should be considered new detections.
     *                              This will treat the given detections as ground truth and will not attempt to match them with existing trackers.
     *                              Instead of matching, it will find the best IOU for each detection and update the corresponding tracker. If no match is found,
     *                              a new tracker will be created for that detection.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-15
     ******************************************************************************/
    void MultiTracker::Update(const cv::Mat& cvFrame, const std::vector<std::shared_ptr<cv::Rect2d>>& vDetections, const bool bNewGroundTruthDetections)
    {
        // Check if the detections are new ground truth detections.
        if (bNewGroundTruthDetections)
        {
            // Clear the existing trackers and bounding boxes.
            m_mTrackers.clear();
            m_mBoundingBoxes.clear();
            m_mLastUpdateTime.clear();
            m_nNextId = 0;

            // Create instance variables.
            std::set<int> sMatchedTrackerIDs;
            std::set<int> sMatchedDetectionsIndices;

            // Loop through the new detections.
            for (size_t siIter = 0; siIter < vDetections.size(); siIter++)
            {
                // Create instance variables.
                double dBestIOU    = 0.0;
                int nBestTrackerID = -1;

                // Loop through the existing trackers to find the best match for the current detection.
                for (std::pair<const int, std::shared_ptr<cv::Rect2d>>& stdEntry : m_mBoundingBoxes)
                {
                    // Skip if this tracker has already been matched with a detection.
                    int nID = stdEntry.first;
                    if (sMatchedTrackerIDs.find(nID) != sMatchedTrackerIDs.end())
                    {
                        continue;
                    }
                    // Calculate the Intersection over Union (IoU) between the current detection and the existing tracker.
                    double dIOU = this->CalculateIOU(*stdEntry.second, *vDetections[siIter]);
                    if (dIOU > dBestIOU)
                    {
                        dBestIOU       = dIOU;
                        nBestTrackerID = nID;
                    }
                }
                // If the best IoU is above the threshold and a tracker was found, update the tracker.
                if (dBestIOU > m_dIOUThreshold && nBestTrackerID != -1)
                {
                    // Reinitialize the tracker with the new detection.
                    cv::Ptr<cv::Tracker> cvTracker = m_fnTrackerFactory(m_eTrackerType);
                    cvTracker->init(cvFrame, *vDetections[siIter]);
                    m_mTrackers[nBestTrackerID]       = cvTracker;
                    m_mBoundingBoxes[nBestTrackerID]  = vDetections[siIter];
                    m_mLastUpdateTime[nBestTrackerID] = std::chrono::steady_clock::now();
                    sMatchedTrackerIDs.insert(nBestTrackerID);
                    sMatchedDetectionsIndices.insert(static_cast<int>(siIter));
                }
            }

            // For any new detection that wasn't associated, add a new tracker.
            for (size_t d = 0; d < vDetections.size(); d++)
            {
                if (sMatchedDetectionsIndices.find(static_cast<int>(d)) == sMatchedDetectionsIndices.end())
                {
                    this->AddTracker(cvFrame, vDetections[d]);
                }
            }
        }
        else
        {
            // Create instance variables.
            std::vector<int> vToRemove;
            std::chrono::steady_clock::time_point tmCurrentTime = std::chrono::steady_clock::now();

            // Loop through the existing trackers.
            for (const std::pair<const int, cv::Ptr<cv::Tracker>>& stdEntry : m_mTrackers)
            {
                int nID = stdEntry.first;
                cv::Rect cvBoundingBox;
                bool bOK = stdEntry.second->update(cvFrame, cvBoundingBox);
                if (bOK)
                {
                    m_mBoundingBoxes[nID]  = std::make_shared<cv::Rect2d>(cvBoundingBox);
                    m_mLastUpdateTime[nID] = tmCurrentTime;
                }
                else
                {
                    // Check elapsed time since last successful update.
                    double dElapsed = std::chrono::duration_cast<std::chrono::duration<double>>(tmCurrentTime - m_mLastUpdateTime[nID]).count();
                    if (dElapsed > m_dTrackingLostThreshold)
                    {
                        vToRemove.push_back(nID);
                    }
                }
            }

            // Remove trackers that have been lost for too long.
            for (int nID : vToRemove)
            {
                m_mTrackers.erase(nID);
                m_mBoundingBoxes.erase(nID);
                m_mLastUpdateTime.erase(nID);
            }
        }
    }

    /******************************************************************************
     * @brief Calculate the Intersection over Union (IoU) of two bounding boxes.
     *
     * @param cvBoxA - The first bounding box (cv::Rect2d).
     * @param cvBoxB - The second bounding box (cv::Rect2d).
     * @return double - The IoU value between the two bounding boxes, ranging from 0.0 to 1.0.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-15
     ******************************************************************************/
    double MultiTracker::CalculateIOU(const cv::Rect2d& cvBoxA, const cv::Rect2d& cvBoxB)
    {
        // Calculate the intersection area.
        double dIntersectionArea = (cvBoxA & cvBoxB).area();
        // Calculate the union area.
        double dUnionArea = cvBoxA.area() + cvBoxB.area() - dIntersectionArea;
        // Return the Intersection over Union (IoU).
        return (dUnionArea > 0) ? (dIntersectionArea / dUnionArea) : 0.0;
    }
}    // namespace tracking
