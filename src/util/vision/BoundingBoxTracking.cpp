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
     * @param dTrackingLostThreshold - The time in seconds after which a tracker is considered lost (default is 1.0).
     * @param dIOUThreshold - The minimum Intersection over Union (IoU) required to associate a new detection with an existing tracker (default is 0.3).
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-15
     ******************************************************************************/
    MultiTracker::MultiTracker(const double dTrackingLostThreshold, const double dIOUThreshold)
    {
        // Initialize member variables.
        m_dTrackingLostThreshold = dTrackingLostThreshold;
        m_dIOUThreshold          = dIOUThreshold;
        m_nNextId                = 0;
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
     * @param eTrackerType - The type of tracker to use (default is KCF).
     * @return true - The given bounding box was matched to an existing tracker and the tracker was updated.
     * @return false - The given bounding box was not matched to an existing tracker and a new tracker was created.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-15
     ******************************************************************************/
    bool MultiTracker::InitTracker(const cv::Mat& cvFrame, const std::shared_ptr<cv::Rect2d> cvBoundingBox, const TrackerType eTrackerType)
    {
        // Create instance variables.
        double dBestIOU         = 0.0;
        int nBestTrackerID      = -1;
        bool bMatchedOldTracker = false;

        // Loop through the existing trackers to find the best match for the given bounding box.
        for (const std::pair<int, std::shared_ptr<cv::Rect2d>>& stdEntry : m_mBoundingBoxes)
        {
            int nID     = stdEntry.first;
            double dIOU = this->CalculateIOU(*stdEntry.second, *cvBoundingBox);
            if (dIOU > dBestIOU)
            {
                dBestIOU       = dIOU;
                nBestTrackerID = nID;
            }
        }

        // If the best IoU is above the threshold and a tracker was found, update the tracker.
        if (dBestIOU > m_dIOUThreshold && nBestTrackerID != -1)
        {
            // Reinitialize the tracker with the new bounding box.
            m_mBoundingBoxes[nBestTrackerID]->x      = cvBoundingBox->x;
            m_mBoundingBoxes[nBestTrackerID]->y      = cvBoundingBox->y;
            m_mBoundingBoxes[nBestTrackerID]->width  = cvBoundingBox->width;
            m_mBoundingBoxes[nBestTrackerID]->height = cvBoundingBox->height;
            cv::Ptr<cv::Tracker> cvTracker           = this->CreateTracker(eTrackerType);
            cvTracker->init(cvFrame, *m_mBoundingBoxes[nBestTrackerID]);
            // Update the last update time for the tracker.
            m_mLastUpdateTime[nBestTrackerID] = std::chrono::system_clock::now();
            // Set the matched tracker flag.
            bMatchedOldTracker = true;
        }
        else
        {
            // Create a new tracker and initialize it with the given frame and bounding box.
            cv::Ptr<cv::Tracker> cvTracker = this->CreateTracker(eTrackerType);
            cvTracker->init(cvFrame, *cvBoundingBox);
            // Add the new tracker to the maps.
            m_mTrackers[m_nNextId]       = cvTracker;
            m_mBoundingBoxes[m_nNextId]  = cvBoundingBox;
            m_mLastUpdateTime[m_nNextId] = std::chrono::system_clock::now();
            m_nNextId++;
        }

        return bMatchedOldTracker;
    }

    /******************************************************************************
     * @brief Initialize the MultiTracker with a new frame and a vector of bounding boxes.
     *
     * @param cvFrame - The initial frame to track objects in.
     *
     * @note If the tracking for a bounding box is lost, it will be set to 0,0,0,0 and removed from the tracker.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-15
     ******************************************************************************/
    void MultiTracker::Update(const cv::Mat& cvFrame)
    {
        // Create instance variables.
        std::vector<int> vToRemove;
        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();

        // Loop through the existing trackers.
        for (const std::pair<const int, cv::Ptr<cv::Tracker>>& stdEntry : m_mTrackers)
        {
            // Create instance variables.
            int nID = stdEntry.first;
            cv::Rect cvBoundingBox;

            // Check if the tracker is valid
            if (!stdEntry.second)
            {
                LOG_WARNING(logging::g_qSharedLogger, "Tracker is null for ID: {}", std::to_string(nID));
                continue;
            }

            // OpenCV loves to throw exceptions, so we need to catch them.
            try
            {
                // Try to update the tracker with the current frame.
                if (stdEntry.second->update(cvFrame, cvBoundingBox))
                {
                    // Update the bounding box
                    m_mBoundingBoxes[nID]->x      = cvBoundingBox.x;
                    m_mBoundingBoxes[nID]->y      = cvBoundingBox.y;
                    m_mBoundingBoxes[nID]->width  = cvBoundingBox.width;
                    m_mBoundingBoxes[nID]->height = cvBoundingBox.height;
                    m_mLastUpdateTime[nID]        = tmCurrentTime;
                }
                else
                {
                    // If the tracker fails to update, we need to check if it has been lost for too long.
                    double dElapsed = std::chrono::duration<double>(tmCurrentTime - m_mLastUpdateTime[nID]).count();
                    if (dElapsed > m_dTrackingLostThreshold)
                    {
                        vToRemove.push_back(nID);
                    }
                }
            }
            catch (const std::exception& stdException)
            {
                // Submit logger message if an exception occurs.
                LOG_ERROR(logging::g_qSharedLogger, "Exception in tracker update for ID: {} Error {}", nID, stdException.what());
                // Remove the problematic tracker.
                vToRemove.push_back(nID);
            }
        }

        // Remove trackers that have been lost for too long.
        for (int nID : vToRemove)
        {
            // Remove the tracker and last update time from the maps.
            m_mTrackers.erase(nID);
            m_mLastUpdateTime.erase(nID);

            // Set the bounding box to 0,0,0,0.
            m_mBoundingBoxes[nID]->x      = 0;
            m_mBoundingBoxes[nID]->y      = 0;
            m_mBoundingBoxes[nID]->width  = 0;
            m_mBoundingBoxes[nID]->height = 0;
            // Remove the bounding box from the map.
            m_mBoundingBoxes.erase(nID);
        }
    }

    /******************************************************************************
     * @brief Clear all trackers and bounding boxes from the MultiTracker.
     *
     * @note This function will set all bounding boxes to 0,0,0,0 and remove all trackers.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-27
     ******************************************************************************/
    void MultiTracker::ClearTrackers()
    {
        m_mTrackers.clear();
        m_mLastUpdateTime.clear();

        // Set the bounding boxes to 0,0,0,0.
        for (const std::pair<int, std::shared_ptr<cv::Rect2d>>& stdEntry : m_mBoundingBoxes)
        {
            stdEntry.second->x      = 0;
            stdEntry.second->y      = 0;
            stdEntry.second->width  = 0;
            stdEntry.second->height = 0;
        }
        // Clear the bounding boxes.
        m_mBoundingBoxes.clear();
    }

    /******************************************************************************
     * @brief Set the timeout for when a tracker is considered lost.
     *
     * @param dTimeout - The time in seconds after which a tracker is considered lost.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-29
     ******************************************************************************/
    void MultiTracker::SetTrackerLostTimeout(const double dTimeout)
    {
        m_dTrackingLostThreshold = dTimeout;
    }

    /******************************************************************************
     * @brief Get the timeout for when a tracker is considered lost.
     *
     * @return double - The time in seconds after which a tracker is considered lost.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-29
     ******************************************************************************/
    double MultiTracker::GetTrackerLostTimeout() const
    {
        return m_dTrackingLostThreshold;
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

    /******************************************************************************
     * @brief Create a tracker based on the specified type.
     *
     * @param eType - The type of tracker to create (eMIL, eKCF, eGOTURN, eCSRT).
     * @return cv::Ptr<cv::Tracker> - A pointer to the created tracker.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-29
     ******************************************************************************/
    cv::Ptr<cv::Tracker> MultiTracker::CreateTracker(const TrackerType eType)
    {
        // Create a tracker based on the specified type.
        switch (eType)
        {
            case TrackerType::eMIL: return cv::TrackerMIL::create(); break;
            case TrackerType::eKCF: return cv::TrackerKCF::create(); break;
            case TrackerType::eGOTURN: return cv::TrackerGOTURN::create(); break;
            case TrackerType::eCSRT: return cv::TrackerCSRT::create(); break;
            default:
                // Submit a warning message if the tracker type is unknown.
                LOG_WARNING(logging::g_qSharedLogger, "Unknown tracker type specified. Defaulting to KCF.");
                // Return a default tracker (KCF) if the type is unknown.
                return cv::TrackerKCF::create();
                break;
        }
    }
}    // namespace tracking
