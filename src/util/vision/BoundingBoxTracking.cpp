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
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-15
     ******************************************************************************/
    void MultiTracker::AddTracker(const cv::Mat& cvFrame, const std::shared_ptr<cv::Rect2d> cvBoundingBox, const TrackerType eTrackerType)
    {
        // Create instance variables.
        double dBestIOU    = 0.0;
        int nBestTrackerID = -1;

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
            cv::Ptr<cv::Tracker> cvTracker = this->CreateTracker(eTrackerType);
            cvTracker->init(cvFrame, *cvBoundingBox);
            m_mTrackers[nBestTrackerID]       = cvTracker;
            m_mBoundingBoxes[nBestTrackerID]  = cvBoundingBox;
            m_mLastUpdateTime[nBestTrackerID] = std::chrono::steady_clock::now();
        }
        else
        {
            // Create a new tracker and initialize it with the given frame and bounding box.
            cv::Ptr<cv::Tracker> cvTracker = this->CreateTracker(eTrackerType);
            cvTracker->init(cvFrame, *cvBoundingBox);
            // Add the new tracker to the maps.
            m_mTrackers[m_nNextId]       = cvTracker;
            m_mBoundingBoxes[m_nNextId]  = cvBoundingBox;
            m_mLastUpdateTime[m_nNextId] = std::chrono::steady_clock::now();
            m_nNextId++;
        }
    }

    /******************************************************************************
     * @brief Initialize the MultiTracker with a new frame and a vector of bounding boxes.
     *
     * @param cvFrame - The initial frame to track objects in.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-15
     ******************************************************************************/
    void MultiTracker::Update(const cv::Mat& cvFrame)
    {
        // Create instance variables.
        std::vector<int> vToRemove;
        std::chrono::steady_clock::time_point tmCurrentTime = std::chrono::steady_clock::now();

        // Loop through the existing trackers.
        for (const std::pair<const int, cv::Ptr<cv::Tracker>>& stdEntry : m_mTrackers)
        {
            int nID = stdEntry.first;
            cv::Rect cvBoundingBox;
            if (stdEntry.second->update(cvFrame, cvBoundingBox))
            {
                // Update the data of the existing bounding box for the tracker.
                *m_mBoundingBoxes[nID] = cvBoundingBox;
                m_mLastUpdateTime[nID] = tmCurrentTime;
            }
            else
            {
                // Check elapsed time since last successful update.
                double dElapsed = std::chrono::duration<double>(tmCurrentTime - m_mLastUpdateTime[nID]).count();
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

    /******************************************************************************
     * @brief Clear all trackers and bounding boxes from the MultiTracker.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-27
     ******************************************************************************/
    void MultiTracker::ClearTrackers()
    {
        m_mTrackers.clear();
        m_mBoundingBoxes.clear();
        m_mLastUpdateTime.clear();
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
