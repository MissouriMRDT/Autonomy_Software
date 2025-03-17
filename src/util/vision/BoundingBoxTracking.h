/******************************************************************************
 * @brief Header file for the BoundingBoxTracking class, which is used to track
 * bounding boxes in images using OpenCV.
 *
 * @file BoundingBoxTracking.h
 * @author clayjay3
 * @date 2025-03-14
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef BOUNDING_BOX_TRACKING_H
#define BOUNDING_BOX_TRACKING_H

/// \cond
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

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
    // Enum class to define the different types of trackers available in OpenCV.
    enum class TrackerType
    {
        eMIL,
        eKCF,
        eGOTURN,
        eCSRT
    };

    /******************************************************************************
     * @brief Class for tracking multiple bounding boxes in images using OpenCV.
     *  This class supports all OpenCV tracking algorithms and can be used to track
     *  multiple objects in a single image or video stream.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-03-14
     ******************************************************************************/
    class MultiTracker
    {
        public:
            /////////////////////////////////////////
            // Declare public methods.
            /////////////////////////////////////////

            MultiTracker(const TrackerType eTrackerType = TrackerType::eKCF, const double dTrackingLostThreshold = 1.0, const double dIOUThreshold = 0.3);
            ~MultiTracker();
            void UpdateDetections(const cv::Mat& cvFrame, const std::vector<cv::Rect2d>& vDetections);
            void AddTracker(const cv::Mat& cvFrame, const cv::Rect2d& cvBoundingBox);
            std::vector<std::pair<int, cv::Rect2d>> Update(const cv::Mat& cvFrame);

        private:
            /////////////////////////////////////////
            // Declare private methods.
            /////////////////////////////////////////

            double CalculateIOU(const cv::Rect2d& cvBoxA, const cv::Rect2d& cvBoxB);

            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////
            std::map<int, cv::Ptr<cv::Tracker>> m_mTrackers;
            std::map<int, cv::Rect2d> m_mBoundingBoxes;
            std::map<int, std::chrono::steady_clock::time_point> m_mLastUpdateTime;
            std::function<cv::Ptr<cv::Tracker>(TrackerType)> m_fnTrackerFactory;
            TrackerType m_eTrackerType;         // The type of tracker to use (MIL, KCF, GOTURN, CSRT).
            double m_dTrackingLostThreshold;    // Time in seconds after which a tracker is considered lost.
            double m_dIOUThreshold;             // Minimum Intersection over Union required to associate a new detection with an existing tracker.
            int m_nNextId;
    };
}    // namespace tracking

#endif    // BOUNDING_BOX_TRACKING_H
