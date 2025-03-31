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
        eMIL,       // Multi Instance Learning
        eKCF,       // Kernelized Correlation Filter
        eGOTURN,    // Generic Object Tracking Using Regression Networks
        eCSRT       // Discriminative Correlation Filter with Channel and Spatial Reliability
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

            MultiTracker(const double dTrackingLostTimeout = 1.0, const double dIOUThreshold = 0.3);
            ~MultiTracker();
            void AddTracker(const cv::Mat& cvFrame, const std::shared_ptr<cv::Rect2d> cvBoundingBox, const TrackerType eTrackerType = TrackerType::eKCF);
            void Update(const cv::Mat& cvFrame);
            void ClearTrackers();

            /////////////////////////////////////////
            // Setters.
            /////////////////////////////////////////
            void SetTrackerLostTimeout(const double dTimeout);

            /////////////////////////////////////////
            // Getters.
            /////////////////////////////////////////
            double GetTrackerLostTimeout() const;

        private:
            /////////////////////////////////////////
            // Declare private methods.
            /////////////////////////////////////////

            double CalculateIOU(const cv::Rect2d& cvBoxA, const cv::Rect2d& cvBoxB);
            cv::Ptr<cv::Tracker> CreateTracker(const TrackerType eType);

            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////
            std::map<int, cv::Ptr<cv::Tracker>> m_mTrackers;
            std::map<int, std::shared_ptr<cv::Rect2d>> m_mBoundingBoxes;
            std::map<int, std::chrono::steady_clock::time_point> m_mLastUpdateTime;
            double m_dTrackingLostThreshold;    // Time in seconds after which a tracker is considered lost.
            double m_dIOUThreshold;             // Minimum Intersection over Union required to associate a new detection with an existing tracker.
            int m_nNextId;
    };
}    // namespace tracking

#endif    // BOUNDING_BOX_TRACKING_H
