/******************************************************************************
 * @brief Header file for the BoundingBoxTracking class, which is used to track
 * bounding boxes in images using OpenCV.
 *
 * @file BoundingBoxTracking.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-03-14
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef BOUNDING_BOX_TRACKING_HPP
#define BOUNDING_BOX_TRACKING_HPP

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
        private:
    };
}    // namespace tracking

#endif
