/******************************************************************************
 * @brief Defines the PIXEL_FORMATS enumerator used throughout the vision code.
 *
 *      This header formerly also defined containers::FrameFetchContainer and
 *      containers::DataFetchContainer, which carried a destination pointer plus a
 *      shared_ptr<std::promise<bool>> so a request could be queued to a camera or
 *      detector thread and waited on. That request/queue/promise fan-out has been
 *      replaced by the publish-latest pubsub::Publisher channels, so those
 *      containers (and their queues/mutexes) are gone. Only the pixel-format
 *      enumerator remains.
 *
 * @file FetchContainers.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-16
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef FETCH_CONTAINERS_HPP
#define FETCH_CONTAINERS_HPP

/// \cond
// These includes are retained because several vision headers include FetchContainers.hpp and
// rely on it to transitively provide the OpenCV and Stereolabs types alongside PIXEL_FORMATS.
#include <future>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

/// \endcond

// Declare global/file-scope enumerator.
enum class PIXEL_FORMATS
{
    eRGB,
    eBGR,
    eRGBA,
    eBGRA,
    eARGB,
    eABGR,
    eRGBE,
    eXYZ,
    eXYZBGRA,
    eXYZRGBA,
    eZED,
    eGrayscale,
    eDepthImage,
    eDepthMeasure,
    eCMYK,
    eYUV,
    eYUYV,
    eYUVJ,
    eHSV,
    eHSL,
    eSRGB,
    eLAB,
    eArucoDetection,
    eObjectDetection,
    eObstacleDetection,
    eDepthDetection,
    eTorchDetection,
    eUNKNOWN
};

#endif
