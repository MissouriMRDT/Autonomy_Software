/******************************************************************************
 * @brief Defines and implements objects/structs related to storing data types
 *      that will be passed and/or queued between threads. All objects should
 *      provide the proper mutexes and condition variables to aid in proper and
 *      fast resource management between threads.
 *
 * @file FetchContainers.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-16
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef FETCH_CONTAINERS_HPP
#define FETCH_CONTAINERS_HPP

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

/******************************************************************************
 * @brief Namespace containing functions or objects/struct used to aid in data
 *      storage and passage between threads.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-16
 ******************************************************************************/
namespace containers
{
    /******************************************************************************
     * @brief This struct is used to contain camera frames for scheduling and copying.
     *      It also stores a condition variable and mutex for signaling when the copy
     *      has been completed.
     *
     * @tparam T - The mat type that the struct will be containing.
     *
     * @author ClayJay3 (claytonraycowen@gmail.com)
     * @date 2023-09-08
     ******************************************************************************/
    template<typename T>
    struct FrameFetchContainer
    {
        public:
            // Declare and define public struct member variables.
            std::condition_variable cdMatWriteSuccess;
            std::mutex muConditionMutex;
            T& tFrame;
            PIXEL_FORMATS eFrameType;

            /******************************************************************************
             * @brief Construct a new Frame Fetch Container object.
             *
             * @param tFrame - A reference to the frame object to store.
             * @param eFrameType - The image or measure type to store in the frame. This
             *                  is used to determine what is copied to the given frame object.
             *
             * @author ClayJay3 (claytonraycowen@gmail.com)
             * @date 2023-09-09
             ******************************************************************************/
            FrameFetchContainer(T& tFrame, PIXEL_FORMATS eFrameType) : tFrame(tFrame), eFrameType(eFrameType) {}
    };

    /******************************************************************************
     * @brief The struct is used to contain any datatype for scheduling and copying.
     *      It also stores a condition variable and mutex for signaling when the copy
     *      has been completed.
     *
     * @tparam T - The type of data object that this struct will be containing.
     *
     * @author ClayJay3 (claytonraycowen@gmail.com)
     * @date 2023-09-10
     ******************************************************************************/
    template<typename T>
    struct DataFetchContainer
    {
        public:
            // Declare and define public struct member variables.
            std::condition_variable cdMatWriteSuccess;
            std::mutex muConditionMutex;
            T& tData;

            /******************************************************************************
             * @brief Construct a new Frame Fetch Container object.
             *
             * @param tTata - A reference to the data object to store.
             *
             * @author ClayJay3 (claytonraycowen@gmail.com)
             * @date 2023-09-09
             ******************************************************************************/
            DataFetchContainer(T& tData) : tData(tData) {}
    };
}    // namespace containers

#endif
