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

/// \cond

#include <future>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

/// \endcond

// Declare global/file-scope enumerator.
enum PIXEL_FORMATS
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
    eDepthDetection,
    eTensorflowDetection,
    eUNKNOWN
};

///////////////////////////////////////////////////////////////////////////////

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
     * @brief This struct is used to carry references to camera frames for scheduling and copying.
     *      It is constructed so that a reference to a frame is passed into the container and the pointer
     *      to that reference is stored internally. Then a shared_pointer to a new std::promise<bool> is created,
     *      which allows the programmer to return a future from whatever method created this container.
     *      The future can then be waited on before the passed in frame is used. The future will return a true or false.
     *
     *      The idea is that any threads that call a grab/retrieve method of a camera object (which also runs in a different thread)
     *      will have the empty frame that they passed in be put into the camera queues and then the grab/retrieve
     *      method will immediately return. This allows BOTH threads to continue processing other things or request
     *      multiple frame copies non-sequentially/serially.
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
            T* pFrame;
            PIXEL_FORMATS eFrameType;
            std::shared_ptr<std::promise<bool>> pCopiedFrameStatus;

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
            FrameFetchContainer(T& tFrame, PIXEL_FORMATS eFrameType) : pFrame(&tFrame), eFrameType(eFrameType), pCopiedFrameStatus(std::make_shared<std::promise<bool>>())
            {}

            /******************************************************************************
             * @brief Copy Construct a new Frame Fetch Container object.
             *
             * @param stOtherFrameContainer - FrameFetchContainer to copy pointers and values from.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-26
             ******************************************************************************/
            FrameFetchContainer(const FrameFetchContainer& stOtherFrameContainer) :
                pFrame(stOtherFrameContainer.pFrame), eFrameType(stOtherFrameContainer.eFrameType), pCopiedFrameStatus(stOtherFrameContainer.pCopiedFrameStatus)
            {}

            /******************************************************************************
             * @brief Operator equals for FrameFetchContainer. Shallow Copy.
             *
             * @param stOtherFrameContainer - FrameFetchContainer to copy pointers and values from.
             * @return FrameFetchContainer& - A reference to this object.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-26
             ******************************************************************************/
            FrameFetchContainer& operator=(const FrameFetchContainer& stOtherFrameContainer)
            {
                // Check if the passed in container is the same as this one.
                if (this != &stOtherFrameContainer)
                {
                    // Copy struct attributes.
                    this->pFrame             = stOtherFrameContainer.pFrame;
                    this->eFrameType         = stOtherFrameContainer.eFrameType;
                    this->pCopiedFrameStatus = stOtherFrameContainer.pCopiedFrameStatus;
                }

                // Return pointer to this object which now contains the copied values.
                return *this;
            }
    };

    /******************************************************************************
     * @brief This struct is used to carry references to any datatype for scheduling and copying.
     *      It is constructed so that a reference to the data object is passed into the container and the pointer
     *      to that reference is stored internally. Then a shared_pointer to a new std::promise<bool> is created,
     *      which allows the programmer to return a future from whatever method created this container.
     *      The future can then be waited on before the data is used. The future will return a true or false.
     *
     *      The idea is that any threads that call a grab/retrieve method of an object (which also runs in a different thread)
     *      will have the data reference that they passed in be put into the called objects queues and then the grab/retrieve
     *      method will immediately return. This allows BOTH threads to continue processing other things or request
     *      multiple data copies non-sequentially/serially.
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
            T* pData;
            std::shared_ptr<std::promise<bool>> pCopiedDataStatus;

            /******************************************************************************
             * @brief Construct a new Frame Fetch Container object.
             *
             * @param tTata - A reference to the data object to store.
             *
             * @author ClayJay3 (claytonraycowen@gmail.com)
             * @date 2023-09-09
             ******************************************************************************/
            DataFetchContainer(T& tData) : pData(&tData), pCopiedDataStatus(std::make_shared<std::promise<bool>>()) {}

            /******************************************************************************
             * @brief Copy Construct a new Frame Fetch Container object.
             *
             * @param stOtherDataContainer - DataFetchContainer to copy pointers and values from.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-26
             ******************************************************************************/
            DataFetchContainer(const DataFetchContainer& stOtherDataContainer) :
                pData(stOtherDataContainer.pData), pCopiedDataStatus(stOtherDataContainer.pCopiedDataStatus)
            {}

            /******************************************************************************
             * @brief Operator equals for FrameFetchContainer. Shallow Copy.
             *
             * @param stOtherDataContainer - DataFetchContainer to copy pointers and values from.
             * @return DataFetchContainer& - A reference to this object.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-26
             ******************************************************************************/
            DataFetchContainer& operator=(const DataFetchContainer& stOtherDataContainer)
            {
                // Check if the passed in container is the same as this one.
                if (this != &stOtherDataContainer)
                {
                    // Copy struct attributes.
                    this->pData             = stOtherDataContainer.pData;
                    this->pCopiedDataStatus = stOtherDataContainer.pCopiedDataStatus;
                }

                // Return pointer to this object which now contains the copied values.
                return *this;
            }
    };
}    // namespace containers

#endif
