/******************************************************************************
 * @brief Defines the Camera base interface class.
 *
 * @file Camera.hpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "../util/IPS.hpp"
#include "../util/vision/FetchContainers.hpp"

#include <future>
#include <shared_mutex>

/******************************************************************************
 * @brief This interface class serves as a base for all other classes that will
 *      implement and interface with a type of camera.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
template<typename T>
class Camera
{
    private:
        // Declare private methods and member variables.

    protected:
        // Declare protected methods and member variables.
        int m_nPropResolutionX;
        int m_nPropResolutionY;
        int m_nPropFramesPerSecond;
        PIXEL_FORMATS m_ePropPixelFormat;
        double m_dPropHorizontalFOV;
        double m_dPropVerticalFOV;

        // Queues and mutexes for scheduling and copying camera frames and data to other threads.
        std::queue<containers::FrameFetchContainer<T>> m_qFrameCopySchedule;
        std::shared_mutex m_muPoolScheduleMutex;
        std::mutex m_muFrameCopyMutex;

        // Declare interface class pure virtual functions. (These must be overriden by inheritor.)
        virtual std::future<bool> RequestFrameCopy(T& tFrame) = 0;    // This is where the code to retrieve an image from the camera is put.
        virtual bool GetCameraIsOpen()                        = 0;    // This is where the code to check if the camera is current open goes.

        // Declare protected object pointers.
        IPS m_IPS = IPS();

    public:
        // Declare public methods and member variables.
        /******************************************************************************
         * @brief Construct a new Camera object.
         *
         * @param nPropResolutionX - X res of camera.
         * @param nPropResolutionY - Y res of camera.
         * @param nPropFramesPerSecond - FPS camera is running at.
         * @param ePropPixelFormat - The pixel layout/format of the image.
         * @param dPropHorizontalFOV - The horizontal field of view.
         * @param dPropVerticalFOV - The vertical field of view.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-08-18
         ******************************************************************************/
        Camera(const int nPropResolutionX,
               const int nPropResolutionY,
               const int nPropFramesPerSecond,
               const PIXEL_FORMATS ePropPixelFormat,
               const double dPropHorizontalFOV,
               const double dPropVerticalFOV)
        {
            // Initialize member variables.
            m_nPropResolutionX     = nPropResolutionX;
            m_nPropResolutionY     = nPropResolutionY;
            m_nPropFramesPerSecond = nPropFramesPerSecond;
            m_ePropPixelFormat     = ePropPixelFormat;
            m_dPropHorizontalFOV   = dPropHorizontalFOV;
            m_dPropVerticalFOV     = dPropVerticalFOV;
        }

        /******************************************************************************
         * @brief Destroy the Camera object.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-08-18
         ******************************************************************************/
        virtual ~Camera() {}

        /******************************************************************************
         * @brief Accessor for the Prop Resolution X private member.
         *
         * @return int - The X resolution of the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-08-19
         ******************************************************************************/
        int GetPropResolutionX() const { return m_nPropResolutionX; }

        /******************************************************************************
         * @brief Accessor for the Prop Resolution Y private member.
         *
         * @return int - The Y resolution of the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-08-19
         ******************************************************************************/
        int GetPropResolutionY() const { return m_nPropResolutionY; }

        /******************************************************************************
         * @brief Accessor for the Prop Frames Per Second private member.
         *
         * @return int - The FPS of the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-08-19
         ******************************************************************************/
        int GetPropFramesPerSecond() const { return m_nPropFramesPerSecond; }

        /******************************************************************************
         * @brief Accessor for the Prop Pixel Format private member.
         *
         * @return PIXEL_FORMATS - The layout/pixel format of the image returned from
         *                      the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-08-19
         ******************************************************************************/
        PIXEL_FORMATS GetPropPixelFormat() const { return m_ePropPixelFormat; }

        /******************************************************************************
         * @brief Accessor for the Prop Horizontal F O V private member.
         *
         * @return double - The horizontal field of view of the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-08-19
         ******************************************************************************/
        double GetPropHorizontalFOV() const { return m_dPropHorizontalFOV; }

        /******************************************************************************
         * @brief Accessor for the Prop Vertical F O V private member.
         *
         * @return double - The vertical field of view of the camera.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-08-19
         ******************************************************************************/
        double GetPropVerticalFOV() const { return m_dPropVerticalFOV; }

        /******************************************************************************
         * @brief Accessor for the Frame I P S private member.
         *
         * @return IPS& - A the camera objects iteration per second counter.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-08-20
         ******************************************************************************/
        virtual IPS& GetIPS() { return m_IPS; }
};
#endif
