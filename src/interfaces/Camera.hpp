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

// Declare global enumerator.
enum PIXEL_FORMATS
{
    eRGB,
    eBGR,
    eRGBA,
    eBGRA,
    eARGB,
    eABGR,
    eRGBE,
    eRGBXZY,
    eRGBAXYZ,
    eGrayscale,
    eDepth,
    eCMYK,
    eYUV,
    eYUYV,
    eYUVJ,
    eHSV,
    eHSL,
    eSRGB,
    eLAB,
};

/******************************************************************************
 * @brief This interface class serves as a base for all other classes that will
 *      implement and interface with a type of camera.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
class Camera
{
    private:
        // Declare private methods and member variables.
        int m_nPropResolutionX;
        int m_nPropResolutionY;
        int m_nPropFramesPerSecond;
        int m_nPropPixelFormat;
        double m_dPropHorizontalFOV;
        double m_dPropVerticalFOV;
        // Declare object pointers.
        IPS* m_pIPS;

    public:
        // Declare public methods and member variables.
        /******************************************************************************
         * @brief Construct a new Camera object.
         *
         * @param nPropResolutionX - X res of camera.
         * @param nPropResolutionY - Y res of camera.
         * @param nPropFramesPerSecond - FPS camera is running at.
         * @param nPropPixelFormat - The pixel layout/format of the image.
         * @param dPropHorizontalFOV - The horizontal field of view.
         * @param dPropVerticalFOV - The vertical field of view.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-08-18
         ******************************************************************************/
        Camera(const int nPropResolutionX,
               const int nPropResolutionY,
               const int nPropFramesPerSecond,
               const int nPropPixelFormat,
               const double dPropHorizontalFOV,
               const double dPropVerticalFOV)
        {
            // Initialize member variables.
            m_nPropResolutionX     = nPropResolutionX;
            m_nPropResolutionY     = nPropResolutionY;
            m_nPropFramesPerSecond = nPropFramesPerSecond;
            m_nPropPixelFormat     = nPropPixelFormat;
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
        ~Camera()
        {
            // Nothing to destroy.
        }
};
#endif
