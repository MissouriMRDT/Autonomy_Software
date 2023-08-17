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
        int m_nResolutionX         = -1;
        int m_nResolutionY         = -1;
        int m_nPropFramesPerSecond = -1;
        double m_dHorizontalFOV    = -1;
        double m_dVeticalFOV       = -1;

    public:
        // Declare public methods and member variables.
        Camera();
        ~Camera();
};
#endif
