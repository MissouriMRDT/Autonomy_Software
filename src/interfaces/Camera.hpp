/******************************************************************************
 * @brief Defines the Camera base interface class.
 *
 * @file Camera.hpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

class Camera
{
    private:
        // Declare private methods and member variables.
        int m_nResolutionX      = -1;
        int m_nResolutionY      = -1;
        int m_nFPS              = -1;
        double m_dHorizontalFOV = -1;
        double m_dVeticalFOV    = -1;

    public:
        // Declare public methods and member variables.
        Camera();
        ~Camera();
};
