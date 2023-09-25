/******************************************************************************
 * @brief Defines and implements functions related to operations on numbers within
 * 		the numops namespace.
 *
 * @file NumberOperations.hpp
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef NUMBER_OPERATIONS_HPP
#define NUMBER_OPERATIONS_HPP

#include <algorithm>

/******************************************************************************
 * @brief Namespace containing functions related to operations on numbers and
 * 		other datatypes.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-06
 ******************************************************************************/
namespace numops
{
    /******************************************************************************
     * @brief Clamps a given value from going above or below a given threshold.
     *
     * @tparam T - Template argument for given value type.
     * @param tValue - The value to clamp.
     * @param tMin - Minimum value quantity.
     * @param tMax - Maximum value quantity.
     * @return T - The clamped value.
     *
     * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
     * @date 2023-06-20
     ******************************************************************************/
    template<typename T>
    T Clamp(T tValue, T tMin, T tMax)
    {
        return std::max(std::min(tMax, tValue), tMin);
    }
}    // namespace numops

#endif    // NUMBER_OPERATIONS_H
