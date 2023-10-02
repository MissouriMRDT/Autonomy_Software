/******************************************************************************
 * @brief Defines and implements functions related to operations on numbers within
 * 		the NumberOperations namespace.
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
     * @brief Maps a value to a new range given the old range.
     *
     * @tparam T - Template value specifying the type of the number to map to new range.
     * @param tValue - The value to remap.
     * @param tOldMinimum - The current range's minimum value.
     * @param tOldMaximum - The current range's maximim value.
     * @param tNewMinimum - The new range's minimim value.
     * @param tNewMaximum - The new range's maximum value.
     * @return T - The resultant templated type mapped to the new range.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-22
     ******************************************************************************/
    template<typename T>
    T MapRange(const T tValue, const T tOldMinimum, const T tOldMaximum, const T tNewMinimum, const T tNewMaximum)
    {
        // Check if the ranges are valid.
        if (tOldMinimum == tOldMaximum || tNewMinimum == tNewMaximum)
        {
            // Submit logger message.
            std::cerr << "MAPRANGE: The old/new range is not valid." << std::endl;

            // Return old, given value.
            return tValue;
        }

        // Perform the mapping using linear interpolation.
        T tOldValueRange = tOldMaximum - tOldMinimum;
        T tNewValueRange = tNewMaximum - tNewMinimum;
        T tScaledValue   = (tValue - tOldMinimum) / tOldValueRange;

        // Return new mapped value.
        return tNewMinimum + tScaledValue * tNewValueRange;
    }

}    // namespace numops
#endif
