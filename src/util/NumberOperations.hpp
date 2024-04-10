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

/// \cond
#include <algorithm>
#include <iostream>

/// \endcond

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
     * @return constexpr T - The clamped value.
     *
     * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
     * @date 2023-06-20
     ******************************************************************************/
    template<typename T>
    inline constexpr T Clamp(T tValue, T tMin, T tMax)
    {
        return std::max(std::min(tMax, tValue), tMin);
    }

    /******************************************************************************
     * @brief Checks if a given value is between the given maximum and minimum ranges.
     *
     * @tparam T - Template argument for given value type.
     * @param tValue - The value to check.
     * @param tMin - The minimum bound for the value to be valid.
     * @param tMax - The maximum bound for the value to be valid.
     * @return true - The value is within the bounds.
     * @return false - The value is not within the bounds.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-16
     ******************************************************************************/
    template<typename T>
    inline bool Bounded(T tValue, T tMin, T tMax, const bool bInclusive = true)
    {
        // Check if value is inclusive or not.
        if (bInclusive)
        {
            // Return true if the given value is valid.
            return (tValue >= tMin) && (tValue <= tMax);
        }
        else
        {
            // Return true if the given value is valid.
            return (tValue > tMin) && (tValue < tMax);
        }
    }

    /******************************************************************************
     * @brief Maps a value to a new range given the old range.
     *
     * @tparam T - Template value specifying the type of the number to map to new range.
     * @param tValue - The value to remap.
     * @param tOldMinimum - The current range's minimum value.
     * @param tOldMaximum - The current range's maximum value.
     * @param tNewMinimum - The new range's minimum value.
     * @param tNewMaximum - The new range's maximum value.
     * @return constexpr T - The resultant templated type mapped to the new range.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-22
     ******************************************************************************/
    template<typename T>
    inline constexpr T MapRange(const T tValue, const T tOldMinimum, const T tOldMaximum, const T tNewMinimum, const T tNewMaximum)
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

    /******************************************************************************
     * @brief Calculates the modulus of an input angle to -180, 180
     *
     * @tparam T - Template value specifying the type of the number to find modulus of.
     * @param tValue - Input value to wrap.
     * @param tMinValue - The minimum value expected from the input.
     * @param tMaxValue - The maximum value expected from the input.
     * @return constexpr T - The wrapped value.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-19
     ******************************************************************************/
    template<typename T>
    inline constexpr T InputAngleModulus(T tValue, T tMinValue, T tMaxValue)
    {
        // Determine the correct modulus number.
        T tModulus = tMaxValue - tMinValue;

        // Wrap input if it's above the maximum input.
        int nNumMax = (tValue - tMinValue) / tModulus;
        tValue -= nNumMax * tModulus;
        // Wrap input if it's below the minimum input.
        int nNumMin = (tValue - tMaxValue) / tModulus;
        tValue -= nNumMin * tModulus;

        // Return wrapped number.
        return tValue;
    }

    /******************************************************************************
     * @brief Calculates the distance in degrees between two angles. This function
     *      accounts for wrap around so that the most acute or smallest radial distance
     *      between the two points is returned.
     *
     * @tparam T - Template value specifying the type of the number to find difference of.
     * @param tFirstValue - The first value.
     * @param tSecondValue - The second value. This will be subtracted from the first value.
     * @return constexpr T - The smallest angular distance.
     *
     * @note This function expects the two values to be between 0-360.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-04-03
     ******************************************************************************/
    template<typename T>
    inline constexpr T AngularDifference(T tFirstValue, T tSecondValue)
    {
        // Check input.
        if (!Bounded<T>(tFirstValue, 0, 360) && !Bounded<T>(tSecondValue, 0, 360))
        {
            // Submit logger message.
            std::cerr << "ANGULARDIFFERENCE: An input value is not valid must be between 0-360. The result difference will not be accurate!" << std::endl;
        }

        // Find absolute difference between the two values.
        T tDifference = std::abs(tFirstValue - tSecondValue);
        // If greater than 180 degrees, subtract 360/
        if (tDifference > 180)
        {
            // Wrap value.
            tDifference -= 360;

            // Check if first values is bigger than second value. If it is, flip sign so that clockwise is positive.
            if (tFirstValue > tSecondValue)
            {
                tDifference *= -1;
            }
        }

        // Return value.
        return tDifference;
    }
}    // namespace numops
#endif
