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
#include <cmath>
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
     * @brief This struct represents a point in a 3D coordinate system.
     *
     * @tparam T - The type of the x, y, and z points.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-04-21
     ******************************************************************************/
    template<typename T>
    struct CoordinatePoint
    {
        public:
            // Declare public struct member variables.
            T tX;
            T tY;
            T tZ;

            /******************************************************************************
             * @brief Construct a new Coordinate Point object.
             *
             * @param tX - The X location of the point coordinate.
             * @param tY - The Y location of the point coordinate.
             * @param tZ - The Z location of the point coordinate.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-04-21
             ******************************************************************************/
            CoordinatePoint(const T tX = 0.0, const T tY = 0.0, const T tZ = 0.0)
            {
                // Initialize member variables.
                this->tX = tX;
                this->tY = tY;
                this->tZ = tZ;
            }
    };

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
     * @brief Calculates the modulus of an input angle.
     *
     * @tparam T - Template value specifying the type of the number to find modulus of.
     * @param tValue - Input value to wrap.
     * @param tMinValue - The minimum value expected from the input. INCLUSIVE
     * @param tMaxValue - The maximum value expected from the input. NOT INCLUSIVE
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

        // Check if the final value is the max value, set to min.
        if (tValue == tMaxValue)
        {
            tValue = tMinValue;
        }

        // Return wrapped number.
        return tValue;
    }

    /******************************************************************************
     * @brief Calculates the distance in degrees between two angles. This function
     *      accounts for wrap around so that the most acute or smallest radial distance
     *      between the two points is returned. The distance is positive if going from
     *      the first angle to the second angle results in clockwise motion.
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

    /******************************************************************************
     * @brief This method will rotate a list of 3D coordinate points a variable amount of degrees
     *      around the X, Y, and Z axis in a standard coordinate plane. Any amount of points can be given
     *      and any angle of rotation can be given for each individual X, Y, and Z axis as long as the points
     *      all share the same coordinate system.
     *
     *      The math for this is based off of these website links for a general 3D cartesian coordinate rotation.
     *          - https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions
     *          - https://www.ni.com/docs/en-US/bundle/labview-api-ref/page/vi-lib/analysis/coordinate-llb/3d-cartesian-coordinate-rotation-euler-vi.html
     *
     *      Each X, Y, and Z component of a point are affected by the new rotation around the X, Y, and Z axis in that order.
     *      (Note that any other order of rotations can result in different resultant point locations, so keep that in mind when setting parameters.)
     *      Because each cartesian component is affected by each axis' rotation, we can represent the rotation of the point in terms of the following
     *      three matrices:
     *
     *                      [1      0           0     ]
     *          Rx(theta) = [0  cos(theta) -sin(theta)]
     *                      [0  sin(theta)  cos(theta)]
     *
     *                      [cos(theta) 0   sin(theta)]
     *          Ry(theta) = [0          1       0     ]
     *                      [-sin(theta) 0  cos(theta)]
     *
     *                      [cos(theta) -sin(theta)  0]
     *          Rz(theta) = [sin(theta)  cos(theta)  0]
     *                      [0             0         1]
     *
     *      Finally multiply each point by the determinate of each of these matrices multiplied together to get the new point after being rotated around each axis.
     *
     *      [X`]       [X]
     *      [Y`] = A * [Y]
     *      [Z`]       [Z]
     *
     *      (Where A is Rz(theta) * Ry(theta) * Rx(theta); X, Y, Z are the original points; X`, Y`, Z` are the new points.)
     *
     *      So for example, rotating point [1, 0, 0] 90 degrees around the Z-axis would result in this point:
     *
     *      | 0 -1  0 |   |1|   |0|
     *      | 1  0  0 | * |0| = |1|
     *      | 0  0  1 |   |0|   |0|
     *
     * @tparam T - The data type of the values stored in the CoordinatePoint struct.
     * @param vPointCloud - A reference to a vector of CoordinatePoint<T> structs used to store the X, Y, and Z values of each point. This will contain
     *          the rotated modified points after this function completes.
     * @param dXRotationDegrees - The degree amount to rotate the points around the x-axis.
     * @param dYRotationDegrees - The degree amount to rotate the points around the y-axis.
     * @param dZRotationDegrees - The degree amount to rotate the points around the z-axis.
     *
     * @note Rotation will happen the the order of X-axis, Y-axis, Z-axis with the positive angle direction being clockwise when looking from the origin
     *      and looking down the vector arrow.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-04-21
     ******************************************************************************/
    template<typename T>
    inline constexpr void CoordinateFrameRotate3D(std::vector<CoordinatePoint<T>>& vPointCloud,
                                                  const double dXRotationDegrees,
                                                  const double dYRotationDegrees,
                                                  const double dZRotationDegrees)
    {
        // Convert input degrees to radians.
        double dXRotationRadians = (dXRotationDegrees * M_PI) / 180.0;
        double dYRotationRadians = (dYRotationDegrees * M_PI) / 180.0;
        double dZRotationRadians = (dZRotationDegrees * M_PI) / 180.0;

        // Find the cosine and sine for the X-axis rotation matrix.
        double dCosA = cos(dXRotationRadians);
        double dSinA = sin(dXRotationRadians);
        // Find the cosine and sine for the Y-axis rotation matrix.
        double dCosB = cos(dYRotationRadians);
        double dSinB = sin(dYRotationRadians);
        // Find the cosine and sine for the Z-axis rotation matrix.
        double dCosC = cos(dZRotationRadians);
        double dSinC = sin(dZRotationRadians);

        // Calculate the 3D rotation matrix using matrix multiplication with proper Euler angles Z, Y, X. A = (Rz * Ry * Rx).
        // First row of A matrix.
        double dAXX = dCosC * dCosB;
        double dAXY = dCosC * dSinB * dSinA - dSinC * dCosA;
        double dAXZ = dCosC * dSinB * dCosA + dSinC * dSinA;
        // Second row of A matrix.
        double dAYX = dSinC * dCosB;
        double dAYY = dSinC * dSinB * dSinA + dCosC * dCosA;
        double dAYZ = dSinC * dSinB * dCosA - dCosC * dSinA;
        // Third row of A matrix.
        double dAZX = -dSinB;
        double dAZY = dCosB * dSinA;
        double dAZZ = dCosB * dCosA;

        // Loop through each point in the given point cloud.
        for (CoordinatePoint<T>& stPoint : vPointCloud)
        {
            // Rotate the points in X, Y, Z order using the rotation matrix A.
            T tX = static_cast<T>((dAXX * stPoint.tX) + (dAXY * stPoint.tY) + (dAXZ * stPoint.tZ));
            T tY = static_cast<T>((dAYX * stPoint.tX) + (dAYY * stPoint.tY) + (dAYZ * stPoint.tZ));
            T tZ = static_cast<T>((dAZX * stPoint.tX) + (dAZY * stPoint.tY) + (dAZZ * stPoint.tZ));
            // Update point cloud.
            stPoint.tX = tX;
            stPoint.tY = tY;
            stPoint.tZ = tZ;
        }
    }
}    // namespace numops
#endif
