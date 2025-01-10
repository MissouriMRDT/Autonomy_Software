/******************************************************************************
 * @brief Defines and implements functions related to operations on time and
 *        date within the timeops namespace.
 *
 * @file TimeOperations.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-07
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef TIME_OPERATIONS_HPP
#define TIME_OPERATIONS_HPP

/// \cond
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

/// \endcond

/******************************************************************************
 * @brief Namespace containing functions related to operations on time and
 *        date related data types.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-07
 ******************************************************************************/
namespace timeops
{
    /******************************************************************************
     * @brief Accessor for getting the current time in a specified format.
     *
     * @param szFormat - The format to return the time in.
     * @return std::string - The current time in the specified format.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2025-01-07
     ******************************************************************************/
    inline std::string GetTimestamp(const std::string& szFormat = "%Y%m%d-%H%M%S")
    {
        // Retrieve the current time
        auto now       = std::chrono::system_clock::now();
        time_t timeNow = std::chrono::system_clock::to_time_t(now);

        // Convert to local time (thread-safe)
        tm localTime{};
        localtime_r(&timeNow, &localTime);

        // Format the time
        std::ostringstream oss;
        oss << std::put_time(&localTime, szFormat.c_str());
        return oss.str();
    }
}    // namespace timeops

#endif    // TIME_OPERATIONS_HPP
