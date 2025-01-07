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
#include <iostream>

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
    inline std::string GetTimestamp(std::string szFormat = "%Y%m%d-%H%M%S")
    {
        // Retrieve the current time for the log file name
        std::chrono::time_point<std::chrono::system_clock> tmCurrentTime = std::chrono::system_clock::now();
        std::time_t tCurrentTime                                         = std::chrono::system_clock::to_time_t(tmCurrentTime);

        // Convert time to local time
        std::tm* tLocalTime = std::localtime(&tCurrentTime);

        // Format the current time in a format that can be used as a file name.
        std::array<char, 80> cCurrentTime;
        size_t siTimeCharacters;
        siTimeCharacters = std::strftime(cCurrentTime.data(), cCurrentTime.size(), szFormat.c_str(), tLocalTime);
        if (siTimeCharacters == 0)
        {
            // Check if Quill has been initialized. If so, log the error. Otherwise, print to stderr.
            if (logging::g_qSharedLogger != nullptr)
            {
                LOG_CRITICAL(logging::g_qSharedLogger, "Unable to format calendar date & time (exceeds string length)");
            }
            else
            {
                std::cerr << "Unable to format calendar date & time (exceeds string length)" << std::endl;
            }
        }

        return cCurrentTime.data();
    }
}    // namespace timeops

#endif    // TIME_OPERATIONS_HPP
