/******************************************************************************
 * @brief
 *
 * @file TimeOperations.hpp
 * @author Eli Byrd (Eli@thebyrdnest.net)
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

namespace timeops
{
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
        siTimeCharacters = std::strftime(cCurrentTime.data(), cCurrentTime.size(), szFormat, tLocalTime);
        if (siTimeCharacters == 0)
        {
            std::cerr << "Unable to format calendar date & time (exceeds string length)" << std::endl;
        }

        return cCurrentTime.data();
    }
}    // namespace timeops

#endif    // TIME_OPERATIONS_HPP