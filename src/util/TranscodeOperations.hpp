/******************************************************************************
 * @brief Defines and implements functions related to transcoding of bytes within
 * 		the transops namespace.
 *
 * @file TranscodeOperations.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-11-12
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef TRANSCODING_OPERATIONS_HPP
#define TRANSCODING_OPERATIONS_HPP

/// \cond
#include <codecvt>
#include <locale>
#include <string>

/// \endcond

/******************************************************************************
 * @brief Namespace containing functions related to operations on binary or string
 *      data that needs to be encoded or decoded.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-11-12
 ******************************************************************************/
namespace transops
{
    /******************************************************************************
     * @brief Decodes a UTF-8 encoded string into a standard human-readable string.
     *
     * @param szEncodedString - The UTF-8 encoded string to decode.
     * @return std::string - The decoded string suitable for console output.
     *
     * @note This function will only work with string formatted in UTF-8.
     *      For example, a string that is formatted as "PixelStreaming".
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-11-12
     ******************************************************************************/
    inline std::string DecodeUTF8EncodedString(const std::string& szEncodedString)
    {
        // Step 1: Use a wstring_convert to convert the UTF-8 string to a wide string.
        std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
        std::wstring wideString = converter.from_bytes(szEncodedString);

        // Step 2: Convert the wide string back to a normal string.
        std::string decodedString(wideString.begin(), wideString.end());

        return decodedString;
    }
}    // namespace transops
#endif
