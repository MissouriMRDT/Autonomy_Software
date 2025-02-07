/******************************************************************************
 * @brief Implements Logging for Autonomy
 *
 *        Note: Functions and Variables are defined in AutonomyGlobals.cpp
 *              by having the declarations occur in a separate header we are
 *              more easily able to use the functionality of the logger in
 *              areas of the program that would normally be unaccessible
 *              due to it being included in the Main Globals Header.
 *
 * @file AutonomyLogging.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-08-22
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

/// \cond
#include <quill/Backend.h>
#include <quill/Frontend.h>
#include <quill/LogMacros.h>
#include <quill/Logger.h>

#include "quill/backend/PatternFormatter.h"
#include "quill/core/Attributes.h"
#include "quill/core/Common.h"
#include "quill/core/Filesystem.h"

#include "quill/sinks/ConsoleSink.h"
#include "quill/sinks/RotatingFileSink.h"

#include <RoveComm/RoveComm.h>
#include <RoveComm/RoveCommManifest.h>
#include <atomic>
#include <shared_mutex>

/// \endcond

#include "./AutonomyConstants.h"
#include "./util/TimeOperations.hpp"

#ifndef AUTONOMY_LOGGING_H
#define AUTONOMY_LOGGING_H

/******************************************************************************
 * @brief Logging Levels:
 *
 *        Priority > Level     > Description
 *        Level 1  > TRACE_L3  > Unused
 *        Level 2  > TRACE_L2  > Unused
 *        Level 3  > TRACE_L1  > Unused
 *        Level 4  > DEBUG     > Details that would only be useful in a debug environment
 *        Level 5  > INFO      > State Changes, RoveComm Updates GPS/IMU/Autonomy, etc
 *        Level 6  > WARNING   > Something unexpected happened - application could potentially error soon.
 *        Level 7  > ERROR     > Something went wrong - application could potentially have critical error soon.
 *        Level 8  > CRITICAL  > Something went very wrong - application will exit after logging is sent.
 *
 *        Note: At testing sessions we will have "DEBUG" Logging set as the level that is being outputted.
 *              However, at competition and when using "RELEASE" code we will be using "INFO" Logging. When
 *              a logging level is set, we only receive logging messages that are that level or higher
 *              priority.
 *
 *        Example: When INFO is set, we only receive: INFO, WARNING, ERROR, CRITICAL
 *                 When DEBUG is set, we only receive: DEBUG, INFO, WARNING, ERROR, CRITICAL
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-08-22
 ******************************************************************************/
namespace logging
{
    //////////////////////////////////////////
    // Declare namespace external variables and objects.
    /////////////////////////////////////////

    extern quill::Logger* g_qFileLogger;
    extern quill::Logger* g_qConsoleLogger;
    extern quill::Logger* g_qSharedLogger;
    extern quill::Logger* g_qRoveCommLogger;

    extern quill::LogLevel g_eConsoleLogLevel;
    extern quill::LogLevel g_eFileLogLevel;
    extern quill::LogLevel g_eRoveCommLogLevel;

    extern std::string g_szProgramStartTimeString;

    /////////////////////////////////////////
    // Declare namespace methods.
    /////////////////////////////////////////

    void InitializeLoggers(std::string szLoggingOutputPath, std::string szProgramTimeLogsDir = timeops::GetTimestamp());

    /////////////////////////////////////////
    // Declare namespace callbacks.
    /////////////////////////////////////////

    const std::function<void(const rovecomm::RoveCommPacket<uint8_t>&, const sockaddr_in&)> SetLoggingLevelsCallback =
        [](const rovecomm::RoveCommPacket<uint8_t>& stPacket, const sockaddr_in& stdAddr)
    {
        // Not using this.
        (void) stdAddr;

        // Convert Minimum Permitted Console Level to Integer Value
        const int nMinConsoleLevel  = static_cast<int>(constants::CONSOLE_MIN_LEVEL);
        const int nMinFileLevel     = static_cast<int>(constants::FILE_MIN_LEVEL);
        const int nMinRoveCommLevel = static_cast<int>(constants::ROVECOMM_MIN_LEVEL);

        // Convert Requested Console Level to Integer Value
        const int nRequestedConsoleLevel  = stPacket.vData[0];
        const int nRequestedFileLevel     = stPacket.vData[1];
        const int nRequestedRoveCommLevel = stPacket.vData[2];

        // Determine if change is allowed
        bool bConsoleLevelChangePermitted  = nRequestedConsoleLevel >= nMinConsoleLevel;
        bool bFileLevelChangePermitted     = nRequestedFileLevel >= nMinFileLevel;
        bool bRoveCommLevelChangePermitted = nRequestedRoveCommLevel >= nMinRoveCommLevel;

        // Convert RoveComm Enumeration to Quill Enumeration and store to logging globals if permitted
        logging::g_eConsoleLogLevel  = bConsoleLevelChangePermitted ? static_cast<quill::LogLevel>(stPacket.vData[0]) : logging::g_eConsoleLogLevel;
        logging::g_eFileLogLevel     = bFileLevelChangePermitted ? static_cast<quill::LogLevel>(stPacket.vData[1]) : logging::g_eFileLogLevel;
        logging::g_eRoveCommLogLevel = bRoveCommLevelChangePermitted ? static_cast<quill::LogLevel>(stPacket.vData[2]) : logging::g_eRoveCommLogLevel;

        // Submit logger message.
        LOG_INFO(logging::g_qSharedLogger, "Incoming SETLOGGINGLEVELS: [Console: {}, File: {}, RoveComm: {}]", stPacket.vData[0], stPacket.vData[1], stPacket.vData[2]);
    };

    /////////////////////////////////////////
    // Define namespace file filters.
    /////////////////////////////////////////

    /******************************************************************************
     * @brief This class serves as a container class for handling log filtering of
     *    loggers. This must be used if you want each handler to have a different
     *    logging level since adding multiple handlers to the same logger will apply the
     *    loggers logging level to each handler.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-03-16
     ******************************************************************************/
    class LoggingFilter : public quill::Filter
    {
        private:
            // Declare private member variables.
            quill::LogLevel m_eMinLogLevel;

        public:
            /******************************************************************************
             * @brief Construct a new Console Filter object.
             *
             * @param eMinLogLevel - The minimum acceptable log level for the console handler.
             *      All log levels above this will also be logged.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-03-16
             ******************************************************************************/
            LoggingFilter(const std::string szFilterBaseType, const quill::LogLevel eMinLogLevel) : quill::Filter(szFilterBaseType)
            {
                // Set member variables.
                m_eMinLogLevel = eMinLogLevel;
            };

            /******************************************************************************
             * @brief This method should never be called by this codebase, it is called internally
             *      by the quill library. This is used to filter log messages based on the log level.
             *      The individual sinks use this method to determine if a log message should be
             *      logged if a logger (ie SharedLogger) has multiple sinks.
             *
             * @param qLogMetadata - The metadata of the log statement.
             * @param unLogTimestamp - The timestamp of the log statement.
             * @param szThreadID - The ID of the thread that generated the log statement.
             * @param szThreadName - The name of the thread that generated the log statement.
             * @param szLoggerName - The name of the logger that generated the log statement.
             * @param qLogLevel - The log level of the log statement.
             * @param szLogMessage - The message of the log statement.
             * @param szLogStatement - The full log statement.
             * @return * QUILL_NODISCARD - Whether or not the log message should be logged.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2025-02-07
             ******************************************************************************/
            QUILL_NODISCARD bool filter(const quill::MacroMetadata* qLogMetadata,
                                        uint64_t unLogTimestamp,
                                        std::string_view szThreadID,
                                        std::string_view szThreadName,
                                        std::string_view szLoggerName,
                                        quill::LogLevel qLogLevel,
                                        std::string_view szLogMessage,
                                        std::string_view szLogStatement) noexcept override
            {
                // Not using these.
                (void) qLogMetadata;
                (void) unLogTimestamp;
                (void) szThreadID;
                (void) szThreadName;
                (void) szLoggerName;
                (void) szLogMessage;
                (void) szLogStatement;

                // Log only m_eMinLogLevel or higher to stdout.
                return qLogLevel >= m_eMinLogLevel;
            }
    };

    /////////////////////////////////////////
    // Define namespace custom sinks
    /////////////////////////////////////////

    /******************************************************************************
     * @brief A custom console sink for logging messages with specific formatting
     *        and timestamping. This class extends `quill::ConsoleSink` and provides
     *        the capability to format log messages using a specified pattern and
     *        time format, allowing for customizable and colorized console outputs.
     *
     * This class is intended to be used in scenarios where real-time logging to the
     * console is required, such as during development, testing, and production. It
     * supports different formatting options, including log level descriptions,
     * thread information, timestamps, and log messages.
     *
     * ### Key Features:
     * - Customizable log message formats using a pattern.
     * - Colorized console output to highlight log levels.
     * - Supports different timestamp formats and timezones.
     * - Inherits from `quill::ConsoleSink` for seamless integration with Quill's
     *   logging framework.
     *
     * @note This class should be used for logging real-time information to the console.
     *       It is flexible enough to handle different environments, including testing
     *       and production, by adjusting the formatting and log levels.
     *
     * @see quill::ConsoleSink
     * @see quill::PatternFormatter
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-08-16
     ******************************************************************************/
    class MRDTConsoleSink : public quill::ConsoleSink
    {
        public:
            /******************************************************************************
             * @brief Constructs a new MRDTConsoleSink object with specified formatting and
             *        console colors. This constructor initializes the sink with a log
             *        message pattern, timestamp format, and optional timezone settings.
             *        The constructor also allows customization of the output szStream.
             *
             * @param qColors - The console colors configuration for highlighting log levels.
             * @param szFormatPattern - The pattern used to format the log message.
             * @param szTimeFormat - The format of the timestamp in the log message.
             * @param qTimestampTimezone - The timezone used for the timestamp (default: LocalTime).
             * @param szStream - The szStream to output the logs to (default: "stdout").
             *
             * @note Ensure that the format pattern and time format are properly defined before
             *       using this constructor, as they directly affect the log output structure.
             *
             * @warning Incorrect configuration of format patterns or time formats may lead to
             *          malformed log outputs.
             *
             * @see quill::ConsoleSink
             * @see quill::PatternFormatter
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-08-16
             ******************************************************************************/
            MRDTConsoleSink(const quill::ConsoleSink::Colours& qColors,                                          // Custom Colors Import
                            const quill::ConsoleSink::ColourMode& qColorMode,
                            const std::string& szFormatPattern,                                                  // Custom Format Pattern
                            const std::string& szTimeFormat,                                                     // Custom Time Format
                            quill::Timezone qTimestampTimezone = quill::Timezone::LocalTime,                     // Timezone
                            const std::string& szStream        = "stdout"                                        // Stream
                            ) :
                quill::ConsoleSink(qColors, qColorMode, szStream),                                               // Pass Parameters into quill::ConsoleSink
                qFormatter(quill::PatternFormatterOptions(szFormatPattern, szTimeFormat, qTimestampTimezone))    // Pass Parameters into qFormatter type
            {}

            void write_log(const quill::MacroMetadata* qLogMetadata,
                           uint64_t unLogTimestamp,
                           std::string_view szThreadID,
                           std::string_view szThreadName,
                           const std::string& szProcessID,
                           std::string_view szLoggerName,
                           quill::LogLevel qLogLevel,
                           std::string_view szLogLevelDescription,
                           std::string_view szLogLevelShortCode,
                           const std::vector<std::pair<std::string, std::string>>* vNamedArgs,
                           std::string_view szLogMessage,
                           std::string_view) override;

        private:
            quill::PatternFormatter qFormatter;
    };

    /******************************************************************************
     * @brief A custom rotating file sink that formats and logs messages to a file
     *        with automatic rotation based on file size or time interval. This class
     *        extends `quill::RotatingFileSink` and provides the ability to format
     *        log messages using a pattern and time format, ensuring that logs are
     *        written to a rotating file system.
     *
     * This class is ideal for scenarios where log files need to be managed based on
     * size or time constraints, such as in long-running applications. It formats the
     * log messages before writing them to files, making it easy to store logs with
     * consistent formatting while preventing file bloat through rotation.
     *
     * ### Key Features:
     * - Customizable log message formats using a pattern.
     * - Automatic file rotation based on configurable size or time intervals.
     * - Supports different timestamp formats and timezones for log entries.
     * - Inherits from `quill::RotatingFileSink` for easy integration with Quill's
     *   logging framework.
     *
     * @note This class is designed to handle file-based logging with automatic rotation,
     *       making it suitable for use in both development and production environments.
     *
     * @see quill::RotatingFileSink
     * @see quill::PatternFormatter
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-08-16
     ******************************************************************************/
    class MRDTRotatingFileSink : public quill::RotatingFileSink
    {
        public:
            /******************************************************************************
             * @brief Constructs a new MRDTRotatingFileSink object with specified formatting,
             *        file rotation settings, and an optional file event notifier. This constructor
             *        initializes the sink with a log message pattern, timestamp format, and
             *        configuration for rotating the log file based on size or time interval.
             *
             * @param qFilename - The path to the log file.
             * @param qConfig - The configuration for rotating the log file (e.g., based on size or time).
             * @param szFormatPattern - The pattern used to format the log message.
             * @param szTimeFormat - The format of the timestamp in the log message.
             * @param qTimestampTimezone - The timezone used for the timestamp (default: LocalTime).
             * @param qFileEventNotifier - Optional event notifier for file-related events (default: none).
             *
             * @note Ensure that the file rotation configuration (`config`) is correctly set up to avoid
             *       unexpected log file behavior. The format pattern and time format should also be defined
             *       correctly to ensure logs are written with the intended structure.
             *
             * @warning Misconfiguration of file rotation settings or format patterns may result in loss of log data or malformed log outputs.
             *
             * @see quill::RotatingFileSink
             * @see quill::PatternFormatter
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-08-16
             ******************************************************************************/
            MRDTRotatingFileSink(const quill::fs::path& qFilename,                                               // File Path
                                 const quill::RotatingFileSinkConfig& qConfig,                                   // Rotating File Sink Config
                                 const std::string& szFormatPattern,                                             // Custom Format Pattern
                                 const std::string& szTimeFormat,                                                // Custom Time Format
                                 quill::Timezone qTimestampTimezone          = quill::Timezone::LocalTime,       // Timezone
                                 quill::FileEventNotifier qFileEventNotifier = quill::FileEventNotifier{}        // Event Notifier (Default: None)
                                 ) :
                quill::RotatingFileSink(qFilename, qConfig, qFileEventNotifier),                                 // Pass Parameters into quill::RotatingFileSink
                qFormatter(quill::PatternFormatterOptions(szFormatPattern, szTimeFormat, qTimestampTimezone))    // Pass Parameters into qFormatter type
            {}

            void write_log(const quill::MacroMetadata* qLogMetadata,
                           uint64_t unLogTimestamp,
                           std::string_view szThreadID,
                           std::string_view szThreadName,
                           const std::string& szProcessID,
                           std::string_view szLoggerName,
                           quill::LogLevel qLogLevel,
                           std::string_view szLogLevelDescription,
                           std::string_view szLogLevelShortCode,
                           const std::vector<std::pair<std::string, std::string>>* vNamedArgs,
                           std::string_view szLogMessage,
                           std::string_view) override;

        private:
            quill::PatternFormatter qFormatter;
    };

    /******************************************************************************
     * @brief A custom logger sink designed to send formatted log messages over the
     *        RoveComm protocol. This class extends `quill::Sink` and is tailored for
     *        use in the Autonomy system, where log messages need to be transmitted
     *        as packets over a network to a BaseStation via UDP.
     *
     * The `MRDTRoveCommSink` class formats log messages using a specified pattern and
     * time format, and then transmits the formatted messages as `RoveCommPacket`
     * objects. It integrates with the Quill logging framework and is designed to
     * handle both real-time and networked logging scenarios.
     *
     * ### Key Features:
     * - Customizable log message formats using a pattern.
     * - Integration with the RoveComm protocol for network transmission of log messages.
     * - Handles the conversion of log messages to a format suitable for network transmission.
     * - Inherits from `quill::Sink` for seamless integration with Quill's logging framework.
     *
     * @note This class is intended for use in networked logging scenarios where logs
     *       are transmitted to a BaseStation using the RoveComm protocol. It should
     *       not be called directly but instead used as part of the Quill logging framework.
     *
     * @see quill::Sink
     * @see rovecomm::RoveCommPacket
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-03-17
     ******************************************************************************/
    class MRDTRoveCommSink : public quill::Sink
    {
        private:
            quill::PatternFormatter qFormatter;

            /******************************************************************************
             * @brief A utility function to convert a string to a vector that is no longer
             *        than 255 characters long.
             *
             * @param szString - The string to convert
             * @return std::vector<char> - The string shown as a vector of characters.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-03-17
             ******************************************************************************/
            std::vector<char> StringToVector(const std::string& szString)
            {
                std::vector<char> result;
                int length = std::min(static_cast<int>(szString.length()), 255);
                result.reserve(length);

                for (int i = 0; i < length; ++i)
                {
                    result.push_back(szString[i]);
                }

                return result;
            }

        public:
            /******************************************************************************
             * @brief Constructs a new MRDTRoveCommSink object with the specified format pattern,
             *        time format, and optional timezone. This constructor initializes the
             *        sink with the necessary format settings for logging messages and prepares
             *        them to be transmitted over the RoveComm protocol.
             *
             * @param szFormatPattern - The pattern used to format the log message.
             * @param szTimeFormat - The format of the timestamp in the log message.
             * @param qTimestampTimezone - The timezone used for the timestamp (default: LocalTime).
             *
             * @note Ensure that the format pattern and time format are correctly set to match
             *       the expected format for the log messages. This is crucial for ensuring the
             *       proper transmission and interpretation of log data over the RoveComm protocol.
             *
             * @warning Misconfiguration of the format pattern or time format may lead to
             *          incorrect log formatting and potential issues with packet transmission.
             *
             * @see quill::Sink
             * @see rovecomm::RoveCommPacket
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-03-17
             ******************************************************************************/
            MRDTRoveCommSink(std::string const& szFormatPattern, std::string const& szTimeFormat, quill::Timezone qTimestampTimezone) :
                qFormatter(quill::PatternFormatterOptions(szFormatPattern, szTimeFormat, qTimestampTimezone))
            {}

            /******************************************************************************
             * @brief Destroy the MRDTRoveCommSink object.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-03-17
             ******************************************************************************/
            ~MRDTRoveCommSink() override = default;

            void write_log(const quill::MacroMetadata* qLogMetadata,
                           uint64_t unLogTimestamp,
                           std::string_view szThreadID,
                           std::string_view szThreadName,
                           const std::string& szProcessID,
                           std::string_view szLoggerName,
                           quill::LogLevel qLogLevel,
                           std::string_view szLogLevelDescription,
                           std::string_view szLogLevelShortCode,
                           const std::vector<std::pair<std::string, std::string>>* vNamedArgs,
                           std::string_view szLogMessage,
                           std::string_view) override;

            /******************************************************************************
             * @brief This method should never be called by this codebase, it is called
             *        internally by the quill library.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-03-17
             ******************************************************************************/
            void flush_sink() noexcept override {}
    };

}    // namespace logging
#endif    // AUTONOMY_LOGGING_H
