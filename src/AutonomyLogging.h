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
/// \endcond

#include "./AutonomyConstants.h"

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
    extern std::string g_szProgramStartTimeString;

    /////////////////////////////////////////
    // Declare namespace methods.
    /////////////////////////////////////////

    void InitializeLoggers(std::string szLoggingOutputPath);

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
             *      by the quill library.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-03-16
             ******************************************************************************/
            QUILL_NODISCARD bool filter(quill::MacroMetadata const* log_metadata,
                                        uint64_t log_timestamp,
                                        std::string_view thread_id,
                                        std::string_view thread_name,
                                        std::string_view logger_name,
                                        quill::LogLevel log_level,
                                        std::string_view log_message,
                                        std::string_view log_statement) noexcept override
            {
                // Not using these.
                (void) log_metadata;
                (void) log_timestamp;
                (void) thread_id;
                (void) thread_name;
                (void) logger_name;
                (void) log_message;
                (void) log_statement;

                // Log only m_eMinLogLevel or higher to stdout.
                return log_level >= m_eMinLogLevel;
            }
    };

    /////////////////////////////////////////
    // Define namespace custom sink
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
             *        The constructor also allows customization of the output stream.
             *
             * @param colours - The console colors configuration for highlighting log levels.
             * @param format_pattern - The pattern used to format the log message.
             * @param time_format - The format of the timestamp in the log message.
             * @param timestamp_timezone - The timezone used for the timestamp (default: LocalTime).
             * @param stream - The stream to output the logs to (default: "stdout").
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
            MRDTConsoleSink(quill::ConsoleColours const& colours,
                            std::string const& format_pattern,
                            std::string const& time_format,
                            quill::Timezone timestamp_timezone = quill::Timezone::LocalTime,
                            std::string const& stream = "stdout") : quill::ConsoleSink(colours, stream), _formatter(format_pattern, time_format, timestamp_timezone)
            {}

            /******************************************************************************
             * @brief Writes a log message to the MRDT console sink, formats the message
             * using the provided formatter, and then passes the formatted log message
             * along with the original data to the parent class (ConsoleSink) for handling.
             *
             * @param log_metadata - Metadata about the log statement (e.g., file, line number).
             * @param log_timestamp - The timestamp of the log statement.
             * @param thread_id - The ID of the thread that generated the log.
             * @param thread_name - The name of the thread that generated the log.
             * @param process_id - The ID of the process that generated the log.
             * @param logger_name - The name of the logger that generated the log.
             * @param log_level - The level/severity of the log statement.
             * @param log_level_description - A description of the log level.
             * @param log_level_short_code - A short code representing the log level.
             * @param named_args - Optional named arguments passed with the log statement.
             * @param log_message - The actual log message content.
             *
             * @note This method calls the base class's `write_log` function to actually
             * handle the log output, after formatting the message with custom formatting logic.
             *
             * @note This method should not be called directly. It is meant to be invoked
             * by the logging framework as part of the log handling process.
             *
             * @see quill::ConsoleSink
             *
             * @warning Ensure that the formatter is correctly configured before using this
             * method, as improper formatting may lead to incorrect log outputs.
             *
             * @attention This method overrides the base class's write_log function to
             * inject custom formatting logic while preserving the core file logging functionality.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-08-16
             ******************************************************************************/
            void write_log(quill::MacroMetadata const* log_metadata,
                           uint64_t log_timestamp,
                           std::string_view thread_id,
                           std::string_view thread_name,
                           std::string const& process_id,
                           std::string_view logger_name,
                           quill::LogLevel log_level,
                           std::string_view log_level_description,
                           std::string_view log_level_short_code,
                           std::vector<std::pair<std::string, std::string>> const* named_args,
                           std::string_view log_message,
                           std::string_view) override;

        private:
            quill::PatternFormatter _formatter;
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
             * @param filename - The path to the log file.
             * @param config - The configuration for rotating the log file (e.g., based on size or time).
             * @param format_pattern - The pattern used to format the log message.
             * @param time_format - The format of the timestamp in the log message.
             * @param timestamp_timezone - The timezone used for the timestamp (default: LocalTime).
             * @param file_event_notifier - Optional event notifier for file-related events (default: none).
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
            MRDTRotatingFileSink(quill::fs::path const& filename,
                                 quill::RotatingFileSinkConfig const& config,
                                 std::string const& format_pattern,
                                 std::string const& time_format,
                                 quill::Timezone timestamp_timezone           = quill::Timezone::LocalTime,
                                 quill::FileEventNotifier file_event_notifier = quill::FileEventNotifier{}) :
                quill::RotatingFileSink(filename, config, file_event_notifier), _formatter(format_pattern, time_format, timestamp_timezone)
            {}

            /******************************************************************************
             * @brief Writes a log message to the MRDT rotating file sink. The log message
             * is first formatted using a custom formatter, and then the formatted message
             * along with the original log details are passed to the base class
             * (RotatingFileSink) for further handling (such as writing to a rotating log file).
             *
             * @param log_metadata - Metadata about the log statement (e.g., file, line number).
             * @param log_timestamp - The timestamp of the log statement.
             * @param thread_id - The ID of the thread that generated the log.
             * @param thread_name - The name of the thread that generated the log.
             * @param process_id - The ID of the process that generated the log.
             * @param logger_name - The name of the logger that generated the log.
             * @param log_level - The level/severity of the log statement.
             * @param log_level_description - A description of the log level.
             * @param log_level_short_code - A short code representing the log level.
             * @param named_args - Optional named arguments passed with the log statement.
             * @param log_message - The actual log message content.
             *
             * @note This method formats the log message using the provided formatter,
             * ensuring that the final output adheres to the defined format pattern. The
             * formatted message is then handled by the rotating file sink for writing to
             * a file that rotates based on file size or time interval.
             *
             * @note This method should not be called directly. It is meant to be invoked
             * by the logging framework as part of the log handling process.
             *
             * @see quill::RotatingFileSink
             *
             * @warning Ensure that the formatter is correctly configured and that the
             * rotating file sink is properly set up to avoid loss of log data.
             *
             * @attention This method overrides the base class's write_log function to
             * inject custom formatting logic while preserving the core file logging functionality.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-08-16
             ******************************************************************************/
            void write_log(quill::MacroMetadata const* log_metadata,
                           uint64_t log_timestamp,
                           std::string_view thread_id,
                           std::string_view thread_name,
                           std::string const& process_id,
                           std::string_view logger_name,
                           quill::LogLevel log_level,
                           std::string_view log_level_description,
                           std::string_view log_level_short_code,
                           std::vector<std::pair<std::string, std::string>> const* named_args,
                           std::string_view log_message,
                           std::string_view) override;

        private:
            quill::PatternFormatter _formatter;
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
            quill::PatternFormatter _formatter;

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
             * @param format_pattern - The pattern used to format the log message.
             * @param time_format - The format of the timestamp in the log message.
             * @param timestamp_timezone - The timezone used for the timestamp (default: LocalTime).
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
            MRDTRoveCommSink(std::string const& format_pattern, std::string const& time_format, quill::Timezone timestamp_timezone) :
                _formatter(format_pattern, time_format, timestamp_timezone)
            {}

            /******************************************************************************
             * @brief Destroy the MRDTRoveCommSink object.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-03-17
             ******************************************************************************/
            ~MRDTRoveCommSink() override = default;

            /******************************************************************************
             * @brief Formats a log message and sends it as a RoveComm packet to the
             * BaseStation. The log message is formatted using the provided metadata,
             * thread, and process information, then packed and transmitted via RoveComm
             * protocol to a specified IP address and port.
             *
             * This function utilizes a custom formatter to combine metadata and log
             * message content into a single formatted string, which is then converted
             * into a `RoveCommPacket` and sent to the BaseStation over UDP.
             *
             * @param log_metadata - Metadata about the log statement (e.g., file, line number).
             * @param log_timestamp - The timestamp of the log statement.
             * @param thread_id - The ID of the thread that generated the log.
             * @param thread_name - The name of the thread that generated the log.
             * @param process_id - The ID of the process that generated the log.
             * @param logger_name - The name of the logger that generated the log.
             * @param log_level - The level/severity of the log statement (currently unused).
             * @param log_level_description - A description of the log level.
             * @param log_level_short_code - A short code representing the log level.
             * @param named_args - Optional named arguments passed with the log statement.
             * @param log_message - The actual log message content.
             * @param log_statement - The full log statement (currently unused).
             *
             * @note This method formats the log message and sends it as a RoveComm packet
             * to the BaseStation if both UDP and TCP statuses are active. The log level
             * and log statement parameters are not currently used in the packet creation.
             *
             * @note This method should not be called directly. It is meant to be invoked
             * by the logging framework as part of the log handling process.
             *
             * @see quill::MacroMetadata
             * @see rovecomm::RoveCommPacket
             * @see network::SendUDPPacket
             *
             * @warning Ensure that the RoveComm protocol is correctly configured and that
             * both UDP and TCP statuses are active before calling this method to avoid
             * packet transmission failure.
             *
             * @attention The formatter must be properly configured to ensure the correct
             * format of the log message before it is sent as a RoveComm packet.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-08-16
             ******************************************************************************/
            void write_log(quill::MacroMetadata const* log_metadata,
                           uint64_t log_timestamp,
                           std::string_view thread_id,
                           std::string_view thread_name,
                           std::string const& process_id,
                           std::string_view logger_name,
                           quill::LogLevel log_level,
                           std::string_view log_level_description,
                           std::string_view log_level_short_code,
                           std::vector<std::pair<std::string, std::string>> const* named_args,
                           std::string_view log_message,
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
