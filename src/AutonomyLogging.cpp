/******************************************************************************
 * @brief Sets up functions and classes used by logging project wide.
 *
 * @file AutonomyLogging.cpp
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-18
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "AutonomyLogging.h"
#include "AutonomyNetworking.h"

/// \cond
#include <RoveComm/RoveComm.h>
#include <RoveComm/RoveCommManifest.h>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iostream>

/// \endcond

/******************************************************************************
 * @brief Namespace containing all global type/structs that will be used project wide
 *      for logging.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
namespace logging
{
    /////////////////////////////////////////
    // Forward declarations for namespace variables and objects.
    /////////////////////////////////////////
    quill::Logger* g_qFileLogger;
    quill::Logger* g_qConsoleLogger;
    quill::Logger* g_qRoveCommLogger;
    quill::Logger* g_qSharedLogger;

    quill::LogLevel g_eConsoleLogLevel;
    quill::LogLevel g_eFileLogLevel;
    quill::LogLevel g_eRoveCommLogLevel;

    std::string g_szProgramStartTimeString;

    /******************************************************************************
     * @brief Logger Initializer - Sets Up all the logging handlers required for
     *        having the above loggers.
     *
     * @param szLoggingOutputPath - A string containing the filepath to output log files to.
     *                      Must be properly formatted.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2023-08-22
     ******************************************************************************/
    void InitializeLoggers(std::string szLoggingOutputPath)
    {
        // Retrieve the current time for the log file name
        std::chrono::time_point<std::chrono::system_clock> tmCurrentTime = std::chrono::system_clock::now();
        std::time_t tCurrentTime                                         = std::chrono::system_clock::to_time_t(tmCurrentTime);

        // Convert time to local time
        std::tm* tLocalTime = std::localtime(&tCurrentTime);

        // Format the current time in a format that can be used as a file name
        char cCurrentTime[80];
        std::strftime(cCurrentTime, sizeof(cCurrentTime), "%Y%m%d-%H%M%S", tLocalTime);

        // Store start time string in member variable.
        g_szProgramStartTimeString = cCurrentTime;

        // Assemble filepath string.
        std::filesystem::path szFilePath;
        std::filesystem::path szFilename;
        szFilePath = szLoggingOutputPath + "/";            // Main location for all recordings.
        szFilePath += g_szProgramStartTimeString + "/";    // Folder for each program run.
        szFilename = "console_output";                     // Base file name.

        // Check if directory exists.
        if (!std::filesystem::exists(szFilePath))
        {
            // Create directory.
            if (!std::filesystem::create_directories(szFilePath))
            {
                // Submit logger message.
                std::cerr << "Unable to create the logging output directory: " << szFilePath.string() << " for console output file." << std::endl;
            }
        }
        else
        {
            // Submit logger message.
            std::cerr << "Unable to create logging output directory " << szFilePath.string() << ": it already exists." << std::endl;
        }

        // Construct the full output path.
        std::filesystem::path szFullOutputPath = szFilePath / szFilename;

        // Set Console Color Profile
        quill::ConsoleColours qColors;
        qColors.set_default_colours();
        qColors.set_colour(quill::LogLevel::TraceL3, constants::szTraceL3Color);
        qColors.set_colour(quill::LogLevel::TraceL2, constants::szTraceL2Color);
        qColors.set_colour(quill::LogLevel::TraceL1, constants::szTraceL1Color);
        qColors.set_colour(quill::LogLevel::Debug, constants::szDebugColor);
        qColors.set_colour(quill::LogLevel::Info, constants::szInfoColor);
        qColors.set_colour(quill::LogLevel::Warning, constants::szWarningColor);
        qColors.set_colour(quill::LogLevel::Error, constants::szErrorColor);
        qColors.set_colour(quill::LogLevel::Critical, constants::szCriticalColor);
        qColors.set_colour(quill::LogLevel::Backtrace, constants::szBacktraceColor);

        // Create Patterns
        std::string szLogFilePattern   = "%(time) %(log_level) [%(thread_id)] [%(file_name):%(line_number)] %(message)";
        std::string szCSVFilePattern   = "%(time),\t%(log_level),\t[%(thread_id)],\t[%(file_name):%(line_number)],\t\"%(message)\"";
        std::string szConsolePattern   = "%(time) %(log_level:9) [%(thread_id)] [%(file_name):%(line_number)] %(message)";
        std::string szRoveCommPattern  = "%(time) %(log_level) [%(thread_id)] [%(file_name):%(line_number)] %(message)";
        std::string szTimestampPattern = "%Y-%m-%d %H:%M:%S.%Qms";

        // Create Sinks
        std::shared_ptr<quill::Sink> qLogFileSink = quill::Frontend::create_or_get_sink<MRDTRotatingFileSink>(
            szFullOutputPath.replace_extension(".log"),    // Log Output Path
            []()
            {
                return quill::RotatingFileSinkConfig();    // Rotating File Sink Configs
            }(),
            szLogFilePattern,                              // Log Output Pattern
            szTimestampPattern,                            // Log Timestamp Pattern
            quill::Timezone::LocalTime                     // Log Timezone
        );

        std::shared_ptr<quill::Sink> qCSVFileSink = quill::Frontend::create_or_get_sink<MRDTRotatingFileSink>(
            szFullOutputPath.replace_extension(".csv"),    // Log Output Path
            []()
            {
                return quill::RotatingFileSinkConfig();    // Rotating File Sink Configs
            }(),
            szCSVFilePattern,                              // Log Output Pattern
            szTimestampPattern,                            // Log Timestamp Pattern
            quill::Timezone::LocalTime                     // Log Timezone
        );

        std::shared_ptr<quill::Sink> qConsoleSink      = quill::Frontend::create_or_get_sink<MRDTConsoleSink>("ConsoleSink",        // Log Name
                                                                                                         qColors,              // Log Custom Colors
                                                                                                         szConsolePattern,     // Log Output Pattern
                                                                                                         szTimestampPattern    // Log Timestamp Pattern
        );

        std::shared_ptr<quill::Sink> qMRDTRoveCommSink = quill::Frontend::create_or_get_sink<MRDTRoveCommSink>("MRDTRoveCommSink",           // Log Name
                                                                                                               szRoveCommPattern,            // Log Output Pattern
                                                                                                               szTimestampPattern,           // Log Timestamp Pattern
                                                                                                               quill::Timezone::LocalTime    // Log Timezone
        );

        // Configure Quill
        quill::BackendOptions qBackendConfig;

        // Start Quill
        quill::Backend::start(qBackendConfig);

        // Set Handler Filters
        qLogFileSink->add_filter(std::make_unique<LoggingFilter>("LogFileFilter", quill::LogLevel::TraceL3));
        qCSVFileSink->add_filter(std::make_unique<LoggingFilter>("CSVFileFilter", quill::LogLevel::TraceL3));
        qConsoleSink->add_filter(std::make_unique<LoggingFilter>("ConsoleFilter", quill::LogLevel::TraceL3));
        qMRDTRoveCommSink->add_filter(std::make_unique<LoggingFilter>("RoveCommFilter", quill::LogLevel::Info));

        // Create Loggers
        g_qFileLogger     = quill::Frontend::create_or_get_logger("FILE_LOGGER", {qLogFileSink, qCSVFileSink});
        g_qConsoleLogger  = quill::Frontend::create_or_get_logger("CONSOLE_LOGGER", {qConsoleSink});
        g_qRoveCommLogger = quill::Frontend::create_or_get_logger("ROVECOMM_LOGGER", {qMRDTRoveCommSink});
        g_qSharedLogger   = quill::Frontend::create_or_get_logger("SHARED_LOGGER", {qLogFileSink, qCSVFileSink, qConsoleSink, qMRDTRoveCommSink});

        // Set Internal Logging Level Limiters
        g_eConsoleLogLevel  = constants::CONSOLE_DEFAULT_LEVEL;
        g_eFileLogLevel     = constants::FILE_DEFAULT_LEVEL;
        g_eRoveCommLogLevel = constants::ROVECOMM_DEFAULT_LEVEL;

        // Set Base Logging Levels
        g_qFileLogger->set_log_level(quill::LogLevel::TraceL3);
        g_qConsoleLogger->set_log_level(quill::LogLevel::TraceL3);
        g_qRoveCommLogger->set_log_level(quill::LogLevel::Info);
        g_qSharedLogger->set_log_level(quill::LogLevel::TraceL3);

        // Enable Backtrace
        g_qFileLogger->init_backtrace(10, quill::LogLevel::Critical);
        g_qConsoleLogger->init_backtrace(10, quill::LogLevel::Critical);
        g_qRoveCommLogger->init_backtrace(10, quill::LogLevel::Critical);
        g_qSharedLogger->init_backtrace(10, quill::LogLevel::Critical);
    }

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
    void MRDTConsoleSink::write_log(quill::MacroMetadata const* log_metadata,
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
                                    std::string_view)
    {
        // Format the log message
        std::string_view szvFormattedLogMessage = _formatter.format(log_timestamp,            // Timestamp
                                                                    thread_id,                // Thread ID
                                                                    thread_name,              // Thread name
                                                                    process_id,               // Process ID
                                                                    logger_name,              // Logger name
                                                                    log_level_description,    // Log level description
                                                                    log_level_short_code,     // Log level short code
                                                                    *log_metadata,            // Log statement metadata
                                                                    named_args,               // Named arguments
                                                                    log_message               // Log message
        );

        // Check if logging level is permitted
        if (static_cast<int>(g_eConsoleLogLevel) <= static_cast<int>(log_level))
        {
            quill::ConsoleSink::write_log(log_metadata,             // Metadata
                                          log_timestamp,            // Timestamp
                                          thread_id,                // Thread ID
                                          thread_name,              // Thread Name
                                          process_id,               // Process ID
                                          logger_name,              // Logger name
                                          log_level,                // Log level
                                          log_level_description,    // Log level description
                                          log_level_short_code,     // Log level short code
                                          named_args,               // Named arguments
                                          log_message,              // Log Message
                                          szvFormattedLogMessage    // Formatted Log Message
            );
        }
    }

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
    void MRDTRotatingFileSink::write_log(quill::MacroMetadata const* log_metadata,
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
                                         std::string_view)
    {
        // Format the log message
        std::string_view szvFormattedLogMessage = _formatter.format(log_timestamp,            // Timestamp
                                                                    thread_id,                // Thread ID
                                                                    thread_name,              // Thread name
                                                                    process_id,               // Process ID
                                                                    logger_name,              // Logger name
                                                                    log_level_description,    // Log level description
                                                                    log_level_short_code,     // Log level short code
                                                                    *log_metadata,            // Log statement metadata
                                                                    named_args,               // Named arguments
                                                                    log_message               // Log message
        );

        // Check if logging level is permitted
        if (static_cast<int>(g_eFileLogLevel) <= static_cast<int>(log_level))
        {
            quill::RotatingFileSink::write_log(log_metadata,             // Metadata
                                               log_timestamp,            // Timestamp
                                               thread_id,                // Thread ID
                                               thread_name,              // Thread Name
                                               process_id,               // Process ID
                                               logger_name,              // Logger name
                                               log_level,                // Log level
                                               log_level_description,    // Log level description
                                               log_level_short_code,     // Log level short code
                                               named_args,               // Named arguments
                                               log_message,              // Log Message
                                               szvFormattedLogMessage    // Formatted Log Message
            );
        }
    }

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
    void MRDTRoveCommSink::write_log(quill::MacroMetadata const* log_metadata,
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
                                     std::string_view log_statement)
    {
        // Not using these (for now)
        (void) log_level;
        (void) log_statement;

        // Format the log message
        std::string_view szvFormattedLogMessage = _formatter.format(log_timestamp,            // Timestamp
                                                                    thread_id,                // Thread ID
                                                                    thread_name,              // Thread name
                                                                    process_id,               // Process ID
                                                                    logger_name,              // Logger name
                                                                    log_level_description,    // Log level description
                                                                    log_level_short_code,     // Log level short code
                                                                    *log_metadata,            // Log statement metadata
                                                                    named_args,               // Named arguments
                                                                    log_message               // Log message
        );

        // Check if logging level is permitted
        if (static_cast<int>(g_eRoveCommLogLevel) <= static_cast<int>(log_level))
        {
            // Construct a RoveComm packet with the logging data.
            rovecomm::RoveCommPacket<char> stPacket;
            stPacket.unDataId    = manifest::Autonomy::TELEMETRY.find("CURRENTLOG")->second.DATA_ID;
            stPacket.unDataCount = manifest::Autonomy::TELEMETRY.find("CURRENTLOG")->second.DATA_COUNT;
            stPacket.eDataType   = manifest::Autonomy::TELEMETRY.find("CURRENTLOG")->second.DATA_TYPE;
            stPacket.vData       = StringToVector({szvFormattedLogMessage.data(), szvFormattedLogMessage.size()});

            // Send log command over RoveComm to BaseStation.
            if (network::g_bRoveCommUDPStatus && network::g_bRoveCommTCPStatus)
            {
                network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, "0.0.0.0", constants::ROVECOMM_OUTGOING_UDP_PORT);
            }
        }
    }
}    // namespace logging
