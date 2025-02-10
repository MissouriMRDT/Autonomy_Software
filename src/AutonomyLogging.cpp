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
    void InitializeLoggers(std::string szLoggingOutputPath, std::string szProgramTimeLogsDir)
    {
        // Store start time string in member variable.
        g_szProgramStartTimeString = szProgramTimeLogsDir;

        // Assemble filepath string.
        std::filesystem::path szFilePath;
        std::filesystem::path szFilename;
        szFilePath = szLoggingOutputPath;                  // Main location for all recordings.
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
        quill::ConsoleSink::Colours qColors;
        qColors.apply_default_colours();
        qColors.assign_colour_to_log_level(quill::LogLevel::TraceL3, constants::szTraceL3Color);
        qColors.assign_colour_to_log_level(quill::LogLevel::TraceL2, constants::szTraceL2Color);
        qColors.assign_colour_to_log_level(quill::LogLevel::TraceL1, constants::szTraceL1Color);
        qColors.assign_colour_to_log_level(quill::LogLevel::Debug, constants::szDebugColor);
        qColors.assign_colour_to_log_level(quill::LogLevel::Info, constants::szInfoColor);
        qColors.assign_colour_to_log_level(quill::LogLevel::Warning, constants::szWarningColor);
        qColors.assign_colour_to_log_level(quill::LogLevel::Error, constants::szErrorColor);
        qColors.assign_colour_to_log_level(quill::LogLevel::Critical, constants::szCriticalColor);
        qColors.assign_colour_to_log_level(quill::LogLevel::Backtrace, constants::szBacktraceColor);

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
                quill::RotatingFileSinkConfig cfg;
                cfg.set_open_mode('a');
                return cfg;               // Rotating File Sink Configs
            }(),
            szLogFilePattern,             // Log Output Pattern
            szTimestampPattern,           // Log Timestamp Pattern
            quill::Timezone::LocalTime    // Log Timezone
        );

        std::shared_ptr<quill::Sink> qCSVFileSink = quill::Frontend::create_or_get_sink<MRDTRotatingFileSink>(
            szFullOutputPath.replace_extension(".csv"),    // Log Output Path
            []()
            {
                quill::RotatingFileSinkConfig cfg;
                cfg.set_open_mode('a');
                return cfg;               // Rotating File Sink Configs
            }(),
            szCSVFilePattern,             // Log Output Pattern
            szTimestampPattern,           // Log Timestamp Pattern
            quill::Timezone::LocalTime    // Log Timezone
        );

        std::shared_ptr<quill::Sink> qConsoleSink =
            quill::Frontend::create_or_get_sink<MRDTConsoleSink>("ConsoleSink",                                // Log Name
                                                                 qColors,                                      // Log Custom Colors
                                                                 quill::ConsoleSink::ColourMode::Automatic,    // Detect is console supports colors.
                                                                 szConsolePattern,                             // Log Output Pattern
                                                                 szTimestampPattern                            // Log Timestamp Pattern
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
     * @param qLogMetadata - Metadata about the log statement (e.g., file, line number).
     * @param unLogTimestamp - The timestamp of the log statement.
     * @param szThreadID - The ID of the thread that generated the log.
     * @param szThreadName - The name of the thread that generated the log.
     * @param szProcessID - The ID of the process that generated the log.
     * @param szLoggerName - The name of the logger that generated the log.
     * @param qLogLevel - The level/severity of the log statement.
     * @param szLogLevelDescription - A description of the log level.
     * @param szLogLevelShortCode - A short code representing the log level.
     * @param vNamedArgs - Optional named arguments passed with the log statement.
     * @param szLogMessage - The actual log message content.
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
    void MRDTConsoleSink::write_log(quill::MacroMetadata const* qLogMetadata,
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
                                    std::string_view)
    {
        // Format the log message
        std::string_view szFormattedLogMessage = qFormatter.format(unLogTimestamp,           // Timestamp
                                                                   szThreadID,               // Thread ID
                                                                   szThreadName,             // Thread name
                                                                   szProcessID,              // Process ID
                                                                   szLoggerName,             // Logger name
                                                                   szLogLevelDescription,    // Log level description
                                                                   szLogLevelShortCode,      // Log level short code
                                                                   *qLogMetadata,            // Log statement metadata
                                                                   vNamedArgs,               // Named arguments
                                                                   szLogMessage              // Log message
        );

        // Check if logging level is permitted
        if (static_cast<int>(g_eConsoleLogLevel) <= static_cast<int>(qLogLevel))
        {
            quill::ConsoleSink::write_log(qLogMetadata,             // Metadata
                                          unLogTimestamp,           // Timestamp
                                          szThreadID,               // Thread ID
                                          szThreadName,             // Thread Name
                                          szProcessID,              // Process ID
                                          szLoggerName,             // Logger name
                                          qLogLevel,                // Log level
                                          szLogLevelDescription,    // Log level description
                                          szLogLevelShortCode,      // Log level short code
                                          vNamedArgs,               // Named arguments
                                          szLogMessage,             // Log Message
                                          szFormattedLogMessage     // Formatted Log Message
            );
        }
    }

    /******************************************************************************
     * @brief Writes a log message to the MRDT rotating file sink. The log message
     * is first formatted using a custom formatter, and then the formatted message
     * along with the original log details are passed to the base class
     * (RotatingFileSink) for further handling (such as writing to a rotating log file).
     *
     * @param qLogMetadata - Metadata about the log statement (e.g., file, line number).
     * @param unLogTimestamp - The timestamp of the log statement.
     * @param szThreadID - The ID of the thread that generated the log.
     * @param szThreadName - The name of the thread that generated the log.
     * @param szProcessID - The ID of the process that generated the log.
     * @param szLoggerName - The name of the logger that generated the log.
     * @param qLogLevel - The level/severity of the log statement.
     * @param szLogLevelDescription - A description of the log level.
     * @param szLogLevelShortCode - A short code representing the log level.
     * @param vNamedArgs - Optional named arguments passed with the log statement.
     * @param szLogMessage - The actual log message content.
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
    void MRDTRotatingFileSink::write_log(const quill::MacroMetadata* qLogMetadata,
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
                                         std::string_view)
    {
        // Format the log message
        std::string_view szFormattedLogMessage = qFormatter.format(unLogTimestamp,           // Timestamp
                                                                   szThreadID,               // Thread ID
                                                                   szThreadName,             // Thread name
                                                                   szProcessID,              // Process ID
                                                                   szLoggerName,             // Logger name
                                                                   szLogLevelDescription,    // Log level description
                                                                   szLogLevelShortCode,      // Log level short code
                                                                   *qLogMetadata,            // Log statement metadata
                                                                   vNamedArgs,               // Named arguments
                                                                   szLogMessage              // Log message
        );

        // Check if logging level is permitted
        if (static_cast<int>(g_eFileLogLevel) <= static_cast<int>(qLogLevel))
        {
            quill::RotatingFileSink::write_log(qLogMetadata,             // Metadata
                                               unLogTimestamp,           // Timestamp
                                               szThreadID,               // Thread ID
                                               szThreadName,             // Thread Name
                                               szProcessID,              // Process ID
                                               szLoggerName,             // Logger name
                                               qLogLevel,                // Log level
                                               szLogLevelDescription,    // Log level description
                                               szLogLevelShortCode,      // Log level short code
                                               vNamedArgs,               // Named arguments
                                               szLogMessage,             // Log Message
                                               szFormattedLogMessage     // Formatted Log Message
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
     * @param qLogMetadata - Metadata about the log statement (e.g., file, line number).
     * @param unLogTimestamp - The timestamp of the log statement.
     * @param szThreadID - The ID of the thread that generated the log.
     * @param szThreadName - The name of the thread that generated the log.
     * @param szProcessID - The ID of the process that generated the log.
     * @param szLoggerName - The name of the logger that generated the log.
     * @param qLogLevel - The level/severity of the log statement (currently unused).
     * @param szLogLevelDescription - A description of the log level.
     * @param szLogLevelShortCode - A short code representing the log level.
     * @param vNamedArgs - Optional named arguments passed with the log statement.
     * @param szLogMessage - The actual log message content.
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
    void MRDTRoveCommSink::write_log(const quill::MacroMetadata* qLogMetadata,
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
                                     std::string_view log_statement)
    {
        // Not using these (for now)
        (void) qLogLevel;
        (void) log_statement;

        // Format the log message
        std::string_view szFormattedLogMessage = qFormatter.format(unLogTimestamp,           // Timestamp
                                                                   szThreadID,               // Thread ID
                                                                   szThreadName,             // Thread name
                                                                   szProcessID,              // Process ID
                                                                   szLoggerName,             // Logger name
                                                                   szLogLevelDescription,    // Log level description
                                                                   szLogLevelShortCode,      // Log level short code
                                                                   *qLogMetadata,            // Log statement metadata
                                                                   vNamedArgs,               // Named arguments
                                                                   szLogMessage              // Log message
        );

        // Check if logging level is permitted
        if (static_cast<int>(g_eRoveCommLogLevel) <= static_cast<int>(qLogLevel))
        {
            // Construct a RoveComm packet with the logging data.
            rovecomm::RoveCommPacket<char> stPacket;
            stPacket.unDataId    = manifest::Autonomy::TELEMETRY.find("CURRENTLOG")->second.DATA_ID;
            stPacket.unDataCount = manifest::Autonomy::TELEMETRY.find("CURRENTLOG")->second.DATA_COUNT;
            stPacket.eDataType   = manifest::Autonomy::TELEMETRY.find("CURRENTLOG")->second.DATA_TYPE;
            stPacket.vData       = StringToVector({szFormattedLogMessage.data(), szFormattedLogMessage.size()});

            // Send log command over RoveComm to BaseStation.
            if (network::g_bRoveCommUDPStatus && network::g_bRoveCommTCPStatus)
            {
                network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, "0.0.0.0", constants::ROVECOMM_OUTGOING_UDP_PORT);
            }
        }
    }
}    // namespace logging
