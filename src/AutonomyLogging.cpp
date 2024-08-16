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
        szFilename = "console_output";                     // Turn the current time into a file name.

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

        // Create Handlers
        std::shared_ptr<quill::Handler> qLogFileHandler  = quill::rotating_file_handler(szFullOutputPath.replace_extension(".log"));
        std::shared_ptr<quill::Handler> qCSVFileHandler  = quill::rotating_file_handler(szFullOutputPath.replace_extension(".csv"));
        std::shared_ptr<quill::Handler> qConsoleHandler  = quill::stdout_handler("ConsoleHandler", qColors);
        std::shared_ptr<quill::Handler> qRoveCommHandler = quill::create_handler<RoveCommHandler>("RoveCommHandler");

        // Configure Patterns
        qLogFileHandler->set_pattern("%(time) %(log_level) [%(thread_id)] [%(file_name):%(line_number)] %(message)",                // format
                                     "%Y-%m-%d %H:%M:%S.%Qms",                                                                      // timestamp format
                                     quill::Timezone::LocalTime);                                                                   // timestamp's timezone

        qCSVFileHandler->set_pattern("%(time),\t%(log_level),\t[%(thread_id)],\t[%(file_name):%(line_number)],\t\"%(message)\"",    // format
                                     "%Y-%m-%d %H:%M:%S.%Qms",                                                                      // timestamp format
                                     quill::Timezone::LocalTime);                                                                   // timestamp's timezone

        qConsoleHandler->set_pattern("%(time) %(log_level) [%(thread_id)] [%(file_name):%(line_number)] %(message)",                // format
                                     "%Y-%m-%d %H:%M:%S.%Qms",                                                                      // timestamp format
                                     quill::Timezone::LocalTime);                                                                   // timestamp's timezone

        qRoveCommHandler->set_pattern("%(time) %(log_level) [%(thread_id)] [%(file_name):%(line_number)] %(message)",               // format
                                      "%Y-%m-%d %H:%M:%S.%Qms",                                                                     // timestamp format
                                      quill::Timezone::LocalTime);                                                                  // timestamp's timezone

        // Configure Quill
        quill::Config qConfig;
        qConfig.enable_console_colours = true;
        qConfig.default_handlers.emplace_back(qConsoleHandler);
        quill::configure(qConfig);

        // Start Quill
        quill::start();

        // Set Handler Filters
        qLogFileHandler->add_filter(std::make_unique<LoggingFilter>("LogFileFilter", quill::LogLevel::TraceL3));
        qCSVFileHandler->add_filter(std::make_unique<LoggingFilter>("CSVFileFilter", quill::LogLevel::TraceL3));
        qConsoleHandler->add_filter(std::make_unique<LoggingFilter>("ConsoleFilter", quill::LogLevel::Info));
        qRoveCommHandler->add_filter(std::make_unique<LoggingFilter>("RoveCommFilter", quill::LogLevel::Info));

        // Create Loggers
        g_qFileLogger     = quill::create_logger("FILE_LOGGER", {qLogFileHandler, qCSVFileHandler});
        g_qConsoleLogger  = quill::create_logger("CONSOLE_LOGGER", {qConsoleHandler});
        g_qRoveCommLogger = quill::create_logger("ROVECOMM_LOGGER", {qRoveCommHandler});
        g_qSharedLogger   = quill::create_logger("SHARED_LOGGER", {qLogFileHandler, qCSVFileHandler, qConsoleHandler /*, qRoveCommHandler*/});

        // Set Base Logging Levels
        g_qFileLogger->set_log_level(quill::LogLevel::TraceL3);
        g_qConsoleLogger->set_log_level(quill::LogLevel::TraceL3);
        g_qRoveCommLogger->set_log_level(quill::LogLevel::TraceL3);
        g_qSharedLogger->set_log_level(quill::LogLevel::TraceL3);

        // Enable Backtrace
        g_qFileLogger->init_backtrace(2, quill::LogLevel::Critical);
        g_qConsoleLogger->init_backtrace(2, quill::LogLevel::Critical);
        g_qRoveCommLogger->init_backtrace(2, quill::LogLevel::Critical);
        g_qSharedLogger->init_backtrace(2, quill::LogLevel::Critical);
    }

    /******************************************************************************
     * @brief This method should never be called by this codebase, it is called
     *        internally by the quill library.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-03-17
     ******************************************************************************/
    void RoveCommHandler::write(quill::fmt_buffer_t const& formatted_log_message, quill::TransitEvent const& log_event)
    {
        // Not using these.
        (void) log_event;

        std::string szTemp{formatted_log_message.data(), formatted_log_message.size()};

        // Construct a RoveComm packet with the logging data.
        rovecomm::RoveCommPacket<char> stPacket;
        stPacket.unDataId    = manifest::Autonomy::TELEMETRY.find("CURRENTLOG")->second.DATA_ID;
        stPacket.unDataCount = manifest::Autonomy::TELEMETRY.find("CURRENTLOG")->second.DATA_COUNT;
        stPacket.eDataType   = manifest::Autonomy::TELEMETRY.find("CURRENTLOG")->second.DATA_TYPE;
        stPacket.vData       = StringToVector({formatted_log_message.data(), formatted_log_message.size()});

        // Send log command over RoveComm to BaseStation.
        if (network::g_bRoveCommUDPStatus && network::g_bRoveCommTCPStatus)
        {
            network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, "0.0.0.0", constants::ROVECOMM_OUTGOING_UDP_PORT);
        }
    }
}    // namespace logging
