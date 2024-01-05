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
#include "AutonomyConstants.h"

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
    quill::Logger* g_qSharedLogger;
    std::string g_szProgramStartTimeString;

    /******************************************************************************
     * @brief Logger Initializer - Sets Up all the logging handlers required for
     *        having the above loggers.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2023-08-22
     ******************************************************************************/
    void InitializeLoggers()
    {
        // Retrieve the current time for the log file name
        time_t tCurrentTime   = time(nullptr);
        struct tm sTimeStruct = *localtime(&tCurrentTime);
        char cCurrentTime[80];

        // Format the current time in a format that can be used as a file name
        std::strftime(cCurrentTime, sizeof(cCurrentTime), "%Y%m%d-%H%M%S", &sTimeStruct);
        // Store start time string in member variable.
        g_szProgramStartTimeString = cCurrentTime;

        // Assemble filepath string.
        std::filesystem::path szFilePath;
        std::filesystem::path szFilenameWithExtension;
        szFilePath = constants::LOGGING_OUTPUT_PATH_ABSOLUTE;             // Main location for all recordings.
        szFilePath += "/console_output/";                                 // Folder for each program run.
        szFilenameWithExtension = g_szProgramStartTimeString + ".log";    // Turn the current time into a file name.

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
        std::filesystem::path szFullOutputPath = szFilePath / szFilenameWithExtension;

        // Create Handlers
        std::shared_ptr<quill::Handler> qFileHandler    = quill::rotating_file_handler(szFullOutputPath);
        std::shared_ptr<quill::Handler> qConsoleHandler = quill::stdout_handler();

        // Configure Patterns
        qFileHandler->set_pattern("%(ascii_time) %(level_name) [%(process)] [%(thread)] %(message)",       // format
                                  "%Y-%m-%d %H:%M:%S.%Qms",                                                // timestamp format
                                  quill::Timezone::GmtTime);                                               // timestamp's timezone

        qConsoleHandler->set_pattern("%(ascii_time) %(level_name) [%(process)] [%(thread)] %(message)",    // format
                                     "%Y-%m-%d %H:%M:%S.%Qms",                                             // timestamp format
                                     quill::Timezone::GmtTime);                                            // timestamp's timezone

        // Enable Color Console
        static_cast<quill::ConsoleHandler*>(qConsoleHandler.get())->enable_console_colours();

        // Configure Quill
        quill::Config qConfig;
        qConfig.enable_console_colours = true;
        qConfig.default_handlers.emplace_back(qConsoleHandler);
        quill::configure(qConfig);

        // Start Quill
        quill::start();

        // Create Loggers
        g_qFileLogger    = quill::create_logger("FILE_LOGGER", {qFileHandler});
        g_qConsoleLogger = quill::create_logger("CONSOLE_LOGGER", {qConsoleHandler});
        g_qSharedLogger  = quill::create_logger("SHARED_LOGGER", {qFileHandler, qConsoleHandler});

        // Set Logging Levels
        g_qFileLogger->set_log_level(quill::LogLevel::TraceL3);
        g_qConsoleLogger->set_log_level(quill::LogLevel::TraceL3);
        g_qSharedLogger->set_log_level(quill::LogLevel::TraceL3);

        // // Enable Backtrace
        g_qFileLogger->init_backtrace(2, quill::LogLevel::Critical);
        g_qConsoleLogger->init_backtrace(2, quill::LogLevel::Critical);
        g_qSharedLogger->init_backtrace(2, quill::LogLevel::Critical);
    }
}    // namespace logging
