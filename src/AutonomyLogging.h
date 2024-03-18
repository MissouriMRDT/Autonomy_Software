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
#include <quill/Quill.h>
/// \endcond

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
    class LoggingFilter : public quill::FilterBase
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
            LoggingFilter(const std::string szFilterBaseType, const quill::LogLevel eMinLogLevel) : quill::FilterBase(szFilterBaseType)
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
            QUILL_NODISCARD bool filter(char const* thread_id,
                                        std::chrono::nanoseconds log_message_timestamp,
                                        quill::MacroMetadata const& metadata,
                                        quill::fmt_buffer_t const& formatted_record) noexcept override
            {
                // Not using these.
                (void) thread_id;
                (void) log_message_timestamp;
                (void) formatted_record;

                // Log only m_eMinLogLevel or higher to stdout.
                return metadata.level() >= m_eMinLogLevel;
            }
    };

}    // namespace logging
#endif    // AUTONOMY_LOGGING_H
