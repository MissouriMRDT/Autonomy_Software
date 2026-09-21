/******************************************************************************
 * @brief Custom Quill Sink and Ring Buffer for Autonomy TUI Live Log Viewer.
 *
 * @file TuiLogSink.h
 * @author Missouri MRDT
 * @date 2026-09-21
 ******************************************************************************/

#ifndef TUI_LOG_SINK_H
#define TUI_LOG_SINK_H

#include <quill/core/Common.h>
#include <quill/core/Attributes.h>
#include <quill/sinks/Sink.h>
#include <quill/backend/PatternFormatter.h>

#include <string>
#include <string_view>
#include <vector>
#include <deque>
#include <mutex>
#include <memory>
#include <cstdint>

namespace tui
{
    struct TuiLogEntry
    {
        std::string szTimestamp;
        quill::LogLevel eLevel;
        std::string szLoggerName;
        std::string szMessage;
        std::string szFormatted;
    };

    class TuiLogBuffer
    {
    public:
        explicit TuiLogBuffer(size_t nCapacity = 2000);
        ~TuiLogBuffer() = default;

        void Push(const std::string& szTimestamp, quill::LogLevel eLevel, const std::string& szLogger, const std::string& szMessage, const std::string& szFormatted);
        void Push(const TuiLogEntry& stEntry);

        std::vector<TuiLogEntry> GetSnapshot(quill::LogLevel eMinLevel = quill::LogLevel::TraceL3) const;
        size_t GetTotalCount() const;
        void Clear();

    private:
        mutable std::mutex m_mtx;
        size_t m_nCapacity;
        size_t m_nTotalPushed;
        std::deque<TuiLogEntry> m_dqEntries;
    };

    class MRDTTuiSink : public quill::Sink
    {
    public:
        MRDTTuiSink(std::shared_ptr<TuiLogBuffer> pBuffer,
                    const std::string& szFormatPattern = "%(time) %(log_level:9) [%(thread_id)] [%(file_name):%(line_number)] %(message)",
                    const std::string& szTimestampPattern = "%Y-%m-%d %H:%M:%S.%Qms",
                    quill::Timezone eTimezone = quill::Timezone::LocalTime);

        ~MRDTTuiSink() override = default;

        void write_log(quill::MacroMetadata const* qLogMetadata,
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
                       std::string_view szFormattedLogMessage) override;

        void flush_sink() override {}

    private:
        std::shared_ptr<TuiLogBuffer> m_pBuffer;
        quill::PatternFormatter m_qFormatter;
    };
}    // namespace tui

#endif    // TUI_LOG_SINK_H
