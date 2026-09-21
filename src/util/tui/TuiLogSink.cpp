/******************************************************************************
 * @brief Implementation of TuiLogBuffer and MRDTTuiSink for Autonomy TUI.
 *
 * @file TuiLogSink.cpp
 * @author Missouri MRDT
 * @date 2026-09-21
 ******************************************************************************/

#include "TuiLogSink.h"

namespace tui
{
    TuiLogBuffer::TuiLogBuffer(size_t nCapacity) : m_nCapacity(nCapacity), m_nTotalPushed(0) {}

    void TuiLogBuffer::Push(const std::string& szTimestamp,
                            quill::LogLevel eLevel,
                            const std::string& szLogger,
                            const std::string& szMessage,
                            const std::string& szFormatted)
    {
        TuiLogEntry stEntry;
        stEntry.szTimestamp  = szTimestamp;
        stEntry.eLevel       = eLevel;
        stEntry.szLoggerName = szLogger;
        stEntry.szMessage    = szMessage;
        stEntry.szFormatted  = szFormatted;
        Push(stEntry);
    }

    void TuiLogBuffer::Push(const TuiLogEntry& stEntry)
    {
        std::lock_guard<std::mutex> lock(m_mtx);
        m_dqEntries.push_back(stEntry);
        if (m_dqEntries.size() > m_nCapacity)
        {
            m_dqEntries.pop_front();
        }
        ++m_nTotalPushed;
    }

    std::vector<TuiLogEntry> TuiLogBuffer::GetSnapshot(quill::LogLevel eMinLevel) const
    {
        std::lock_guard<std::mutex> lock(m_mtx);
        std::vector<TuiLogEntry> vResult;
        vResult.reserve(m_dqEntries.size());

        for (const auto& entry : m_dqEntries)
        {
            if (static_cast<int>(entry.eLevel) >= static_cast<int>(eMinLevel))
            {
                vResult.push_back(entry);
            }
        }
        return vResult;
    }

    size_t TuiLogBuffer::GetTotalCount() const
    {
        std::lock_guard<std::mutex> lock(m_mtx);
        return m_nTotalPushed;
    }

    void TuiLogBuffer::Clear()
    {
        std::lock_guard<std::mutex> lock(m_mtx);
        m_dqEntries.clear();
        m_nTotalPushed = 0;
    }

    MRDTTuiSink::MRDTTuiSink(std::shared_ptr<TuiLogBuffer> pBuffer,
                             const std::string& szFormatPattern,
                             const std::string& szTimestampPattern,
                             quill::Timezone eTimezone)
        : m_pBuffer(std::move(pBuffer)),
          m_qFormatter(quill::PatternFormatterOptions(szFormatPattern, szTimestampPattern, eTimezone))
    {
    }

    void MRDTTuiSink::write_log(quill::MacroMetadata const* qLogMetadata,
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
                                std::string_view szFormattedLogMessage)
    {
        (void) szFormattedLogMessage;

        std::string_view szFormatted = m_qFormatter.format(unLogTimestamp,
                                                          szThreadID,
                                                          szThreadName,
                                                          szProcessID,
                                                          szLoggerName,
                                                          szLogLevelDescription,
                                                          szLogLevelShortCode,
                                                          *qLogMetadata,
                                                          vNamedArgs,
                                                          szLogMessage);

        if (m_pBuffer)
        {
            m_pBuffer->Push(std::string(szLogLevelDescription),
                            qLogLevel,
                            std::string(szLoggerName),
                            std::string(szLogMessage),
                            std::string(szFormatted));
        }
    }
}    // namespace tui
