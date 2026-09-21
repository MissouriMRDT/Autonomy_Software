/******************************************************************************
 * @brief Live Log Stream View (Tab 3) for Autonomy TUI using FTXUI.
 *
 * @file LogView.h
 * @author Missouri MRDT
 * @date 2026-09-21
 ******************************************************************************/

#ifndef LOG_VIEW_H
#define LOG_VIEW_H

#include "../TuiLogSink.h"
#include <ftxui/dom/elements.hpp>
#include <algorithm>
#include <string>
#include <vector>

namespace tui::views
{
    using namespace ftxui;

    inline Color GetLogLevelColor(quill::LogLevel eLevel)
    {
        switch (eLevel)
        {
            case quill::LogLevel::Critical:
                return Color::RedLight;
            case quill::LogLevel::Error:
                return Color::Red;
            case quill::LogLevel::Warning:
                return Color::Yellow;
            case quill::LogLevel::Info:
                return Color::GreenLight;
            case quill::LogLevel::Debug:
                return Color::CyanLight;
            default:
                return Color::GrayLight;
        }
    }

    inline std::string LogLevelToString(quill::LogLevel eLevel)
    {
        switch (eLevel)
        {
            case quill::LogLevel::Critical: return "CRIT ";
            case quill::LogLevel::Error:    return "ERROR";
            case quill::LogLevel::Warning:  return "WARN ";
            case quill::LogLevel::Info:     return "INFO ";
            case quill::LogLevel::Debug:    return "DEBUG";
            case quill::LogLevel::TraceL1:
            case quill::LogLevel::TraceL2:
            case quill::LogLevel::TraceL3:  return "TRACE";
            default:                        return "LOG  ";
        }
    }

    inline Element RenderLogView(const std::vector<TuiLogEntry>& vEntries,
                                 quill::LogLevel eMinLevel,
                                 bool bAutoScroll,
                                 int nScrollOffset)
    {
        // 1. Controls & Filter Status Header
        auto makeFilterBadge = [&](const std::string& szLabel, quill::LogLevel lvl) {
            bool bActive = (eMinLevel == lvl);
            return text(szLabel)
                   | (bActive ? (bold | bgcolor(Color::Blue) | color(Color::White))
                              : (color(Color::GrayLight)));
        };

        Element controlBar = hbox({
            text(" FILTERS [F1-F5]: ") | bold | color(Color::White),
            makeFilterBadge(" ALL ", quill::LogLevel::TraceL3),
            text(" "),
            makeFilterBadge(" DEBUG ", quill::LogLevel::Debug),
            text(" "),
            makeFilterBadge(" INFO ", quill::LogLevel::Info),
            text(" "),
            makeFilterBadge(" WARN ", quill::LogLevel::Warning),
            text(" "),
            makeFilterBadge(" ERROR ", quill::LogLevel::Error),
            separator(),
            text(bAutoScroll ? " [SPACE: AUTO-SCROLL ON] " : " [SPACE: PAUSED (MANUAL SCROLL)] ")
                | bold
                | color(bAutoScroll ? Color::GreenLight : Color::YellowLight)
                | bgcolor(bAutoScroll ? Color::RGB(10, 30, 10) : Color::RGB(40, 30, 10)),
            separator(),
            text(" TOTAL: " + std::to_string(vEntries.size()) + " ") | color(Color::GrayLight)
        }) | border | bgcolor(Color::RGB(15, 20, 25));

        // 2. Filter entries based on eMinLevel
        std::vector<const TuiLogEntry*> vFiltered;
        vFiltered.reserve(vEntries.size());
        for (const auto& entry : vEntries)
        {
            if (entry.eLevel >= eMinLevel)
            {
                vFiltered.push_back(&entry);
            }
        }

        // 3. Render log lines
        Elements vLogLines;
        if (vFiltered.empty())
        {
            vLogLines.push_back(
                text(" No logs matching current filter level.") | dim | color(Color::GrayLight)
            );
        }
        else
        {
            int nTotal = static_cast<int>(vFiltered.size());
            // If auto-scroll is on, nScrollOffset is 0 (bottom).
            // If paused, nScrollOffset > 0 allows scrolling up into history.
            int nStart = 0;
            if (bAutoScroll)
            {
                // Take up to 200 most recent items to avoid excessive DOM tree size
                nStart = std::max(0, nTotal - 200);
            }
            else
            {
                int nWindowSize = 200;
                int nEnd = std::clamp(nTotal - nScrollOffset, 0, nTotal);
                nStart = std::max(0, nEnd - nWindowSize);
            }

            for (int i = nStart; i < nTotal; ++i)
            {
                const auto& entry = *vFiltered[i];
                Color levelCol = GetLogLevelColor(entry.eLevel);

                // Format: [HH:MM:SS.mmm] [LEVEL] [logger] message
                Element lineEl = hbox({
                    text(entry.szTimestamp) | color(Color::GrayDark),
                    text(" [") | color(Color::GrayLight),
                    text(LogLevelToString(entry.eLevel)) | bold | color(levelCol),
                    text("] ") | color(Color::GrayLight),
                    text("[" + entry.szLoggerName + "] ") | color(Color::CyanLight),
                    text(entry.szMessage) | color(entry.eLevel >= quill::LogLevel::Warning ? levelCol : Color::White)
                });

                vLogLines.push_back(lineEl);
            }
        }

        Element logWindow = window(
            text(" 📜 LIVE LOG STREAM ") | bold | color(Color::GreenLight),
            vbox(std::move(vLogLines)) | vscroll_indicator | yframe | flex
        ) | flex;

        return vbox({
            controlBar,
            logWindow
        }) | flex;
    }
}    // namespace tui::views

#endif    // LOG_VIEW_H
