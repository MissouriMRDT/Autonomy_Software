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
#include <ftxui/screen/box.hpp>
#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

namespace tui::views
{
    using namespace ftxui;

    inline bool ContainsCaseInsensitive(const std::string& haystack, const std::string& needle)
    {
        if (needle.empty()) return true;
        auto it = std::search(
            haystack.begin(), haystack.end(),
            needle.begin(), needle.end(),
            [](char ch1, char ch2) {
                return std::tolower(static_cast<unsigned char>(ch1)) == std::tolower(static_cast<unsigned char>(ch2));
            }
        );
        return it != haystack.end();
    }

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
                                 int nSubTab,
                                 bool bAutoScroll,
                                 int nScrollOffset,
                                 const std::string& szSearchQuery,
                                 bool bSearchMode,
                                 Box& boxSub0,
                                 Box& boxSub1,
                                 Box& boxSub2,
                                 Box& boxSub3,
                                 Box& boxSub4,
                                 Box& boxSub5,
                                 Box& boxAutoScroll,
                                 Box& boxSearch,
                                 Box& boxLogContent)
    {
        // 1. Controls & Filter Status Header
        auto makeSubtabBadge = [&](int nIdx, const std::string& szLabel, Box& box) {
            bool bActive = (nSubTab == nIdx);
            return text(szLabel)
                   | (bActive ? (bold | bgcolor(Color::Blue) | color(Color::White))
                              : (color(Color::GrayLight)))
                   | reflect(box);
        };

        std::string szSearchStatus;
        if (bSearchMode)
        {
            szSearchStatus = " SEARCH: [" + szSearchQuery + "_] (ENTER/ESC to finish) ";
        }
        else if (!szSearchQuery.empty())
        {
            szSearchStatus = " SEARCH: [" + szSearchQuery + "] (/ to edit, c to clear) ";
        }
        else
        {
            szSearchStatus = " SEARCH: [/ to search] ";
        }

        Element controlBar = hbox({
            text(" VIEWS: ") | bold | color(Color::White),
            makeSubtabBadge(0, " 0: Console/Raw ", boxSub0),
            text(" "),
            makeSubtabBadge(1, " 1: All ", boxSub1),
            text(" "),
            makeSubtabBadge(2, " 2: Debug ", boxSub2),
            text(" "),
            makeSubtabBadge(3, " 3: Info ", boxSub3),
            text(" "),
            makeSubtabBadge(4, " 4: Warn ", boxSub4),
            text(" "),
            makeSubtabBadge(5, " 5: Error ", boxSub5),
            separator(),
            text(szSearchStatus)
                | bold
                | (bSearchMode ? (color(Color::CyanLight) | bgcolor(Color::RGB(10, 30, 40)))
                               : (!szSearchQuery.empty() ? color(Color::YellowLight) : color(Color::GrayLight)))
                | reflect(boxSearch),
            separator(),
            text(bAutoScroll ? " [AUTO-SCROLL ON] " : " [SCROLL PAUSED] ")
                | bold
                | color(bAutoScroll ? Color::GreenLight : Color::YellowLight)
                | bgcolor(bAutoScroll ? Color::RGB(10, 30, 10) : Color::RGB(40, 30, 10))
                | reflect(boxAutoScroll)
        }) | border | bgcolor(Color::RGB(15, 20, 25));

        // 2. Filter entries based on nSubTab and szSearchQuery
        quill::LogLevel eMinLevel = quill::LogLevel::TraceL3;
        if (nSubTab == 2) eMinLevel = quill::LogLevel::Debug;
        else if (nSubTab == 3) eMinLevel = quill::LogLevel::Info;
        else if (nSubTab == 4) eMinLevel = quill::LogLevel::Warning;
        else if (nSubTab == 5) eMinLevel = quill::LogLevel::Error;

        std::vector<const TuiLogEntry*> vFiltered;
        vFiltered.reserve(vEntries.size());
        for (const auto& entry : vEntries)
        {
            if (nSubTab >= 1 && entry.eLevel < eMinLevel)
            {
                continue;
            }

            if (!szSearchQuery.empty())
            {
                if (!ContainsCaseInsensitive(entry.szMessage, szSearchQuery) &&
                    !ContainsCaseInsensitive(entry.szLoggerName, szSearchQuery) &&
                    !ContainsCaseInsensitive(entry.szTimestamp, szSearchQuery) &&
                    !ContainsCaseInsensitive(entry.szFormatted, szSearchQuery))
                {
                    continue;
                }
            }

            vFiltered.push_back(&entry);
        }

        // 3. Render log lines
        Elements vLogLines;

        // In Console / Raw mode (subtab 0), show ASCII software banner at the very top
        static const std::vector<std::string> vBannerLines = {
            "    _       _                            ___       __ _                             ___ ___ ",
            "   /_\\ _  _| |_ ___ _ _  ___ _ __ _  _  / __| ___ / _| |___ __ ____ _ _ _ ___  __ _|_  ) __|",
            "  / _ \\ || |  _/ _ \\ ' \\/ _ \\ '  \\ || | \\__ \\/ _ \\  _|  _\\ V  V / _` | '_/ -_) \\ V // /|__ \\",
            " /_/ \\_\\_,_|\\__\\___/_||_\\___/_|_|_\\_, | |___/\\___/_|  \\__|\\_/\\_/\\__,_|_| \\___|  \\_//___|___/",
            "                                  |__/                                                      ",
            "Copyright \u00A9 2025 - Mars Rover Design Team\n"
        };

        if (vFiltered.empty())
        {
            if (nSubTab == 0)
            {
                for (const auto& bannerLine : vBannerLines)
                {
                    vLogLines.push_back(text(bannerLine) | bold | color(Color::CyanLight));
                }
            }
            vLogLines.push_back(
                text(" No logs matching current filter level or search query.") | dim | color(Color::GrayLight)
            );
        }
        else
        {
            int nTotal = static_cast<int>(vFiltered.size());
            int nWindowSize = 250;
            int nEnd = nTotal;
            int nStart = 0;

            if (bAutoScroll || nScrollOffset <= 0)
            {
                nEnd = nTotal;
                nStart = std::max(0, nEnd - nWindowSize);
            }
            else
            {
                nEnd = std::clamp(nTotal - nScrollOffset, 0, nTotal);
                nStart = std::max(0, nEnd - nWindowSize);
            }

            // Show banner if scrolled to top in console mode
            if (nSubTab == 0 && nStart == 0)
            {
                for (const auto& bannerLine : vBannerLines)
                {
                    vLogLines.push_back(text(bannerLine) | bold | color(Color::CyanLight));
                }
            }

            for (int i = nStart; i < nEnd; ++i)
            {
                const auto& entry = *vFiltered[i];
                Color levelCol = GetLogLevelColor(entry.eLevel);

                if (nSubTab == 0)
                {
                    // Console / Raw view: Display full formatted line just like standard non-TUI terminal output
                    vLogLines.push_back(
                        text(entry.szFormatted.empty() ? entry.szMessage : entry.szFormatted)
                        | color(entry.eLevel >= quill::LogLevel::Warning ? levelCol : Color::White)
                    );
                }
                else
                {
                    // Structured log view
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
        }

        std::string szTitle = (nSubTab == 0)
            ? " CONSOLE OUTPUT (STANDARD RAW STREAM) "
            : " LIVE LOG STREAM ";
        szTitle += "— " + std::to_string(vFiltered.size()) + " / " + std::to_string(vEntries.size()) + " ENTRIES ";
        if (!szSearchQuery.empty())
        {
            szTitle += "[FILTERED: \"" + szSearchQuery + "\"] ";
        }
        if (!bAutoScroll)
        {
            szTitle += "[SCROLL OFFSET: " + std::to_string(nScrollOffset) + " LINES] ";
        }

        Element logWindow = window(
            text(szTitle) | bold | color(Color::GreenLight),
            vbox(std::move(vLogLines)) | vscroll_indicator | yframe | flex
        ) | flex | reflect(boxLogContent);

        return vbox({
            controlBar,
            logWindow
        }) | flex;
    }
}    // namespace tui::views

#endif    // LOG_VIEW_H
