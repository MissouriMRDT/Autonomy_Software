/******************************************************************************
 * @brief Interactive Terminal User Interface Manager using FTXUI.
 *
 * @file TuiManager.cpp
 * @author Missouri MRDT
 * @date 2026-09-21
 ******************************************************************************/

#include "TuiManager.h"
#include "views/TelemetryView.h"
#include "views/HardwareView.h"
#include "views/LogView.h"

#include <ftxui/component/event.hpp>
#include <ftxui/component/loop.hpp>
#include <iostream>
#include <chrono>

namespace tui
{
    TerminalGuard::TerminalGuard()
    {
    }

    TerminalGuard::~TerminalGuard()
    {
        Restore();
    }

    void TerminalGuard::Restore()
    {
        if (!m_bRestored)
        {
            m_bRestored = true;
            // Restore normal screen buffer, cursor, and text formatting.
            std::cout << "\033[?1049l\033[?25h\033[0m" << std::flush;
        }
    }

    TuiManager::TuiManager(std::shared_ptr<TuiLogBuffer> pLogBuffer, QuitCallback fnOnQuit)
        : m_pLogBuffer(std::move(pLogBuffer)),
          m_fnOnQuit(std::move(fnOnQuit))
    {
        m_pTermGuard = std::make_unique<TerminalGuard>();
    }

    TuiManager::~TuiManager()
    {
        Stop();
    }

    void TuiManager::Start()
    {
        if (m_bRunning.exchange(true))
        {
            return;
        }

        m_thRender = std::thread(&TuiManager::RenderThreadFunc, this);
        m_thRefresh = std::thread(&TuiManager::RefreshLoopFunc, this);

        // Wait until m_pScreen is ready before returning
        while (m_bRunning.load() && m_pScreen.load() == nullptr)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void TuiManager::Stop()
    {
        if (!m_bRunning.exchange(false))
        {
            return;
        }

        ftxui::ScreenInteractive* pScreen = m_pScreen.load();
        if (pScreen)
        {
            pScreen->Exit();
            pScreen->PostEvent(ftxui::Event::Custom);
        }

        if (m_thRefresh.joinable() && m_thRefresh.get_id() != std::this_thread::get_id())
        {
            m_thRefresh.join();
        }

        if (m_thRender.joinable() && m_thRender.get_id() != std::this_thread::get_id())
        {
            m_thRender.join();
        }

        if (m_pTermGuard)
        {
            m_pTermGuard->Restore();
        }
    }

    bool TuiManager::IsRunning() const
    {
        return m_bRunning.load();
    }

    void TuiManager::UpdateTelemetry(const TuiTelemetrySnapshot& snap)
    {
        std::lock_guard<std::mutex> lock(m_mtxTelemetry);
        m_stTelemetrySnapshot = snap;
    }

    ftxui::Element TuiManager::RenderTabBar()
    {
        using namespace ftxui;

        auto makeTab = [this](int nIndex, const std::string& szLabel) {
            bool bSelected = (m_nActiveTab == nIndex);
            return text(" " + szLabel + " ")
                   | (bSelected ? (bold | bgcolor(Color::Blue) | color(Color::White))
                                : (color(Color::GrayLight)));
        };

        return hbox({
            text(" 🤖 AUTONOMY TUI DASHBOARD ") | bold | color(Color::CyanLight),
            separator(),
            makeTab(0, "1: TELEMETRY"),
            text(" "),
            makeTab(1, "2: HARDWARE"),
            text(" "),
            makeTab(2, "3: LIVE LOGS"),
            filler(),
            text(" [Tab/←/→: Switch Tabs | 'q': Quit] ") | color(Color::GrayDark)
        }) | border | bgcolor(Color::RGB(15, 15, 25));
    }

    ftxui::Element TuiManager::RenderMainContent()
    {
        using namespace ftxui;

        Element currentView;
        switch (m_nActiveTab)
        {
            case 0:
            {
                TuiTelemetrySnapshot snap;
                {
                    std::lock_guard<std::mutex> lock(m_mtxTelemetry);
                    snap = m_stTelemetrySnapshot;
                }
                currentView = views::RenderTelemetryView(snap);
                break;
            }
            case 1:
            {
                HardwareStats stats;
                {
                    std::lock_guard<std::mutex> lock(m_mtxHardware);
                    stats = m_cachedHardwareStats;
                }
                currentView = views::RenderHardwareView(stats);
                break;
            }
            case 2:
            default:
            {
                std::vector<TuiLogEntry> vLogs;
                if (m_pLogBuffer)
                {
                    vLogs = m_pLogBuffer->GetSnapshot();
                }
                currentView = views::RenderLogView(vLogs, m_eLogMinLevel, m_bLogAutoScroll, m_nLogScrollOffset);
                break;
            }
        }

        return vbox({
            RenderTabBar(),
            currentView | flex
        }) | flex;
    }

    void TuiManager::RenderThreadFunc()
    {
        using namespace ftxui;

        auto screen = ScreenInteractive::Fullscreen();
        m_pScreen.store(&screen);

        auto renderer = Renderer([this]() {
            return RenderMainContent();
        });

        auto component = CatchEvent(renderer, [this, &screen](Event event) -> bool {
            if (event == Event::Character('q') || event == Event::Character('Q'))
            {
                if (m_fnOnQuit)
                {
                    m_fnOnQuit();
                }
                screen.Exit();
                return true;
            }

            if (event == Event::Character('1'))
            {
                m_nActiveTab = 0;
                return true;
            }
            if (event == Event::Character('2'))
            {
                m_nActiveTab = 1;
                return true;
            }
            if (event == Event::Character('3'))
            {
                m_nActiveTab = 2;
                return true;
            }

            if (event == Event::Tab || event == Event::ArrowRight)
            {
                m_nActiveTab = (m_nActiveTab + 1) % 3;
                return true;
            }

            if (event == Event::TabReverse || event == Event::ArrowLeft)
            {
                m_nActiveTab = (m_nActiveTab + 2) % 3;
                return true;
            }

            // Tab 2 (Live Logs) key interactions
            if (m_nActiveTab == 2)
            {
                if (event == Event::Character(' '))
                {
                    m_bLogAutoScroll = !m_bLogAutoScroll;
                    if (m_bLogAutoScroll)
                    {
                        m_nLogScrollOffset = 0;
                    }
                    return true;
                }
                if (event == Event::ArrowUp)
                {
                    if (!m_bLogAutoScroll)
                    {
                        m_nLogScrollOffset += 5;
                    }
                    return true;
                }
                if (event == Event::ArrowDown)
                {
                    if (!m_bLogAutoScroll)
                    {
                        m_nLogScrollOffset = std::max(0, m_nLogScrollOffset - 5);
                    }
                    return true;
                }
                if (event == Event::PageUp)
                {
                    if (!m_bLogAutoScroll)
                    {
                        m_nLogScrollOffset += 25;
                    }
                    return true;
                }
                if (event == Event::PageDown)
                {
                    if (!m_bLogAutoScroll)
                    {
                        m_nLogScrollOffset = std::max(0, m_nLogScrollOffset - 25);
                    }
                    return true;
                }
                if (event == Event::F1)
                {
                    m_eLogMinLevel = quill::LogLevel::TraceL3;
                    return true;
                }
                if (event == Event::F2)
                {
                    m_eLogMinLevel = quill::LogLevel::Debug;
                    return true;
                }
                if (event == Event::F3)
                {
                    m_eLogMinLevel = quill::LogLevel::Info;
                    return true;
                }
                if (event == Event::F4)
                {
                    m_eLogMinLevel = quill::LogLevel::Warning;
                    return true;
                }
                if (event == Event::F5)
                {
                    m_eLogMinLevel = quill::LogLevel::Error;
                    return true;
                }
                if (event == Event::Character('f') || event == Event::Character('F'))
                {
                    if (m_eLogMinLevel == quill::LogLevel::TraceL3)
                        m_eLogMinLevel = quill::LogLevel::Debug;
                    else if (m_eLogMinLevel == quill::LogLevel::Debug)
                        m_eLogMinLevel = quill::LogLevel::Info;
                    else if (m_eLogMinLevel == quill::LogLevel::Info)
                        m_eLogMinLevel = quill::LogLevel::Warning;
                    else if (m_eLogMinLevel == quill::LogLevel::Warning)
                        m_eLogMinLevel = quill::LogLevel::Error;
                    else
                        m_eLogMinLevel = quill::LogLevel::TraceL3;
                    return true;
                }
            }

            return false;
        });

        Loop loop(&screen, component);
        while (m_bRunning.load() && !loop.HasQuitted())
        {
            loop.RunOnceBlocking();
        }
        m_pScreen.store(nullptr);
    }

    void TuiManager::RefreshLoopFunc()
    {
        uint32_t nTickCount = 0;
        while (m_bRunning.load())
        {
            // Query hardware metrics at ~2 Hz (every 5 ticks of 100ms)
            if (nTickCount % 5 == 0)
            {
                HardwareStats stats = m_metricsCollector.Query();
                {
                    std::lock_guard<std::mutex> lock(m_mtxHardware);
                    m_cachedHardwareStats = stats;
                }
            }

            // Post render event to FTXUI loop at 10 Hz
            ftxui::ScreenInteractive* pScreen = m_pScreen.load();
            if (pScreen)
            {
                pScreen->PostEvent(ftxui::Event::Custom);
            }

            nTickCount++;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}    // namespace tui
