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
#include <csignal>

namespace tui
{
    std::atomic<TuiManager*> g_pActiveTuiManager{nullptr};

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

        g_pActiveTuiManager.store(this);

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

        g_pActiveTuiManager.store(nullptr);

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

    void TuiManager::RequestQuit()
    {
        ftxui::ScreenInteractive* pScreen = m_pScreen.load();
        if (pScreen)
        {
            pScreen->Exit();
            pScreen->PostEvent(ftxui::Event::Custom);
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

        auto makeTab = [this](int nIndex, const std::string& szLabel, ftxui::Box& box) {
            bool bSelected = (m_nActiveTab == nIndex);
            return text(" " + szLabel + " ")
                   | (bSelected ? (bold | bgcolor(Color::Blue) | color(Color::White))
                                : (color(Color::GrayLight)))
                   | reflect(box);
        };

        return hbox({
            text(" AUTONOMY TUI DASHBOARD ") | bold | color(Color::CyanLight),
            separator(),
            makeTab(0, "1: TELEMETRY", m_boxTab0),
            text(" "),
            makeTab(1, "2: HARDWARE", m_boxTab1),
            text(" "),
            makeTab(2, "3: LIVE LOGS", m_boxTab2),
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
                currentView = views::RenderLogView(vLogs,
                                                   m_nLogSubTab,
                                                   m_bLogAutoScroll,
                                                   m_nLogScrollOffset,
                                                   m_szSearchQuery,
                                                   m_bSearchMode,
                                                   m_boxSubTab0,
                                                   m_boxSubTab1,
                                                   m_boxSubTab2,
                                                   m_boxSubTab3,
                                                   m_boxSubTab4,
                                                   m_boxSubTab5,
                                                   m_boxAutoScroll,
                                                   m_boxSearch,
                                                   m_boxLogContent);
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
            // Global Ctrl+C / quit handling
            if (event == Event::Special({3}) || event == Event::Special("\x03"))
            {
                if (m_fnOnQuit)
                {
                    m_fnOnQuit();
                }
                screen.Exit();
                return true;
            }

            // Mouse events
            if (event.is_mouse())
            {
                int mx = event.mouse().x;
                int my = event.mouse().y;

                if (event.mouse().button == Mouse::Left && event.mouse().motion == Mouse::Pressed)
                {
                    // Top tab bar clicks
                    if (m_boxTab0.Contain(mx, my)) { m_nActiveTab = 0; return true; }
                    if (m_boxTab1.Contain(mx, my)) { m_nActiveTab = 1; return true; }
                    if (m_boxTab2.Contain(mx, my)) { m_nActiveTab = 2; return true; }

                    // Log tab subtabs and controls
                    if (m_nActiveTab == 2)
                    {
                        if (m_boxSubTab0.Contain(mx, my)) { m_nLogSubTab = 0; return true; }
                        if (m_boxSubTab1.Contain(mx, my)) { m_nLogSubTab = 1; m_eLogMinLevel = quill::LogLevel::TraceL3; return true; }
                        if (m_boxSubTab2.Contain(mx, my)) { m_nLogSubTab = 2; m_eLogMinLevel = quill::LogLevel::Debug; return true; }
                        if (m_boxSubTab3.Contain(mx, my)) { m_nLogSubTab = 3; m_eLogMinLevel = quill::LogLevel::Info; return true; }
                        if (m_boxSubTab4.Contain(mx, my)) { m_nLogSubTab = 4; m_eLogMinLevel = quill::LogLevel::Warning; return true; }
                        if (m_boxSubTab5.Contain(mx, my)) { m_nLogSubTab = 5; m_eLogMinLevel = quill::LogLevel::Error; return true; }

                        if (m_boxAutoScroll.Contain(mx, my))
                        {
                            m_bLogAutoScroll = !m_bLogAutoScroll;
                            if (m_bLogAutoScroll) m_nLogScrollOffset = 0;
                            return true;
                        }

                        if (m_boxSearch.Contain(mx, my))
                        {
                            m_bSearchMode = true;
                            return true;
                        }
                    }
                }
                else if (event.mouse().button == Mouse::WheelUp)
                {
                    if (m_nActiveTab == 2)
                    {
                        m_bLogAutoScroll = false;
                        m_nLogScrollOffset += 3;
                        return true;
                    }
                }
                else if (event.mouse().button == Mouse::WheelDown)
                {
                    if (m_nActiveTab == 2)
                    {
                        m_nLogScrollOffset = std::max(0, m_nLogScrollOffset - 3);
                        if (m_nLogScrollOffset == 0) m_bLogAutoScroll = true;
                        return true;
                    }
                }
            }

            // Search input mode
            if (m_bSearchMode)
            {
                if (event == Event::Escape || event == Event::Return)
                {
                    m_bSearchMode = false;
                    return true;
                }
                if (event == Event::Backspace)
                {
                    if (!m_szSearchQuery.empty())
                    {
                        m_szSearchQuery.pop_back();
                    }
                    return true;
                }
                if (event == Event::Special({21}))    // Ctrl+U: clear query
                {
                    m_szSearchQuery.clear();
                    return true;
                }
                if (event.is_character())
                {
                    m_szSearchQuery += event.character();
                    return true;
                }
                return true;
            }

            // Normal keyboard mode
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
                if (event == Event::Character('/') || event == Event::Character('s') || event == Event::Character('S'))
                {
                    m_bSearchMode = true;
                    return true;
                }
                if (event == Event::Character('c') || event == Event::Character('C'))
                {
                    m_szSearchQuery.clear();
                    return true;
                }
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
                    m_bLogAutoScroll = false;
                    m_nLogScrollOffset += 3;
                    return true;
                }
                if (event == Event::ArrowDown)
                {
                    m_nLogScrollOffset = std::max(0, m_nLogScrollOffset - 3);
                    if (m_nLogScrollOffset == 0)
                    {
                        m_bLogAutoScroll = true;
                    }
                    return true;
                }
                if (event == Event::PageUp)
                {
                    m_bLogAutoScroll = false;
                    m_nLogScrollOffset += 20;
                    return true;
                }
                if (event == Event::PageDown)
                {
                    m_nLogScrollOffset = std::max(0, m_nLogScrollOffset - 20);
                    if (m_nLogScrollOffset == 0)
                    {
                        m_bLogAutoScroll = true;
                    }
                    return true;
                }
                if (event == Event::Home)
                {
                    m_bLogAutoScroll = false;
                    m_nLogScrollOffset = 5000;
                    return true;
                }
                if (event == Event::End)
                {
                    m_nLogScrollOffset = 0;
                    m_bLogAutoScroll = true;
                    return true;
                }
                if (event == Event::Character('0'))
                {
                    m_nLogSubTab = 0;
                    return true;
                }
                if (event == Event::F1)
                {
                    m_nLogSubTab = 1;
                    m_eLogMinLevel = quill::LogLevel::TraceL3;
                    return true;
                }
                if (event == Event::F2)
                {
                    m_nLogSubTab = 2;
                    m_eLogMinLevel = quill::LogLevel::Debug;
                    return true;
                }
                if (event == Event::F3)
                {
                    m_nLogSubTab = 3;
                    m_eLogMinLevel = quill::LogLevel::Info;
                    return true;
                }
                if (event == Event::F4)
                {
                    m_nLogSubTab = 4;
                    m_eLogMinLevel = quill::LogLevel::Warning;
                    return true;
                }
                if (event == Event::F5)
                {
                    m_nLogSubTab = 5;
                    m_eLogMinLevel = quill::LogLevel::Error;
                    return true;
                }
                if (event == Event::Character('f') || event == Event::Character('F'))
                {
                    m_nLogSubTab = (m_nLogSubTab + 1) % 6;
                    if (m_nLogSubTab == 1) m_eLogMinLevel = quill::LogLevel::TraceL3;
                    else if (m_nLogSubTab == 2) m_eLogMinLevel = quill::LogLevel::Debug;
                    else if (m_nLogSubTab == 3) m_eLogMinLevel = quill::LogLevel::Info;
                    else if (m_nLogSubTab == 4) m_eLogMinLevel = quill::LogLevel::Warning;
                    else if (m_nLogSubTab == 5) m_eLogMinLevel = quill::LogLevel::Error;
                    return true;
                }
            }

            return false;
        });

        Loop loop(&screen, component);

        // Reinstall signal handler after FTXUI PreMain() to ensure Ctrl+C triggers graceful shutdown
        struct sigaction stSig;
        stSig.sa_handler = [](int nSig) {
            (void)nSig;
            TuiManager* pMgr = g_pActiveTuiManager.load();
            if (pMgr)
            {
                auto fnQuit = pMgr->GetQuitCallback();
                if (fnQuit)
                {
                    fnQuit();
                }
                pMgr->RequestQuit();
            }
        };
        stSig.sa_flags = 0;
        sigemptyset(&stSig.sa_mask);
        sigaction(SIGINT, &stSig, nullptr);
        sigaction(SIGTERM, &stSig, nullptr);

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
