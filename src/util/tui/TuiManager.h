/******************************************************************************
 * @brief Interactive Terminal User Interface Manager using FTXUI.
 *
 * @file TuiManager.h
 * @author Missouri MRDT
 * @date 2026-09-21
 ******************************************************************************/

#ifndef TUI_MANAGER_H
#define TUI_MANAGER_H

#include "TuiTelemetrySnapshot.h"
#include "TuiLogSink.h"
#include "SystemMetrics.h"

#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <functional>

namespace tui
{
    class TuiManager;
    extern std::atomic<TuiManager*> g_pActiveTuiManager;

    class TerminalGuard
    {
    public:
        TerminalGuard();
        ~TerminalGuard();
        void Restore();

    private:
        bool m_bRestored = false;
    };

    class TuiManager
    {
    public:
        using QuitCallback = std::function<void()>;

        explicit TuiManager(std::shared_ptr<TuiLogBuffer> pLogBuffer, QuitCallback fnOnQuit = nullptr);
        ~TuiManager();

        void Start();
        void Stop();
        void RequestQuit();
        bool IsRunning() const;

        void UpdateTelemetry(const TuiTelemetrySnapshot& snap);
        QuitCallback GetQuitCallback() const { return m_fnOnQuit; }

    private:
        void RenderThreadFunc();
        void RefreshLoopFunc();

        ftxui::Element RenderTabBar();
        ftxui::Element RenderMainContent();

        std::shared_ptr<TuiLogBuffer> m_pLogBuffer;
        QuitCallback m_fnOnQuit;

        SystemMetricsCollector m_metricsCollector;
        HardwareStats m_cachedHardwareStats;
        TuiTelemetrySnapshot m_stTelemetrySnapshot;
        mutable std::mutex m_mtxTelemetry;
        mutable std::mutex m_mtxHardware;

        // TUI Navigation and View State
        int m_nActiveTab = 0;              // 0: Telemetry, 1: Hardware, 2: Logs
        quill::LogLevel m_eLogMinLevel = quill::LogLevel::TraceL3;
        bool m_bLogAutoScroll = true;
        int m_nLogScrollOffset = 0;

        std::atomic<bool> m_bRunning{false};
        std::thread m_thRender;
        std::thread m_thRefresh;

        std::atomic<ftxui::ScreenInteractive*> m_pScreen{nullptr};
        std::unique_ptr<TerminalGuard> m_pTermGuard;
    };
}    // namespace tui

#endif    // TUI_MANAGER_H
