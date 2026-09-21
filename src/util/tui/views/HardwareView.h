/******************************************************************************
 * @brief Hardware Monitor View (Tab 2) for Autonomy TUI using FTXUI.
 *
 * @file HardwareView.h
 * @author Missouri MRDT
 * @date 2026-09-21
 ******************************************************************************/

#ifndef HARDWARE_VIEW_H
#define HARDWARE_VIEW_H

#include "../SystemMetrics.h"
#include <ftxui/dom/elements.hpp>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <vector>

namespace tui::views
{
    using namespace ftxui;

    inline Color GetUsageColor(float fPercent)
    {
        if (fPercent < 50.0f) return Color::Green;
        if (fPercent < 80.0f) return Color::Yellow;
        return Color::Red;
    }

    inline Color GetTempColor(float fTemp)
    {
        if (fTemp < 60.0f) return Color::Green;
        if (fTemp < 80.0f) return Color::Yellow;
        return Color::Red;
    }

    inline Element RenderHardwareView(const HardwareStats& stats)
    {
        // 1. Thermals & Load Summary Bar
        std::ostringstream ossLoad, ossCpuTemp, ossGpuTemp, ossBoardTemp;
        ossLoad << std::fixed << std::setprecision(2)
                << stats.fLoad1Min << ", " << stats.fLoad5Min << ", " << stats.fLoad15Min;
        ossCpuTemp << std::fixed << std::setprecision(1) << stats.fCpuTempCelsius << "°C";
        ossGpuTemp << std::fixed << std::setprecision(1) << stats.fGpuTempCelsius << "°C";
        ossBoardTemp << std::fixed << std::setprecision(1) << stats.fBoardTempCelsius << "°C";

        std::string szCpuTempStr = stats.fCpuTempCelsius > 0.0f
            ? ossCpuTemp.str()
            : (stats.bIsVirtualMachine ? "N/A (VM)" : "N/A");
        std::string szBoardTempStr = stats.fBoardTempCelsius > 0.0f
            ? ossBoardTemp.str()
            : (stats.bIsVirtualMachine ? "N/A (VM)" : "N/A");
        std::string szGpuTempStr = stats.fGpuTempCelsius > 0.0f
            ? ossGpuTemp.str()
            : "N/A";

        Element summaryBar = hbox({
            text(" LOAD AVG: ") | bold,
            text(ossLoad.str()) | color(Color::CyanLight),
            separator(),
            text(" CPU TEMP: ") | bold,
            text(szCpuTempStr) | bold | color(stats.fCpuTempCelsius > 0.0f ? GetTempColor(stats.fCpuTempCelsius) : Color::GrayLight),
            separator(),
            text(" GPU TEMP: ") | bold,
            text(szGpuTempStr) | bold | color(stats.fGpuTempCelsius > 0.0f ? GetTempColor(stats.fGpuTempCelsius) : Color::GrayLight),
            separator(),
            text(" BOARD TEMP: ") | bold,
            text(szBoardTempStr) | color(stats.fBoardTempCelsius > 0.0f ? GetTempColor(stats.fBoardTempCelsius) : Color::GrayLight)
        }) | border | bgcolor(Color::RGB(20, 25, 35));

        // 2. CPU Pane (Per-Core btop-style horizontal meters)
        Elements vCoreElementsCol1;
        Elements vCoreElementsCol2;
        size_t nCores = stats.vCoreUsage.size();
        size_t nHalf = (nCores + 1) / 2;

        for (size_t i = 0; i < nCores; ++i)
        {
            float fPct = std::clamp(stats.vCoreUsage[i], 0.0f, 100.0f);
            std::ostringstream ossPct;
            ossPct << std::setw(3) << static_cast<int>(fPct) << "%";

            std::ostringstream ossCoreLabel;
            ossCoreLabel << " C" << std::setw(2) << std::left << i << " ";

            Element coreRow = hbox({
                text(ossCoreLabel.str()) | color(Color::GrayLight),
                gauge(fPct / 100.0f) | color(GetUsageColor(fPct)) | flex,
                text(" " + ossPct.str()) | bold | color(Color::White)
            });

            if (i < nHalf)
            {
                vCoreElementsCol1.push_back(coreRow);
            }
            else
            {
                vCoreElementsCol2.push_back(coreRow);
            }
        }

        // Total CPU Row
        float fTotPct = std::clamp(stats.fCpuTotalUsage, 0.0f, 100.0f);
        std::ostringstream ossTotPct;
        ossTotPct << std::setw(3) << static_cast<int>(fTotPct) << "%";
        Element totalCpuRow = hbox({
            text(" TOTAL: ") | bold | color(Color::White),
            gauge(fTotPct / 100.0f) | color(GetUsageColor(fTotPct)) | flex,
            text(" " + ossTotPct.str()) | bold | color(Color::YellowLight)
        });

        Element cpuGrid;
        if (vCoreElementsCol2.empty())
        {
            cpuGrid = vbox(std::move(vCoreElementsCol1));
        }
        else
        {
            cpuGrid = hbox({
                vbox(std::move(vCoreElementsCol1)) | flex,
                separator(),
                vbox(std::move(vCoreElementsCol2)) | flex
            });
        }

        Element cpuPane = window(
            text(" PROCESSOR (CPU) ") | bold | color(Color::CyanLight),
            vbox({
                totalCpuRow,
                separator(),
                cpuGrid | flex
            })
        ) | flex;

        // 3. Memory & Swap Pane
        float fRamPct = (stats.fRamTotalGB > 0.0f) ? std::clamp((stats.fRamUsedGB / stats.fRamTotalGB) * 100.0f, 0.0f, 100.0f) : 0.0f;
        float fSwapPct = (stats.fSwapTotalGB > 0.0f) ? std::clamp((stats.fSwapUsedGB / stats.fSwapTotalGB) * 100.0f, 0.0f, 100.0f) : 0.0f;

        std::ostringstream ossRam, ossSwap;
        ossRam << std::fixed << std::setprecision(2) << stats.fRamUsedGB << " / " << stats.fRamTotalGB << " GB (" << static_cast<int>(fRamPct) << "%)";
        ossSwap << std::fixed << std::setprecision(2) << stats.fSwapUsedGB << " / " << stats.fSwapTotalGB << " GB (" << static_cast<int>(fSwapPct) << "%)";

        Element memPane = window(
            text(" SYSTEM MEMORY ") | bold | color(Color::MagentaLight),
            vbox({
                text(" RAM Usage:") | color(Color::GrayLight),
                hbox({
                    gauge(fRamPct / 100.0f) | color(GetUsageColor(fRamPct)) | flex,
                    text(" " + ossRam.str()) | bold | color(Color::White)
                }),
                separator(),
                text(" Swap Usage:") | color(Color::GrayLight),
                hbox({
                    gauge(fSwapPct / 100.0f) | color(GetUsageColor(fSwapPct)) | flex,
                    text(" " + ossSwap.str()) | bold | color(Color::White)
                })
            })
        );

        // 4. GPU & Accelerator Pane
        float fGpuPct = std::clamp(stats.fGpuUsagePercent, 0.0f, 100.0f);
        std::ostringstream ossGpuPct, ossVram;
        ossGpuPct << std::fixed << std::setprecision(1) << fGpuPct << "%";
        float fVramPct = stats.fVramTotalGB > 0.0f ? (stats.fVramUsedGB / stats.fVramTotalGB) * 100.0f : 0.0f;
        ossVram << std::fixed << std::setprecision(2) << stats.fVramUsedGB << " / " << stats.fVramTotalGB << " GB (" << static_cast<int>(fVramPct) << "%)";

        std::string szModel = !stats.szGpuModel.empty() ? stats.szGpuModel : (stats.fGpuUsagePercent >= 0.0f ? "NVIDIA Tegra Orin / Ampere" : "Unavailable / Standard CPU");

        Elements vGpuElements;
        vGpuElements.push_back(text(" GPU Engine Load:") | color(Color::GrayLight));
        vGpuElements.push_back(
            hbox({
                gauge(fGpuPct / 100.0f) | color(GetUsageColor(fGpuPct)) | flex,
                text(" " + ossGpuPct.str()) | bold | color(Color::White)
            })
        );

        if (stats.fVramTotalGB > 0.0f)
        {
            vGpuElements.push_back(separator());
            vGpuElements.push_back(text(" Dedicated VRAM:") | color(Color::GrayLight));
            vGpuElements.push_back(
                hbox({
                    gauge(fVramPct / 100.0f) | color(GetUsageColor(fVramPct)) | flex,
                    text(" " + ossVram.str()) | bold | color(Color::White)
                })
            );
        }

        vGpuElements.push_back(separator());
        vGpuElements.push_back(
            hbox({
                text(" Model: ") | color(Color::GrayLight),
                text(szModel) | bold | color(Color::YellowLight)
            })
        );

        Element gpuPane = window(
            text(" ACCELERATOR / GPU ") | bold | color(Color::GreenLight),
            vbox(std::move(vGpuElements))
        );

        return vbox({
            summaryBar,
            cpuPane | flex,
            hbox({memPane | flex, gpuPane | flex})
        }) | flex;
    }
}    // namespace tui::views

#endif    // HARDWARE_VIEW_H
