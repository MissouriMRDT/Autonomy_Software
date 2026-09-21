/******************************************************************************
 * @brief Implementation of SystemMetricsCollector for Autonomy TUI.
 *
 * @file SystemMetrics.cpp
 * @author Missouri MRDT
 * @date 2026-09-21
 ******************************************************************************/

#include "SystemMetrics.h"

#include <fstream>
#include <sstream>
#include <cstdlib>
#include <algorithm>
#include <filesystem>

#if defined(__linux__)
#include <sys/sysinfo.h>
#endif

namespace tui
{
    SystemMetricsCollector::SystemMetricsCollector()
    {
        HardwareStats dummy;
        UpdateCpu(dummy);
    }

    HardwareStats SystemMetricsCollector::Query()
    {
        HardwareStats stStats;
        UpdateCpu(stStats);
        UpdateMemory(stStats);
        UpdateThermals(stStats);
        UpdateGpu(stStats);
        return stStats;
    }

    void SystemMetricsCollector::UpdateCpu(HardwareStats& stOut)
    {
        std::ifstream procStat("/proc/stat");
        if (!procStat.is_open())
        {
            return;
        }

        std::string szLine;
        CpuTimeSample currentTotal;
        std::vector<CpuTimeSample> vCurrentCores;

        while (std::getline(procStat, szLine))
        {
            if (szLine.rfind("cpu", 0) != 0)
            {
                break;
            }

            std::istringstream iss(szLine);
            std::string cpuLabel;
            CpuTimeSample sample;

            iss >> cpuLabel >> sample.unUser >> sample.unNice >> sample.unSystem >> sample.unIdle
                >> sample.unIowait >> sample.unIrq >> sample.unSoftirq >> sample.unSteal;

            if (cpuLabel == "cpu")
            {
                currentTotal = sample;
            }
            else
            {
                vCurrentCores.push_back(sample);
            }
        }

        if (m_bHasPrevCpu)
        {
            uint64_t totalDelta = currentTotal.GetTotal() - m_stPrevTotalCpu.GetTotal();
            uint64_t activeDelta = currentTotal.GetActive() - m_stPrevTotalCpu.GetActive();

            if (totalDelta > 0)
            {
                stOut.fCpuTotalUsage = (static_cast<float>(activeDelta) / static_cast<float>(totalDelta)) * 100.0f;
            }

            size_t numCores = std::min(vCurrentCores.size(), m_vPrevCoreCpu.size());
            stOut.vCoreUsage.resize(numCores, 0.0f);

            for (size_t i = 0; i < numCores; ++i)
            {
                uint64_t coreTotalDelta = vCurrentCores[i].GetTotal() - m_vPrevCoreCpu[i].GetTotal();
                uint64_t coreActiveDelta = vCurrentCores[i].GetActive() - m_vPrevCoreCpu[i].GetActive();

                if (coreTotalDelta > 0)
                {
                    stOut.vCoreUsage[i] = (static_cast<float>(coreActiveDelta) / static_cast<float>(coreTotalDelta)) * 100.0f;
                }
            }
        }
        else
        {
            m_bHasPrevCpu = true;
        }

        m_stPrevTotalCpu = currentTotal;
        m_vPrevCoreCpu   = vCurrentCores;

        double load[3];
        if (getloadavg(load, 3) != -1)
        {
            stOut.fLoad1Min  = static_cast<float>(load[0]);
            stOut.fLoad5Min  = static_cast<float>(load[1]);
            stOut.fLoad15Min = static_cast<float>(load[2]);
        }
    }

    void SystemMetricsCollector::UpdateMemory(HardwareStats& stOut)
    {
        std::ifstream procMem("/proc/meminfo");
        if (!procMem.is_open())
        {
            return;
        }

        std::string szKey;
        uint64_t unValue;
        std::string szUnit;

        uint64_t unMemTotalKB = 0, unMemAvailableKB = 0;
        uint64_t unSwapTotalKB = 0, unSwapFreeKB = 0;

        while (procMem >> szKey >> unValue >> szUnit)
        {
            if (szKey == "MemTotal:") unMemTotalKB = unValue;
            else if (szKey == "MemAvailable:") unMemAvailableKB = unValue;
            else if (szKey == "SwapTotal:") unSwapTotalKB = unValue;
            else if (szKey == "SwapFree:") unSwapFreeKB = unValue;
        }

        if (unMemTotalKB > 0)
        {
            stOut.fRamTotalGB = static_cast<float>(unMemTotalKB) / (1024.0f * 1024.0f);
            stOut.fRamUsedGB  = static_cast<float>(unMemTotalKB - unMemAvailableKB) / (1024.0f * 1024.0f);
        }

        if (unSwapTotalKB > 0)
        {
            stOut.fSwapTotalGB = static_cast<float>(unSwapTotalKB) / (1024.0f * 1024.0f);
            stOut.fSwapUsedGB  = static_cast<float>(unSwapTotalKB - unSwapFreeKB) / (1024.0f * 1024.0f);
        }
    }

    void SystemMetricsCollector::UpdateThermals(HardwareStats& stOut)
    {
        std::error_code ec;
        std::string thermalBasePath = "/sys/class/thermal";
        if (!std::filesystem::exists(thermalBasePath, ec))
        {
            return;
        }

        for (const auto& entry : std::filesystem::directory_iterator(thermalBasePath, ec))
        {
            if (entry.path().filename().string().rfind("thermal_zone", 0) == 0)
            {
                std::ifstream typeFile(entry.path() / "type");
                std::ifstream tempFile(entry.path() / "temp");

                if (typeFile.is_open() && tempFile.is_open())
                {
                    std::string szType;
                    int32_t nRawTemp = 0;
                    typeFile >> szType;
                    tempFile >> nRawTemp;

                    float fTempC = static_cast<float>(nRawTemp) / 1000.0f;
                    if (szType.find("cpu") != std::string::npos || szType.find("x86_pkg_temp") != std::string::npos)
                    {
                        if (stOut.fCpuTempCelsius == 0.0f || fTempC > stOut.fCpuTempCelsius)
                        {
                            stOut.fCpuTempCelsius = fTempC;
                        }
                    }
                    else if (szType.find("gpu") != std::string::npos)
                    {
                        stOut.fGpuTempCelsius = fTempC;
                    }
                    else if (szType.find("board") != std::string::npos || szType.find("AO-therm") != std::string::npos)
                    {
                        stOut.fBoardTempCelsius = fTempC;
                    }
                }
            }
        }
    }

    void SystemMetricsCollector::UpdateGpu(HardwareStats& stOut)
    {
        // 1. Try Tegra Jetson sysfs GPU load
        std::ifstream tegraGpuLoad("/sys/devices/gpu.0/load");
        if (tegraGpuLoad.is_open())
        {
            int nLoadTenthPercent = 0;
            if (tegraGpuLoad >> nLoadTenthPercent)
            {
                stOut.fGpuUsagePercent = static_cast<float>(nLoadTenthPercent) / 10.0f;
                return;
            }
        }

        // 2. Try desktop Linux drm gpu_busy_percent
        std::ifstream drmGpuBusy("/sys/class/drm/card0/device/gpu_busy_percent");
        if (drmGpuBusy.is_open())
        {
            int nLoad = 0;
            if (drmGpuBusy >> nLoad)
            {
                stOut.fGpuUsagePercent = static_cast<float>(nLoad);
            }
        }
    }
}    // namespace tui
