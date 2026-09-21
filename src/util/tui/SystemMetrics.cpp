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
#include <dlfcn.h>
#endif

namespace tui
{
    struct NvmlContext
    {
        void* pLib = nullptr;
        typedef int (*FnInit)();
        typedef int (*FnShutdown)();
        typedef int (*FnGetHandle)(unsigned int, void**);
        typedef int (*FnGetName)(void*, char*, unsigned int);
        typedef int (*FnGetTemp)(void*, int, unsigned int*);
        struct nvmlUtil { unsigned int gpu; unsigned int memory; };
        typedef int (*FnGetUtil)(void*, nvmlUtil*);
        struct nvmlMem { unsigned long long total; unsigned long long free; unsigned long long used; };
        typedef int (*FnGetMem)(void*, nvmlMem*);

        FnInit fnInit = nullptr;
        FnShutdown fnShutdown = nullptr;
        FnGetHandle fnGetHandle = nullptr;
        FnGetName fnGetName = nullptr;
        FnGetTemp fnGetTemp = nullptr;
        FnGetUtil fnGetUtil = nullptr;
        FnGetMem fnGetMem = nullptr;
        bool bInitialized = false;

        NvmlContext()
        {
#if defined(__linux__)
            pLib = dlopen("libnvidia-ml.so.1", RTLD_NOW);
            if (!pLib)
            {
                pLib = dlopen("libnvidia-ml.so", RTLD_NOW);
            }
            if (pLib)
            {
                fnInit     = (FnInit)dlsym(pLib, "nvmlInit_v2");
                if (!fnInit) fnInit = (FnInit)dlsym(pLib, "nvmlInit");
                fnShutdown = (FnShutdown)dlsym(pLib, "nvmlShutdown");
                fnGetHandle = (FnGetHandle)dlsym(pLib, "nvmlDeviceGetHandleByIndex_v2");
                if (!fnGetHandle) fnGetHandle = (FnGetHandle)dlsym(pLib, "nvmlDeviceGetHandleByIndex");
                fnGetName  = (FnGetName)dlsym(pLib, "nvmlDeviceGetName");
                fnGetTemp  = (FnGetTemp)dlsym(pLib, "nvmlDeviceGetTemperature");
                fnGetUtil  = (FnGetUtil)dlsym(pLib, "nvmlDeviceGetUtilizationRates");
                fnGetMem   = (FnGetMem)dlsym(pLib, "nvmlDeviceGetMemoryInfo");

                if (fnInit && fnGetHandle && fnGetTemp && fnGetUtil && fnGetMem)
                {
                    if (fnInit() == 0)
                    {
                        bInitialized = true;
                    }
                }
            }
#endif
        }

        ~NvmlContext()
        {
#if defined(__linux__)
            if (bInitialized && fnShutdown)
            {
                fnShutdown();
            }
            if (pLib)
            {
                dlclose(pLib);
            }
#endif
        }
    };

    SystemMetricsCollector::SystemMetricsCollector()
    {
        m_pNvml = new NvmlContext();
        HardwareStats dummy;
        UpdateCpu(dummy);
    }

    SystemMetricsCollector::~SystemMetricsCollector()
    {
        if (m_pNvml)
        {
            delete static_cast<NvmlContext*>(m_pNvml);
            m_pNvml = nullptr;
        }
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
        // Detect virtualization environment
        std::ifstream osRelease("/proc/sys/kernel/osrelease");
        if (osRelease.is_open())
        {
            std::string szRel;
            osRelease >> szRel;
            if (szRel.find("WSL") != std::string::npos || szRel.find("microsoft") != std::string::npos)
            {
                stOut.bIsVirtualMachine = true;
            }
        }
        std::ifstream cpuInfo("/proc/cpuinfo");
        if (cpuInfo.is_open())
        {
            std::string szLine;
            while (std::getline(cpuInfo, szLine))
            {
                if (szLine.find("hypervisor") != std::string::npos)
                {
                    stOut.bIsVirtualMachine = true;
                    break;
                }
            }
        }

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
                        if (stOut.fGpuTempCelsius == 0.0f)
                        {
                            stOut.fGpuTempCelsius = fTempC;
                        }
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
        if (m_pNvml)
        {
            NvmlContext* pCtx = static_cast<NvmlContext*>(m_pNvml);
            if (pCtx->bInitialized && pCtx->fnGetHandle)
            {
                void* pDevice = nullptr;
                if (pCtx->fnGetHandle(0, &pDevice) == 0 && pDevice)
                {
                    if (pCtx->fnGetName)
                    {
                        char szName[128] = {0};
                        if (pCtx->fnGetName(pDevice, szName, sizeof(szName)) == 0)
                        {
                            stOut.szGpuModel = std::string(szName);
                        }
                    }

                    if (pCtx->fnGetTemp)
                    {
                        unsigned int unTemp = 0;
                        if (pCtx->fnGetTemp(pDevice, 0, &unTemp) == 0)
                        {
                            stOut.fGpuTempCelsius = static_cast<float>(unTemp);
                        }
                    }

                    if (pCtx->fnGetUtil)
                    {
                        NvmlContext::nvmlUtil util = {};
                        if (pCtx->fnGetUtil(pDevice, &util) == 0)
                        {
                            stOut.fGpuUsagePercent = static_cast<float>(util.gpu);
                        }
                    }

                    if (pCtx->fnGetMem)
                    {
                        NvmlContext::nvmlMem mem = {};
                        if (pCtx->fnGetMem(pDevice, &mem) == 0)
                        {
                            stOut.fVramUsedGB  = static_cast<float>(mem.used) / (1024.0f * 1024.0f * 1024.0f);
                            stOut.fVramTotalGB = static_cast<float>(mem.total) / (1024.0f * 1024.0f * 1024.0f);
                        }
                    }
                    return;
                }
            }
        }

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
