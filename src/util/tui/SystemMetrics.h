/******************************************************************************
 * @brief System hardware metrics collector for Autonomy TUI (CPU, RAM, GPU, Thermals).
 *
 * @file SystemMetrics.h
 * @author Missouri MRDT
 * @date 2026-09-21
 ******************************************************************************/

#ifndef SYSTEM_METRICS_H
#define SYSTEM_METRICS_H

#include <vector>
#include <string>
#include <cstdint>

namespace tui
{
    struct CpuTimeSample
    {
        uint64_t unUser    = 0;
        uint64_t unNice    = 0;
        uint64_t unSystem  = 0;
        uint64_t unIdle    = 0;
        uint64_t unIowait  = 0;
        uint64_t unIrq     = 0;
        uint64_t unSoftirq = 0;
        uint64_t unSteal   = 0;

        uint64_t GetTotal() const
        {
            return unUser + unNice + unSystem + unIdle + unIowait + unIrq + unSoftirq + unSteal;
        }

        uint64_t GetActive() const
        {
            return unUser + unNice + unSystem + unIrq + unSoftirq + unSteal;
        }
    };

    struct HardwareStats
    {
        float fCpuTotalUsage        = 0.0f;
        std::vector<float> vCoreUsage;
        float fLoad1Min             = 0.0f;
        float fLoad5Min             = 0.0f;
        float fLoad15Min            = 0.0f;

        float fRamUsedGB            = 0.0f;
        float fRamTotalGB           = 0.0f;
        float fSwapUsedGB           = 0.0f;
        float fSwapTotalGB          = 0.0f;

        float fGpuUsagePercent      = 0.0f;
        float fVramUsedGB           = 0.0f;
        float fVramTotalGB          = 0.0f;
        std::string szGpuModel      = "";

        float fCpuTempCelsius       = 0.0f;
        float fGpuTempCelsius       = 0.0f;
        float fBoardTempCelsius     = 0.0f;
        bool bIsVirtualMachine      = false;
    };

    class SystemMetricsCollector
    {
    public:
        SystemMetricsCollector();
        ~SystemMetricsCollector();

        HardwareStats Query();

    private:
        void UpdateCpu(HardwareStats& stOut);
        void UpdateMemory(HardwareStats& stOut);
        void UpdateThermals(HardwareStats& stOut);
        void UpdateGpu(HardwareStats& stOut);

        CpuTimeSample m_stPrevTotalCpu;
        std::vector<CpuTimeSample> m_vPrevCoreCpu;
        bool m_bHasPrevCpu = false;
        void* m_pNvml      = nullptr;
    };
}    // namespace tui

#endif    // SYSTEM_METRICS_H
