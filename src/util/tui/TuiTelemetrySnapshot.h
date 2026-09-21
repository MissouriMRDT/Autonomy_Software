/******************************************************************************
 * @brief Plain struct holding thread-safe snapshot data for the Autonomy TUI.
 *
 * @file TuiTelemetrySnapshot.h
 * @author Missouri MRDT
 * @date 2026-09-21
 ******************************************************************************/

#ifndef TUI_TELEMETRY_SNAPSHOT_H
#define TUI_TELEMETRY_SNAPSHOT_H

#include <string>
#include <vector>
#include <cstdint>
#include "../../interfaces/State.hpp"

namespace tui
{
    struct TuiTelemetrySnapshot
    {
        // State Machine
        statemachine::States eCurrentState = statemachine::States::eIdle;
        std::string szStateName            = "Initializing...";
        int nCurrentWaypointID             = -1;
        double dMissionUptimeSeconds       = 0.0;
        bool bSimMode                      = false;

        // Navigation & Rover Pose
        double dEasting                    = 0.0;
        double dNorthing                   = 0.0;
        double dAltitude                   = 0.0;
        double dCompassHeading             = 0.0;
        double dTargetHeading              = 0.0;
        double dHeadingError               = 0.0;
        double dDistanceToWaypoint         = 0.0;

        // Drive System
        float fLeftDrivePower              = 0.0f;
        float fRightDrivePower             = 0.0f;
        float fSlopeMultiplier             = 1.0f;
        float fPitchAngle                  = 0.0f;
        float fRollAngle                   = 0.0f;

        // Vision & Detections
        float fMainCamFPS                  = 0.0f;
        float fRearCamFPS                  = 0.0f;
        int nDetectedTagsCount             = 0;
        int nBestTagID                     = -1;
        double dBestTagDistance            = 0.0;
        double dBestTagYaw                 = 0.0;
        int nDetectedObjectsCount          = 0;
        std::string szBestObjectClass      = "None";
        float fBestObjectConfidence        = 0.0f;
        double dBestObjectDistance         = 0.0;
        float fImuAccelX                   = 0.0f;
        float fImuAccelY                   = 0.0f;
        float fImuAccelZ                   = 0.0f;

        // GeoPlanner & LiDAR
        bool bLidarDBLoaded                = false;
        std::string szLidarDBPath          = "";
        int nPlannedPathWaypoints          = 0;
        int nObstacleCount                 = 0;
        int nCurrentTileX                  = 0;
        int nCurrentTileY                  = 0;

        // Network & Threads
        bool bRoveCommUDPOnline            = false;
        bool bRoveCommTCPOnline            = false;
        uint32_t nRoveCommUDPIPS           = 0;
        uint32_t nRoveCommTCPIPS           = 0;
        uint32_t nStateMachineIPS          = 0;
        uint32_t nMainProcessIPS           = 0;
        uint32_t nVizClientCount           = 0;

        // Hardware & System Metrics
        float fCpuTotalUsage               = 0.0f;
        std::vector<float> vPerCoreUsage;
        float fRamUsedGB                   = 0.0f;
        float fRamTotalGB                  = 0.0f;
        float fGpuUsagePercent             = 0.0f;
        float fVramUsedGB                  = 0.0f;
        float fVramTotalGB                 = 0.0f;
        float fCpuTempCelsius              = 0.0f;
        float fGpuTempCelsius              = 0.0f;
    };
}    // namespace tui

#endif    // TUI_TELEMETRY_SNAPSHOT_H
