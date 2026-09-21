/******************************************************************************
 * @brief Telemetry View (Tab 1) for Autonomy TUI using FTXUI.
 *
 * @file TelemetryView.h
 * @author Missouri MRDT
 * @date 2026-09-21
 ******************************************************************************/

#ifndef TELEMETRY_VIEW_H
#define TELEMETRY_VIEW_H

#include "../TuiTelemetrySnapshot.h"
#include <ftxui/dom/elements.hpp>
#include <iomanip>
#include <sstream>

namespace tui::views
{
    using namespace ftxui;

    inline Element RenderTelemetryView(const TuiTelemetrySnapshot& snap)
    {
        // 1. Mission Header Banner
        Color stateColor = Color::Cyan;
        if (snap.eCurrentState == statemachine::States::eNavigating) stateColor = Color::Green;
        else if (snap.eCurrentState == statemachine::States::eApproachingMarker || snap.eCurrentState == statemachine::States::eApproachingObject) stateColor = Color::Yellow;
        else if (snap.eCurrentState == statemachine::States::eStuck) stateColor = Color::Red;

        int nHours = static_cast<int>(snap.dMissionUptimeSeconds) / 3600;
        int nMins  = (static_cast<int>(snap.dMissionUptimeSeconds) % 3600) / 60;
        int nSecs  = static_cast<int>(snap.dMissionUptimeSeconds) % 60;
        std::ostringstream ossUptime;
        ossUptime << std::setfill('0') << std::setw(2) << nHours << ":"
                  << std::setfill('0') << std::setw(2) << nMins << ":"
                  << std::setfill('0') << std::setw(2) << nSecs;

        Element headerBanner = hbox({
            text(" STATE: ") | bold,
            text(snap.szStateName) | bold | color(stateColor),
            separator(),
            text(snap.bSimMode ? " SIM MODE: ENABLED (RoveSoSim) " : " SIM MODE: DISABLED (Field) ") | color(snap.bSimMode ? Color::Yellow : Color::GreenLight),
            separator(),
            text(" UPTIME: " + ossUptime.str() + " ") | color(Color::White)
        }) | border | bgcolor(Color::RGB(20, 20, 30));

        // 2. Pane 1: Nav & Rover Pose
        std::ostringstream ossEasting, ossNorthing, ossAlt, ossLat, ossLon, ossHeading, ossTarget, ossErr, ossDist;
        ossEasting << std::fixed << std::setprecision(2) << snap.dEasting;
        ossNorthing << std::fixed << std::setprecision(2) << snap.dNorthing;
        ossAlt << std::fixed << std::setprecision(1) << snap.dAltitude;
        ossLat << std::fixed << std::setprecision(6) << std::abs(snap.dLatitude);
        ossLon << std::fixed << std::setprecision(6) << std::abs(snap.dLongitude);
        ossHeading << std::fixed << std::setprecision(1) << snap.dCompassHeading;
        ossTarget << std::fixed << std::setprecision(1) << snap.dTargetHeading;
        ossErr << std::fixed << std::setprecision(1) << snap.dHeadingError;
        ossDist << std::fixed << std::setprecision(1) << snap.dDistanceToWaypoint;

        std::string szUtmZone = (snap.nUTMZone > 0 ? std::to_string(snap.nUTMZone) + (snap.bUTMNorth ? "N" : "S") : "N/A");
        std::string szGpsStr = (snap.bHasGPSFix || (snap.dLatitude != 0.0 || snap.dLongitude != 0.0))
            ? (ossLat.str() + "° " + (snap.dLatitude >= 0 ? "N" : "S") + ", " + ossLon.str() + "° " + (snap.dLongitude >= 0 ? "E" : "W"))
            : "No GNSS Fix";

        float fLeftClamped = std::clamp((snap.fLeftDrivePower + 1.0f) * 0.5f, 0.0f, 1.0f);
        float fRightClamped = std::clamp((snap.fRightDrivePower + 1.0f) * 0.5f, 0.0f, 1.0f);

        Element navPane = window(
            text(" NAV / ROVER POSE ") | bold | color(Color::CyanLight),
            vbox({
                hbox({text(" UTM: ") | bold | color(Color::White), text(szUtmZone) | bold | color(Color::CyanLight), text(" | E: ") | color(Color::GrayLight), text(ossEasting.str() + "m") | color(Color::YellowLight), text(" N: ") | color(Color::GrayLight), text(ossNorthing.str() + "m") | color(Color::YellowLight), text(" Alt: ") | color(Color::GrayLight), text(ossAlt.str() + "m") | color(Color::White)}),
                hbox({text(" GPS: ") | bold | color(Color::White), text(szGpsStr) | bold | color(snap.bHasGPSFix ? Color::GreenLight : Color::YellowLight)}),
                hbox({text(" Compass Yaw: ") | color(Color::GrayLight), text(ossHeading.str() + "°") | bold | color(Color::White), text(" (Target: " + ossTarget.str() + "°)") | color(Color::GrayLight)}),
                hbox({text(" Heading Err: ") | color(Color::GrayLight), text((snap.dHeadingError >= 0 ? "+" : "") + ossErr.str() + "°") | bold | color(std::abs(snap.dHeadingError) < 5.0 ? Color::Green : Color::RedLight), text(" | Dist: " + ossDist.str() + " m") | color(Color::White)}),
                separator(),
                text(" Drive Effort (L / R):") | color(Color::GrayLight),
                hbox({text(" L: ") | color(Color::GrayLight), gauge(fLeftClamped) | color(Color::BlueLight), text(" " + std::to_string(static_cast<int>(snap.fLeftDrivePower * 100)) + "%")}),
                hbox({text(" R: ") | color(Color::GrayLight), gauge(fRightClamped) | color(Color::BlueLight), text(" " + std::to_string(static_cast<int>(snap.fRightDrivePower * 100)) + "%")})
            })
        ) | flex;

        // 3. Pane 2: Vision & Sensors
        std::ostringstream ossTagDist, ossTagYaw, ossObjDist, ossObjConf;
        ossTagDist << std::fixed << std::setprecision(2) << snap.dBestTagDistance;
        ossTagYaw << std::fixed << std::setprecision(1) << snap.dBestTagYaw;
        ossObjDist << std::fixed << std::setprecision(2) << snap.dBestObjectDistance;
        ossObjConf << std::fixed << std::setprecision(2) << snap.fBestObjectConfidence;

        Element mainCamStatus = snap.bMainCamOpen
            ? (text("[ONLINE / " + std::to_string(static_cast<int>(snap.fMainCamFPS)) + " FPS]") | bold | color(snap.fMainCamFPS > 20 ? Color::Green : Color::YellowLight))
            : (text("[OFFLINE / 0 FPS]") | bold | color(Color::RedLight));

        Element rearCamStatus = snap.bRearCamOpen
            ? (text("[ONLINE / " + std::to_string(static_cast<int>(snap.fRearCamFPS)) + " FPS]") | bold | color(snap.fRearCamFPS > 20 ? Color::Green : Color::YellowLight))
            : (text("[OFFLINE / 0 FPS]") | bold | color(Color::RedLight));

        Element tagDetectorStatus = snap.bTagDetectorReady
            ? (text("[ACTIVE]") | bold | color(Color::GreenLight))
            : (text("[OFFLINE]") | bold | color(Color::RedLight));

        Element objDetectorStatus = snap.bObjectDetectorReady
            ? (text("[ACTIVE]") | bold | color(Color::GreenLight))
            : (text("[OFFLINE]") | bold | color(Color::RedLight));

        Element visionPane = window(
            text(" VISION & SENSORS ") | bold | color(Color::MagentaLight),
            vbox({
                hbox({text(" Front ZED: ") | color(Color::GrayLight), mainCamStatus, text("  Rear ZED: ") | color(Color::GrayLight), rearCamStatus}),
                hbox({text(" Tag Det:   ") | color(Color::GrayLight), tagDetectorStatus, text("  Obj Det:  ") | color(Color::GrayLight), objDetectorStatus}),
                separator(),
                text(" Tag Detections: " + std::to_string(snap.nDetectedTagsCount)) | bold | color(Color::White),
                snap.nBestTagID != -1
                    ? hbox({text(" • Tag ID #" + std::to_string(snap.nBestTagID) + " | Dist: " + ossTagDist.str() + "m | Yaw: " + ossTagYaw.str() + "°") | color(Color::GreenLight)})
                    : text(" • No target tags in view") | color(Color::GrayLight),
                separator(),
                text(" Object Detections: " + std::to_string(snap.nDetectedObjectsCount)) | bold | color(Color::White),
                snap.szBestObjectClass != "None"
                    ? hbox({text(" • " + snap.szBestObjectClass + " (Conf: " + ossObjConf.str() + ", Dist: " + ossObjDist.str() + "m)") | color(Color::YellowLight)})
                    : text(" • No target objects detected") | color(Color::GrayLight)
            })
        ) | flex;

        // 4. Pane 3: GeoPlanner & LiDAR
        Element geoPane = window(
            text(" GEOPLANNER / LIDAR ") | bold | color(Color::YellowLight),
            vbox({
                hbox({text(" Database: ") | color(Color::GrayLight), text(snap.bLidarDBLoaded ? "LOADED (" + snap.szLidarDBPath + ")" : "NOT LOADED") | color(snap.bLidarDBLoaded ? Color::Green : Color::RedLight)}),
                hbox({text(" Active Tile: [X: ") | color(Color::GrayLight), text(std::to_string(snap.nCurrentTileX)) | bold, text(", Y: ") | color(Color::GrayLight), text(std::to_string(snap.nCurrentTileY)) | bold, text("]")}),
                hbox({text(" Planned Path: ") | color(Color::GrayLight), text(std::to_string(snap.nPlannedPathWaypoints) + " waypoints") | color(Color::CyanLight)}),
                hbox({text(" Obstacles in FOV: ") | color(Color::GrayLight), text(std::to_string(snap.nObstacleCount) + " clusters") | color(snap.nObstacleCount > 0 ? Color::YellowLight : Color::GreenLight)})
            })
        ) | flex;

        // 5. Pane 4: Network & Threads
        Element netPane = window(
            text(" NETWORK & THREADS ") | bold | color(Color::GreenLight),
            vbox({
                hbox({text(" RoveComm UDP: ") | color(Color::GrayLight), text(snap.bRoveCommUDPOnline ? "ONLINE" : "OFFLINE") | bold | color(snap.bRoveCommUDPOnline ? Color::Green : Color::RedLight), text(" (" + std::to_string(snap.nRoveCommUDPIPS) + " pkt/s)") | color(Color::GrayLight)}),
                hbox({text(" RoveComm TCP: ") | color(Color::GrayLight), text(snap.bRoveCommTCPOnline ? "ONLINE" : "OFFLINE") | bold | color(snap.bRoveCommTCPOnline ? Color::Green : Color::RedLight), text(" (" + std::to_string(snap.nRoveCommTCPIPS) + " pkt/s)") | color(Color::GrayLight)}),
                hbox({text(" Viz Clients:   ") | color(Color::GrayLight), text(std::to_string(snap.nVizClientCount)) | color(Color::CyanLight)}),
                hbox({text(" StateMachine: ") | color(Color::GrayLight), text(std::to_string(snap.nStateMachineIPS) + " Hz") | color(Color::White), text(" | Main: " + std::to_string(snap.nMainProcessIPS) + " Hz") | color(Color::GrayLight)})
            })
        ) | flex;

        // Combine into 2x2 Grid with Header
        return vbox({
            headerBanner,
            hbox({navPane, visionPane}) | flex,
            hbox({geoPane, netPane}) | flex
        }) | flex;
    }
}    // namespace tui::views

#endif    // TELEMETRY_VIEW_H
