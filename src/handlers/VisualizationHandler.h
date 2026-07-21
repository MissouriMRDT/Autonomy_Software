/******************************************************************************
 * @brief Defines the VisualizationHandler class.
 *
 * @file VisualizationHandler.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2026-01-20
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#ifndef VISUALIZATIONHANDLER_H
#define VISUALIZATIONHANDLER_H

#include "../handlers/LiDARHandler.h"
#include "../interfaces/AutonomyThread.hpp"
#include "../util/GeospatialOperations.hpp"
#include "../util/handlers/SimpleWebServer.h"

/// \cond
#include <atomic>
#include <mutex>
#include <string>
#include <tracy/Tracy.hpp>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief The VisualizationHandler class manages persistent world state and
 * hosts a 3D web server for interactive visualization.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2026-01-20
 ******************************************************************************/
class VisualizationHandler : public AutonomyThread<void>
{
    public:
        ////////////////////////////////////
        // Declare class methods.
        ////////////////////////////////////
        VisualizationHandler(int nPort = 8080);
        ~VisualizationHandler();
        void SaveVisualization(const std::string& szFilename);

    private:
        ////////////////////////////////////
        // Declare and define private structs
        ////////////////////////////////////

        /******************************************************************************
         * @brief Struct representing a single point in the path history.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-01-22
         ******************************************************************************/
        struct DisplayPoint
        {
            public:
                float fX, fY, fZ;    // RELATIVE to m_stOriginUTM.
                float fScore;        // Traversal Score.
                int nState;          // Robot State at this point.
        };

        /******************************************************************************
         * @brief Struct representing a waypoint or goal beacon for visualization.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-01-22
         ******************************************************************************/
        struct DisplayWaypoint
        {
            public:
                float fX, fY, fZ;    // RELATIVE to m_stOriginUTM.
                int nType;           // Type of waypoint/obstacle.
        };

        /******************************************************************************
         * @brief Struct representing a persistent detection for visualization.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-01-22
         ******************************************************************************/
        struct DisplayDetection
        {
            public:
                float fX, fY, fZ;    // RELATIVE to m_stOriginUTM.
                int nType;           // 10=Tag, 11=Mallet, 12=Bottle, 13=Pick
        };

        ////////////////////////////////////
        // Private Methods
        ////////////////////////////////////
        void UpdatePathHistory(const geoops::UTMCoordinate& stRoverUTM);
        void UpdatePlannedPath();
        void UpdateWaypoints();
        void UpdateGoalBeacons(const geoops::UTMCoordinate& stRoverUTM);
        void UpdateDetections();
        void UpdatePointCloud();

        // API.
        std::vector<char> OnRequestTelemetry(const std::string& szQuery);
        std::vector<char> OnRequestMap(const std::string& szQuery);
        std::vector<char> OnRequestPlannedPath(const std::string& szQuery);
        std::vector<char> OnRequestWaypoints(const std::string& szQuery);
        std::vector<char> OnRequestDetections(const std::string& szQuery);
        std::vector<char> OnRequestDetectionList(const std::string& szQuery);
        std::vector<char> OnRequestPointCloud(const std::string& szQuery);

        // Utilities.
        std::vector<char> LoadFileToBuffer(const std::string& szPath);
        std::string Base64Encode(const std::vector<char>& vData);

        // HTML Generators.
        std::string GenerateStaticHtml(const std::vector<LiDARHandler::PointRow>& vLidar);

        // Internals
        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;

        ////////////////////////////////////
        // Private Members
        ////////////////////////////////////

        // Web Server.
        std::unique_ptr<SimpleWebServer> m_pWebServer;
        int m_nPort;
        // Origin.
        geoops::UTMCoordinate m_stOriginUTM;
        bool m_bOriginSet;
        // Path History.
        std::vector<DisplayPoint> m_vPathHistory;
        std::mutex m_muPathMutex;
        // Planned Path.
        std::vector<DisplayPoint> m_vPlannedPath;
        std::mutex m_muPlannedPathMutex;
        // Waypoints and Obstacles.
        std::vector<DisplayWaypoint> m_vWaypoints;
        std::mutex m_muWaypointMutex;
        // Goal Beacons.
        std::vector<DisplayWaypoint> m_vGoalBeacons;
        std::mutex m_muGoalBeaconMutex;
        // Detections.
        std::vector<DisplayDetection> m_vDetections;
        std::mutex m_muDetectionMutex;
        // Point Cloud
        cv::Mat m_cvFrontPointCloud, m_cvRearPointCloud;
        std::mutex m_muPointCloudMutex;
};
#endif
