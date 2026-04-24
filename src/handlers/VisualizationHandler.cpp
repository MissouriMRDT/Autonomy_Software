/******************************************************************************
 * @brief Implements the VisualizationHandler class.
 *
 * @file VisualizationHandler.cpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2026-01-20
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#include "./VisualizationHandler.h"
#include "../AutonomyGlobals.h"
#include "../handlers/StateMachineHandler.h"
#include "../util/states/ObjectDetectionChecker.hpp"
#include "../util/states/TagDetectionChecker.hpp"

/// \cond
#include <chrono>
#include <fstream>
#include <iomanip>

/// \endcond

/******************************************************************************
 * @brief Construct a new Visualization Handler:: Visualization Handler object.
 *
 * @param nPort - The port to host the web server on.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
VisualizationHandler::VisualizationHandler(int nPort)
{
    // Initialize member variables.
    m_nPort      = nPort;
    m_bOriginSet = false;

    // Start Web Server.
    m_pWebServer = std::make_unique<SimpleWebServer>(m_nPort);

    // Serve the detections folder as static files from the current logging session
    // This allows access via http://ip:port/detections/filename.png
    std::string szDetectionsPath = logging::g_szLoggingOutputPath + "detections";
    m_pWebServer->AddStaticDirectory("/detections", szDetectionsPath);

    // Register Asset Endpoints.
    m_pWebServer->RegisterEndpoint("/lib/three.js", std::bind(&VisualizationHandler::OnRequestLibThree, this, std::placeholders::_1));
    m_pWebServer->RegisterEndpoint("/lib/orbit.js", std::bind(&VisualizationHandler::OnRequestLibOrbit, this, std::placeholders::_1));

    // Register Data Endpoints.
    m_pWebServer->SetHtmlContent(this->GetEmbeddedHtml());
    m_pWebServer->RegisterEndpoint("/api/telemetry", std::bind(&VisualizationHandler::OnRequestTelemetry, this, std::placeholders::_1));
    m_pWebServer->RegisterEndpoint("/api/map", std::bind(&VisualizationHandler::OnRequestMap, this, std::placeholders::_1));
    m_pWebServer->RegisterEndpoint("/api/planned_path", std::bind(&VisualizationHandler::OnRequestPlannedPath, this, std::placeholders::_1));
    m_pWebServer->RegisterEndpoint("/api/waypoints", std::bind(&VisualizationHandler::OnRequestWaypoints, this, std::placeholders::_1));
    m_pWebServer->RegisterEndpoint("/api/detections", std::bind(&VisualizationHandler::OnRequestDetections, this, std::placeholders::_1));
    m_pWebServer->RegisterEndpoint("/api/detection_list", std::bind(&VisualizationHandler::OnRequestDetectionList, this, std::placeholders::_1));

    // Set main thread's max iteration rate.
    this->SetMainThreadIPSLimit(20);    // 20 Hz
}

/******************************************************************************
 * @brief Destroy the Visualization Handler:: Visualization Handler object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
VisualizationHandler::~VisualizationHandler()
{
    // Stop Web Server.
    m_pWebServer.reset();
}

/******************************************************************************
 * @brief The main continuous code for the VisualizationHandler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
void VisualizationHandler::ThreadedContinuousCode()
{
    // Check that required global handlers are valid.
    if (globals::g_pStateMachineHandler == nullptr || globals::g_pNavigationBoard == nullptr)
    {
        return;
    }

    // Retrieve Rover UTM Position.
    geoops::RoverPose stRoverPose    = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();
    geoops::UTMCoordinate stRoverUTM = stRoverPose.GetUTMCoordinate();

    // Set origin if not set.
    if (!m_bOriginSet)
    {
        // Only set origin if we have a valid position.
        if (std::abs(stRoverUTM.dEasting) > 1.0 || std::abs(stRoverUTM.dNorthing) > 1.0)
        {
            m_stOriginUTM = stRoverUTM;
            m_bOriginSet  = true;
            LOG_DEBUG(logging::g_qSharedLogger, "VisualizationHandler: Origin set to E:{}, N:{}", m_stOriginUTM.dEasting, m_stOriginUTM.dNorthing);
            LOG_INFO(logging::g_qSharedLogger, "VisualizationHandler: WebServer has been started on http://0.0.0.0:{}!", m_nPort);
        }
    }

    // Update Data if origin is set.
    if (m_bOriginSet)
    {
        this->UpdatePathHistory(stRoverUTM);
        this->UpdateGoalBeacons(stRoverUTM);
        this->UpdateDetections();

        static std::chrono::system_clock::time_point tmLastAuxUpdate = std::chrono::system_clock::now();
        // Update auxiliary data at 1 Hz.
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - tmLastAuxUpdate).count() >= 1)
        {
            this->UpdatePlannedPath();
            this->UpdateWaypoints();
            tmLastAuxUpdate = std::chrono::system_clock::now();
        }
    }
}

/******************************************************************************
 * @brief The pooled linear code for the VisualizationHandler. This is not used.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
void VisualizationHandler::PooledLinearCode() {}

/******************************************************************************
 * @brief Save the current visualization to a single HTML file with embedded assets.
 *
 * @param szFilename - The filename to save the visualization as.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
void VisualizationHandler::SaveVisualization(const std::string& szFilename)
{
    // Check that origin is set.
    if (!m_bOriginSet)
    {
        LOG_WARNING(logging::g_qSharedLogger, "VisualizationHandler: Cannot save, origin not set.");
        return;
    }

    // Submit logger message at start of saving process.
    LOG_INFO(logging::g_qSharedLogger, "VisualizationHandler: Saving visualization to {}...", szFilename);

    // Gather LiDAR Bounds based on Path History
    double dMinE = 1e9, dMaxE = -1e9, dMinN = 1e9, dMaxN = -1e9;

    {
        // Acquire lock to read path history.
        std::lock_guard<std::mutex> lkPathLock(m_muPathMutex);

        // If no path history, use current rover position.
        if (m_vPathHistory.empty())
        {
            geoops::RoverPose stPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();
            dMinE = dMaxE = stPose.GetUTMCoordinate().dEasting;
            dMinN = dMaxN = stPose.GetUTMCoordinate().dNorthing;
        }
        else
        {
            // Loop through each point in the path history to find extremes.
            for (const DisplayPoint& stPoint : m_vPathHistory)
            {
                double dAbsE = m_stOriginUTM.dEasting + stPoint.fX;
                double dAbsN = m_stOriginUTM.dNorthing + stPoint.fZ;
                if (dAbsE < dMinE)
                    dMinE = dAbsE;
                if (dAbsE > dMaxE)
                    dMaxE = dAbsE;
                if (dAbsN < dMinN)
                    dMinN = dAbsN;
                if (dAbsN > dMaxN)
                    dMaxN = dAbsN;
            }
        }
    }

    // Calculate LiDAR filter parameters.
    double dBuffer  = 60.0;
    double dCenterE = (dMinE + dMaxE) / 2.0;
    double dCenterN = (dMinN + dMaxN) / 2.0;
    double dRadius  = std::max(dMaxE - dMinE, dMaxN - dMinN) / 2.0 + dBuffer;
    std::vector<LiDARHandler::PointRow> vLidarPoints;
    // Check that global LiDAR handler is valid.
    if (globals::g_pLiDARHandler)
    {
        // Construct point filter.
        LiDARHandler::PointFilter stFilter;
        stFilter.dEasting  = dCenterE;
        stFilter.dNorthing = dCenterN;
        stFilter.dRadius   = dRadius;
        // Get LiDAR data from handler.
        vLidarPoints = globals::g_pLiDARHandler->GetLiDARData(stFilter);
    }

    // Generate HTML content (including embedded JS).
    std::string szHtml = GenerateStaticHtml(vLidarPoints);

    std::ofstream stdOutFile(szFilename);
    if (stdOutFile.is_open())
    {
        // Write file.
        stdOutFile << szHtml;
        stdOutFile.close();

        // Submit logger message that we're done saving.
        LOG_INFO(logging::g_qSharedLogger, "VisualizationHandler: Saved successfully.");
    }
    else
    {
        LOG_ERROR(logging::g_qSharedLogger, "VisualizationHandler: Failed to open file for writing.");
    }
}

/******************************************************************************
 * @brief Update the path history with the current rover position.
 *
 * @param stRoverUTM - The current UTM coordinate of the rover.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
void VisualizationHandler::UpdatePathHistory(const geoops::UTMCoordinate& stRoverUTM)
{
    // Acquire lock to update path history.
    std::lock_guard<std::mutex> lkPathLock(m_muPathMutex);

    // Add new point if moved more than 0.5 meters from last point.
    static geoops::UTMCoordinate stLastPos = {};
    if (std::hypot(stRoverUTM.dEasting - stLastPos.dEasting, stRoverUTM.dNorthing - stLastPos.dNorthing) > 0.5)
    {
        // Create new point, set its values, and append to history.
        DisplayPoint stPoint;
        stPoint.fX     = static_cast<float>(stRoverUTM.dEasting - m_stOriginUTM.dEasting);
        stPoint.fY     = static_cast<float>(stRoverUTM.dAltitude - m_stOriginUTM.dAltitude);
        stPoint.fZ     = static_cast<float>(stRoverUTM.dNorthing - m_stOriginUTM.dNorthing);
        stPoint.fScore = 0.0f;

        // Check if our global state machine handler is valid to get current state.
        if (globals::g_pStateMachineHandler)
        {
            // Set state.
            stPoint.nState = static_cast<int>(globals::g_pStateMachineHandler->GetCurrentState());
        }
        else
        {
            // Default to Idle state.
            stPoint.nState = 0;
        }

        // Append to history and update last position.
        m_vPathHistory.push_back(stPoint);
        stLastPos = stRoverUTM;
    }
}

/******************************************************************************
 * @brief Updates the planned path from the waypoint handler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
void VisualizationHandler::UpdatePlannedPath()
{
    // Check that global waypoint handler is valid.
    if (globals::g_pWaypointHandler == nullptr)
    {
        return;
    }

    // Retrieve raw planned path that the GeoPlanner put in the waypoint handler for us.
    // This may or may not exist or be the active path that the rover is following.
    std::vector<geoops::Waypoint> vRawPath = globals::g_pWaypointHandler->RetrievePath("GeoPlannerPath");

    // Acquire lock and update planned path.
    std::lock_guard<std::mutex> lkPathLock(m_muPlannedPathMutex);
    // Clear the old path and reserve space for the new path.
    m_vPlannedPath.clear();
    m_vPlannedPath.reserve(vRawPath.size());
    // Loop through raw path and convert to DisplayPoint format.
    for (const geoops::Waypoint& stWaypoint : vRawPath)
    {
        const geoops::UTMCoordinate& stUTMCoord = stWaypoint.GetUTMCoordinate();
        DisplayPoint stPoint;
        stPoint.fX     = static_cast<float>(stUTMCoord.dEasting - m_stOriginUTM.dEasting);
        stPoint.fY     = static_cast<float>(stUTMCoord.dAltitude - m_stOriginUTM.dAltitude);
        stPoint.fZ     = static_cast<float>(stUTMCoord.dNorthing - m_stOriginUTM.dNorthing);
        stPoint.fScore = 0.0f;
        m_vPlannedPath.push_back(stPoint);
    }
}

/******************************************************************************
 * @brief Updates the waypoints from the waypoint handler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
void VisualizationHandler::UpdateWaypoints()
{
    // Check that global waypoint handler is valid.
    if (globals::g_pWaypointHandler == nullptr)
    {
        return;
    }

    // Retrieve all waypoints and obstacles from the WaypointHandler.
    std::vector<geoops::Waypoint> vTargets   = globals::g_pWaypointHandler->GetAllWaypoints();
    std::vector<geoops::Waypoint> vObstacles = globals::g_pWaypointHandler->GetAllObstacles();

    // Acquire a lock before updating waypoints list and clear existing waypoints.
    std::lock_guard<std::mutex> lkWaypointLock(m_muWaypointMutex);

    // Clear old waypoints from list.
    m_vWaypoints.clear();
    // Helper to process waypoints and obstacles.
    std::function ProcessList = [&](const std::vector<geoops::Waypoint>& vWaypoints)
    {
        // Loop through all the given waypoints and add them to this classes waypoints list.
        for (const geoops::Waypoint& stWaypoint : vWaypoints)
        {
            const geoops::UTMCoordinate& stUTMCoord = stWaypoint.GetUTMCoordinate();
            DisplayWaypoint stPoint;
            stPoint.fX    = static_cast<float>(stUTMCoord.dEasting - m_stOriginUTM.dEasting);
            stPoint.fY    = static_cast<float>(stUTMCoord.dAltitude - m_stOriginUTM.dAltitude);
            stPoint.fZ    = static_cast<float>(stUTMCoord.dNorthing - m_stOriginUTM.dNorthing);
            stPoint.nType = static_cast<int>(stWaypoint.eType);
            m_vWaypoints.push_back(stPoint);
        }
    };

    // Use our utility function to process and add the waypoints and objects from the WaypointHandler to the member variables.
    ProcessList(vTargets);
    ProcessList(vObstacles);
}

/******************************************************************************
 * @brief Updates the persistent detections for visualization.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
void VisualizationHandler::UpdateDetections()
{
    // Check that required global handlers are valid.
    if (globals::g_pTagDetectionHandler == nullptr || globals::g_pObjectDetectionHandler == nullptr)
    {
        return;
    }

    // Helper to process detections
    std::function ProcessDetection = [&](float fX, float fY, float fZ, int nType)
    {
        // Acquire lock to update detections.
        std::lock_guard<std::mutex> lkDetectionsLock(m_muDetectionMutex);

        // Deduplication.
        for (DisplayDetection& stExistingDetection : m_vDetections)
        {
            // Same type check
            if (stExistingDetection.nType == nType)
            {
                // Calculate the distance between existing detection and new detection.
                float dist = std::hypot(stExistingDetection.fX - fX, stExistingDetection.fZ - fZ);
                // If close to another detection of the same type, update the existing location.
                if (dist < 1.5f)
                {
                    // Update the existing detection's position to be the average of the two detections.
                    stExistingDetection.fX = (stExistingDetection.fX + fX) / 2.0f;
                    stExistingDetection.fY = (stExistingDetection.fY + fY) / 2.0f;
                    stExistingDetection.fZ = (stExistingDetection.fZ + fZ) / 2.0f;

                    return;
                }
            }
        }

        // Construct a new detection and add it to our member list.
        DisplayDetection stDetection;
        stDetection.fX    = fX;
        stDetection.fY    = fY;
        stDetection.fZ    = fZ;
        stDetection.nType = nType;
        m_vDetections.push_back(stDetection);
    };

    // Prepare Detector Vectors
    std::vector<std::shared_ptr<TagDetector>> vTagDetectors    = {globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam),
                                                                  globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eRearCam)};

    std::vector<std::shared_ptr<ObjectDetector>> vObjDetectors = {
        globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eHeadMainCam),
        globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eRearCam)};

    /////////////////////////////////////////
    // Tags.
    /////////////////////////////////////////

    // Get the best Aruco and Torch tags.
    tagdetectutils::ArucoTag stBestAruco;
    tagdetectutils::ArucoTag stBestTorchTag;
    statemachine::IdentifyTargetMarker(vTagDetectors, stBestAruco, stBestTorchTag);
    // Process Aruco Tag if valid.
    if (stBestAruco.nID != -1 && stBestAruco.stGeolocatedPosition.eType != geoops::WaypointType::eUNKNOWN)
    {
        geoops::UTMCoordinate stUTM = stBestAruco.stGeolocatedPosition.GetUTMCoordinate();
        float fX                    = static_cast<float>(stUTM.dEasting - m_stOriginUTM.dEasting);
        float fY                    = static_cast<float>(stUTM.dAltitude - m_stOriginUTM.dAltitude);
        float fZ                    = static_cast<float>(stUTM.dNorthing - m_stOriginUTM.dNorthing);
        ProcessDetection(fX, fY, fZ, 10);
    }
    // Process Torch Tag if valid.
    if (stBestTorchTag.nID != -1 && stBestTorchTag.stGeolocatedPosition.eType != geoops::WaypointType::eUNKNOWN)
    {
        geoops::UTMCoordinate stUTM = stBestTorchTag.stGeolocatedPosition.GetUTMCoordinate();
        float fX                    = static_cast<float>(stUTM.dEasting - m_stOriginUTM.dEasting);
        float fY                    = static_cast<float>(stUTM.dAltitude - m_stOriginUTM.dAltitude);
        float fZ                    = static_cast<float>(stUTM.dNorthing - m_stOriginUTM.dNorthing);
        ProcessDetection(fX, fY, fZ, 10);
    }

    /////////////////////////////////////////
    // Mallets, Bottles, Picks.
    /////////////////////////////////////////

    // Get the best detected object.
    objectdetectutils::Object stObject;
    statemachine::IdentifyTargetObject(vObjDetectors, stObject);
    // Process object if valid.
    if (stObject.dConfidence > 0.6 && stObject.stGeolocatedPosition.eType != geoops::WaypointType::eUNKNOWN)
    {
        geoops::UTMCoordinate stUTM = stObject.stGeolocatedPosition.GetUTMCoordinate();
        float fX                    = static_cast<float>(stUTM.dEasting - m_stOriginUTM.dEasting);
        float fY                    = static_cast<float>(stUTM.dAltitude - m_stOriginUTM.dAltitude);
        float fZ                    = static_cast<float>(stUTM.dNorthing - m_stOriginUTM.dNorthing);

        // Determine type based on detection type.
        int nType = 0;
        switch (stObject.eDetectionType)
        {
            case objectdetectutils::ObjectDetectionType::eMallet: nType = 11; break;
            case objectdetectutils::ObjectDetectionType::eWaterBottle: nType = 12; break;
            case objectdetectutils::ObjectDetectionType::eRockPick: nType = 13; break;
            default: break;
        }

        // If we have a type, process the detection.
        if (nType != 0)
        {
            ProcessDetection(fX, fY, fZ, nType);
        }
    }
}

/******************************************************************************
 * @brief Updates the persistent goal beacons for visualization.
 *
 * @param stRoverUTM - The current UTM coordinate of the rover.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
void VisualizationHandler::UpdateGoalBeacons(const geoops::UTMCoordinate& stRoverUTM)
{
    // Check that global multimedia board is valid.
    if (globals::g_pMultimediaBoard == nullptr)
    {
        return;
    }

    // If we are in the ReachedGoal lighting state, add a persistent goal beacon at our current location.
    if (globals::g_pMultimediaBoard->GetCurrentLightingState() == MultimediaBoard::MultimediaBoardLightingState::eReachedGoal)
    {
        // Acquire lock to update goal beacons.
        std::lock_guard<std::mutex> lkBeaconLock(m_muGoalBeaconMutex);
        // Calculate relative position.
        float fCurX = static_cast<float>(stRoverUTM.dEasting - m_stOriginUTM.dEasting);
        float fCurY = static_cast<float>(stRoverUTM.dAltitude - m_stOriginUTM.dAltitude);
        float fCurZ = static_cast<float>(stRoverUTM.dNorthing - m_stOriginUTM.dNorthing);

        // Check for deduplication (within 2 meters).
        bool bAdd = true;
        if (!m_vGoalBeacons.empty())
        {
            // Calculate distance to last added beacon.
            const DisplayWaypoint& stLast = m_vGoalBeacons.back();
            float fDistance               = std::hypot(fCurX - stLast.fX, fCurZ - stLast.fZ);
            // If the distance is less than 2 meters, do not add.
            if (fDistance < 2.0f)
            {
                bAdd = false;
            }
        }

        // Check if our last beacon is in the same location.
        if (bAdd)
        {
            // Construct beacon.
            DisplayWaypoint stWaypoint;
            stWaypoint.fX    = fCurX;
            stWaypoint.fY    = fCurY;
            stWaypoint.fZ    = fCurZ;
            stWaypoint.nType = 8;
            m_vGoalBeacons.push_back(stWaypoint);

            // Submit logger message.
            LOG_DEBUG(logging::g_qSharedLogger, "VisualizationHandler: Added persistent Goal Beacon at current location.");
        }
    }
}

/******************************************************************************
 * @brief Handles telemetry requests from the web server.
 *
 * @param szQuery - The query string from the request.
 * @return std::vector<char> - The binary response data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
std::vector<char> VisualizationHandler::OnRequestTelemetry(const std::string& szQuery)
{
    (void) szQuery;

    // Acquire resource lock for reading the path.
    std::lock_guard<std::mutex> lkPathLock(m_muPathMutex);

    // Calculate buffer size.
    // Header = Pose(3f) + Heading(1f) + LPower(1f) + RPower(1f) + Count(1u)
    size_t siPathBytes = m_vPathHistory.size() * 4 * sizeof(float);
    size_t siHeader    = (6 * sizeof(float)) + (1 * sizeof(uint32_t));
    std::vector<char> vBuffer;
    vBuffer.reserve(siHeader + siPathBytes);

    // Create a util function for adding a float and to the buffer.
    std::function PushFloat = [&](float fFloat)
    {
        const char* pStart = reinterpret_cast<const char*>(&fFloat);
        vBuffer.insert(vBuffer.end(), pStart, pStart + sizeof(float));
    };
    // Create a util function for adding a uint32_t to the buffer.
    std::function PushUint = [&](uint32_t unInt32)
    {
        const char* pStart = reinterpret_cast<const char*>(&unInt32);
        vBuffer.insert(vBuffer.end(), pStart, pStart + sizeof(uint32_t));
    };

    // Push Rover Pose.
    geoops::RoverPose stPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();
    if (m_bOriginSet)
    {
        PushFloat(static_cast<float>(stPose.GetUTMCoordinate().dEasting - m_stOriginUTM.dEasting));
        PushFloat(static_cast<float>(stPose.GetUTMCoordinate().dAltitude - m_stOriginUTM.dAltitude));
        PushFloat(static_cast<float>(stPose.GetUTMCoordinate().dNorthing - m_stOriginUTM.dNorthing));
    }
    else
    {
        PushFloat(0.0f);
        PushFloat(0.0f);
        PushFloat(0.0f);
    }

    // Push Compass Heading.
    PushFloat(static_cast<float>(stPose.GetCompassHeading()));

    // Push Drive Powers.
    diffdrive::DrivePowers stPowers = {0.0, 0.0};
    if (globals::g_pDriveBoard)
    {
        stPowers = globals::g_pDriveBoard->GetDrivePowers();
    }
    PushFloat(static_cast<float>(stPowers.dLeftDrivePower));
    PushFloat(static_cast<float>(stPowers.dRightDrivePower));

    // Push Path History size.
    PushUint(static_cast<uint32_t>(m_vPathHistory.size()));
    // Push each point in the path history.
    for (const DisplayPoint& pt : m_vPathHistory)
    {
        PushFloat(pt.fX);
        PushFloat(pt.fY);
        PushFloat(pt.fZ);
        PushFloat(static_cast<float>(pt.nState));
    }

    return vBuffer;
}

/******************************************************************************
 * @brief Handles planned path requests from the web server.
 *
 * @param szQuery - The query string from the request.
 * @return std::vector<char> - The binary response data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
std::vector<char> VisualizationHandler::OnRequestPlannedPath(const std::string& szQuery)
{
    (void) szQuery;

    // Acquire lock to read planned path.
    std::lock_guard<std::mutex> lk(m_muPlannedPathMutex);

    // Prepare buffer.
    std::vector<char> vBuffer;
    size_t siBytes = sizeof(uint32_t) + (m_vPlannedPath.size() * 3 * sizeof(float));
    vBuffer.reserve(siBytes);

    // Insert number of points.
    uint32_t unCount   = static_cast<uint32_t>(m_vPlannedPath.size());
    const char* pCount = reinterpret_cast<const char*>(&unCount);
    vBuffer.insert(vBuffer.end(), pCount, pCount + sizeof(uint32_t));

    // Loop through each of the points in the planned path and add them to the buffer to be sent to the client.
    for (const DisplayPoint& stPoint : m_vPlannedPath)
    {
        const float data[3] = {stPoint.fX, stPoint.fY, stPoint.fZ};
        const char* pStart  = reinterpret_cast<const char*>(data);
        vBuffer.insert(vBuffer.end(), pStart, pStart + sizeof(data));
    }

    return vBuffer;
}

/******************************************************************************
 * @brief Handles waypoint requests from the web server.
 *
 * @param szQuery - The query string from the request.
 * @return std::vector<char> - The binary response data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
std::vector<char> VisualizationHandler::OnRequestWaypoints(const std::string& szQuery)
{
    (void) szQuery;

    // Acquire locks to read waypoints and goal beacons.
    std::lock_guard<std::mutex> lkWaypointLock(m_muWaypointMutex);
    std::lock_guard<std::mutex> lkBeaconLock(m_muGoalBeaconMutex);

    // Get the total number of points.
    size_t siTotalPoints = m_vWaypoints.size() + m_vGoalBeacons.size();

    // Prepare buffer.
    std::vector<char> vBuffer;
    size_t siBytes = sizeof(uint32_t) + (siTotalPoints * (3 * sizeof(float) + sizeof(int)));
    vBuffer.reserve(siBytes);
    // Insert number of points.
    uint32_t unCount   = static_cast<uint32_t>(siTotalPoints);
    const char* pStart = reinterpret_cast<const char*>(&unCount);
    vBuffer.insert(vBuffer.end(), pStart, pStart + sizeof(uint32_t));

    // Helper to push a DisplayWaypoint to the buffer.
    std::function PushPoint = [&](const DisplayWaypoint& stWaypoint)
    {
        const char* pX = reinterpret_cast<const char*>(&stWaypoint.fX);
        vBuffer.insert(vBuffer.end(), pX, pX + sizeof(float));

        const char* pY = reinterpret_cast<const char*>(&stWaypoint.fY);
        vBuffer.insert(vBuffer.end(), pY, pY + sizeof(float));

        const char* pZ = reinterpret_cast<const char*>(&stWaypoint.fZ);
        vBuffer.insert(vBuffer.end(), pZ, pZ + sizeof(float));

        const char* pT = reinterpret_cast<const char*>(&stWaypoint.nType);
        vBuffer.insert(vBuffer.end(), pT, pT + sizeof(int));
    };

    // Loop through each of the waypoints and goal beacons and add them to the buffer.
    for (const DisplayWaypoint& stWaypoint : m_vWaypoints)
    {
        PushPoint(stWaypoint);
    }
    for (const DisplayWaypoint& stWaypoint : m_vGoalBeacons)
    {
        PushPoint(stWaypoint);
    }

    return vBuffer;
}

/******************************************************************************
 * @brief Handles detection requests from the web server.
 *
 * @param szQuery - The query string from the request.
 * @return std::vector<char> - The binary response data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
std::vector<char> VisualizationHandler::OnRequestDetections(const std::string& szQuery)
{
    (void) szQuery;

    // Acquire lock to read detections.
    std::lock_guard<std::mutex> lkDetectionsLock(m_muDetectionMutex);

    // Prepare buffer.
    std::vector<char> vBuffer;
    size_t siBytes = sizeof(uint32_t) + (m_vDetections.size() * (3 * sizeof(float) + sizeof(int)));
    vBuffer.reserve(siBytes);
    // Insert number of detections.
    uint32_t unCount   = static_cast<uint32_t>(m_vDetections.size());
    const char* pStart = reinterpret_cast<const char*>(&unCount);
    vBuffer.insert(vBuffer.end(), pStart, pStart + sizeof(uint32_t));

    // Loop through each of the detections and add them to the buffer.
    for (const DisplayDetection& stDetection : m_vDetections)
    {
        // Insert position data.
        const float aData[3] = {stDetection.fX, stDetection.fY, stDetection.fZ};
        const char* pStart   = reinterpret_cast<const char*>(aData);
        vBuffer.insert(vBuffer.end(), pStart, pStart + sizeof(aData));
        // Insert type data.
        const char* pTypeStart = reinterpret_cast<const char*>(&stDetection.nType);
        vBuffer.insert(vBuffer.end(), pTypeStart, pTypeStart + sizeof(int));
    }

    return vBuffer;
}

/******************************************************************************
 * @brief Handles detection list requests from the web server.
 *
 * @param szQuery - The query string from the request.
 * @return std::vector<char> - The binary response data.
 *
 * @author Targed (ltklionel@gmail.com)
 * @date 2026-01-30
 ******************************************************************************/
std::vector<char> VisualizationHandler::OnRequestDetectionList(const std::string& szQuery)
{
    (void) szQuery;
    std::string szJson = "[";
    std::string szDir  = logging::g_szLoggingOutputPath + "/detections/";

    // Helper function to escape JSON strings
    std::function<std::string(const std::string&)> fnEscapeJson = [](const std::string& szInput) -> std::string
    {
        std::string szOutput;
        szOutput.reserve(szInput.size());
        for (char c : szInput)
        {
            switch (c)
            {
                case '"': szOutput += "\\\""; break;
                case '\\': szOutput += "\\\\"; break;
                case '\b': szOutput += "\\b"; break;
                case '\f': szOutput += "\\f"; break;
                case '\n': szOutput += "\\n"; break;
                case '\r': szOutput += "\\r"; break;
                case '\t': szOutput += "\\t"; break;
                default:
                    if (static_cast<unsigned char>(c) < 0x20)
                    {
                        // Escape control characters
                        char szBuf[7];
                        snprintf(szBuf, sizeof(szBuf), "\\u%04x", static_cast<unsigned char>(c));
                        szOutput += szBuf;
                    }
                    else
                    {
                        szOutput += c;
                    }
                    break;
            }
        }
        return szOutput;
    };

    // Ensure the directory exists
    if (std::filesystem::exists(szDir) && std::filesystem::is_directory(szDir))
    {
        std::vector<std::filesystem::directory_entry> vEntries;

        // Iterate over files in the directory and collect them
        for (const std::filesystem::directory_entry& stEntry : std::filesystem::directory_iterator(szDir))
        {
            if (stEntry.is_regular_file())
            {
                // Get filename
                std::string szFilename = stEntry.path().filename().string();

                // Simple filter for image extensions
                if (szFilename.ends_with(".png") || szFilename.ends_with(".jpg"))
                {
                    vEntries.push_back(stEntry);
                }
            }
        }

        // Sort entries chronologically by last write time (oldest first, newest last)
        std::sort(vEntries.begin(),
                  vEntries.end(),
                  [](const std::filesystem::directory_entry& a, const std::filesystem::directory_entry& b)
                  { return std::filesystem::last_write_time(a) < std::filesystem::last_write_time(b); });

        bool bFirst = true;
        // Build JSON array using the sorted entries
        for (const std::filesystem::directory_entry& stEntry : vEntries)
        {
            std::string szFilename = stEntry.path().filename().string();

            if (!bFirst)
                szJson += ",";
            // Append escaped filename to JSON array
            szJson += "\"" + fnEscapeJson(szFilename) + "\"";
            bFirst = false;
        }
    }

    szJson += "]";

    // Convert string to vector
    return std::vector<char>(szJson.begin(), szJson.end());
}

/******************************************************************************
 * @brief Handles map data requests from the web server.
 *
 * @param szQuery - The query string from the request.
 * @return std::vector<char> - The binary response data.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
std::vector<char> VisualizationHandler::OnRequestMap(const std::string& szQuery)
{
    // Declare instance variables with default values.
    float fCenterX  = 0.0f;
    float fCenterY  = 0.0f;
    float fRadius   = 50.0f;
    float fMinScore = 0.0f;

    // Helper to parse float values from query string.
    std::function ParseVal = [&](std::string szKey) -> float
    {
        size_t siPos = szQuery.find(szKey + "=");
        if (siPos != std::string::npos)
        {
            // Extract substring value.
            size_t siEnd      = szQuery.find("&", siPos);
            std::string szSub = szQuery.substr(siPos + szKey.length() + 1, siEnd - (siPos + szKey.length() + 1));

            // Convert to float.
            try
            {
                return std::stof(szSub);
            }
            catch (...)
            {
                return 0.0f;
            }
        }

        return 0.0f;
    };

    // Parse values from query string.
    fCenterX  = ParseVal("x");
    fCenterY  = ParseVal("y");
    fRadius   = ParseVal("r");
    fMinScore = ParseVal("s");

    // Clamp radius to minimum of 10 meters.
    if (fRadius < 10.0f)
    {
        fRadius = 10.0f;
    }

    // Calculate absolute UTM coordinates for center point.
    double dAbsE         = m_stOriginUTM.dEasting + fCenterX;
    double dAbsN         = m_stOriginUTM.dNorthing + fCenterY;
    double dSearchRadius = fRadius * 1.5;

    // Construct LiDAR point filter.
    LiDARHandler::PointFilter stFilter;
    stFilter.dEasting  = dAbsE;
    stFilter.dNorthing = dAbsN;
    stFilter.dRadius   = dSearchRadius;
    // Retrieve LiDAR points from global LiDAR handler.
    std::vector<LiDARHandler::PointRow> vPoints;
    // Check that global LiDAR handler is valid.
    if (globals::g_pLiDARHandler)
    {
        // Get LiDAR data from handler.
        vPoints = globals::g_pLiDARHandler->GetLiDARData(stFilter);
    }

    // Prepare buffer.
    std::vector<char> vBuffer;
    size_t siBytes = sizeof(uint32_t) + (vPoints.size() * 4 * sizeof(float));
    vBuffer.reserve(siBytes);
    // Insert number of points placeholder.
    uint32_t unPtCount = 0;
    vBuffer.resize(sizeof(uint32_t));

    // Loop through each point and add to buffer if within radius and above min score.
    for (const LiDARHandler::PointRow& stPoint : vPoints)
    {
        // Check traversal score.
        if (stPoint.dTraversalScore < fMinScore)
        {
            continue;
        }

        // Calculate relative coordinates.
        float fRelX = static_cast<float>(stPoint.dEasting - m_stOriginUTM.dEasting);
        float fRelZ = static_cast<float>(stPoint.dNorthing - m_stOriginUTM.dNorthing);
        float fRelY = static_cast<float>(stPoint.dAltitude - m_stOriginUTM.dAltitude);

        // Check if within radius.
        if (std::abs(fRelX - fCenterX) <= fRadius && std::abs(fRelZ - fCenterY) <= fRadius)
        {
            unPtCount++;
            const float aData[4] = {fRelX, fRelY, fRelZ, static_cast<float>(stPoint.dTraversalScore)};
            const char* pStart   = reinterpret_cast<const char*>(aData);
            vBuffer.insert(vBuffer.end(), pStart, pStart + sizeof(aData));
        }
    }

    // Write actual point count to the start of the buffer.
    uint32_t* pHead = reinterpret_cast<uint32_t*>(vBuffer.data());
    *pHead          = unPtCount;

    return vBuffer;
}

/******************************************************************************
 * @brief Serves the local three.min.js file.
 *
 * @param szQuery - The query string (unused).
 * @return std::vector<char> - The binary content of the file.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-24
 ******************************************************************************/
std::vector<char> VisualizationHandler::OnRequestLibThree(const std::string& szQuery)
{
    (void) szQuery;
    return LoadFileToBuffer(constants::VISUALIZER_THREEJS_PATH);
}

/******************************************************************************
 * @brief Serves the local OrbitControls.js file.
 *
 * @param szQuery - The query string (unused).
 * @return std::vector<char> - The binary content of the file.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-24
 ******************************************************************************/
std::vector<char> VisualizationHandler::OnRequestLibOrbit(const std::string& szQuery)
{
    (void) szQuery;
    return LoadFileToBuffer(constants::VISUALIZER_ORBITCONTROLS_PATH);
}

/******************************************************************************
 * @brief Helper to load a file into a byte buffer.
 *
 * @param szPath - The file path.
 * @return std::vector<char> - The file content as a byte buffer.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-24
 ******************************************************************************/
std::vector<char> VisualizationHandler::LoadFileToBuffer(const std::string& szPath)
{
    std::ifstream stdFile(szPath, std::ios::binary);
    if (!stdFile.is_open())
    {
        LOG_ERROR(logging::g_qSharedLogger, "VisualizationHandler: Failed to load asset: {}", szPath);
        return {};
    }

    return std::vector<char>((std::istreambuf_iterator<char>(stdFile)), std::istreambuf_iterator<char>());
}

/******************************************************************************
 * @brief Encodes binary data to a Base64 string.
 *
 * @param vData - The binary data to encode.
 * @return std::string - The Base64 encoded string.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-24
 ******************************************************************************/
std::string VisualizationHandler::Base64Encode(const std::vector<char>& vData)
{
    // Create instance variables.
    const std::string szBase64Chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                      "abcdefghijklmnopqrstuvwxyz"
                                      "0123456789+/";
    std::string szReturnString;
    int nIter = 0;
    int nJter = 0;
    unsigned char aCharArray3[3];
    unsigned char aCharArray4[4];

    // Loop through each character in the data.
    for (char chCharacter : vData)
    {
        // Fill the 3-byte array.
        aCharArray3[nIter++] = chCharacter;

        // If we have 3 bytes, encode to 4 Base64 characters.
        if (nIter == 3)
        {
            // Convert to Base64.
            aCharArray4[0] = (aCharArray3[0] & 0xfc) >> 2;
            aCharArray4[1] = ((aCharArray3[0] & 0x03) << 4) + ((aCharArray3[1] & 0xf0) >> 4);
            aCharArray4[2] = ((aCharArray3[1] & 0x0f) << 2) + ((aCharArray3[2] & 0xc0) >> 6);
            aCharArray4[3] = aCharArray3[2] & 0x3f;
            // Append to return string.
            for (nIter = 0; (nIter < 4); nIter++)
            {
                szReturnString += szBase64Chars[aCharArray4[nIter]];
            }

            nIter = 0;
        }
    }

    // Handle padding for remaining bytes.
    if (nIter)
    {
        // Fill remaining bytes with zeros.
        for (nJter = nIter; nJter < 3; nJter++)
        {
            aCharArray3[nJter] = '\0';
        }
        // Convert to Base64.
        aCharArray4[0] = (aCharArray3[0] & 0xfc) >> 2;
        aCharArray4[1] = ((aCharArray3[0] & 0x03) << 4) + ((aCharArray3[1] & 0xf0) >> 4);
        aCharArray4[2] = ((aCharArray3[1] & 0x0f) << 2) + ((aCharArray3[2] & 0xc0) >> 6);
        aCharArray4[3] = aCharArray3[2] & 0x3f;
        // Append to return string.
        for (nJter = 0; (nJter < nIter + 1); nJter++)
        {
            szReturnString += szBase64Chars[aCharArray4[nJter]];
        }
        // Add padding '=' characters.
        while ((nIter++ < 3))
        {
            szReturnString += '=';
        }
    }

    return szReturnString;
}

/******************************************************************************
 * @brief Gets the embedded HTML for the visualization page.
 *
 * @return std::string - The HTML content as a string.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
std::string VisualizationHandler::GetEmbeddedHtml()
{
    return R"RAW_HTML(
<!DOCTYPE html>
<html>
<head>
    <title>MRDT 3D Visualizer</title>
    <style>
        body { margin: 0; overflow: hidden; background: #222; font-family: sans-serif; }
        #ui-layer {
            position: absolute;
            top: 10px;
            left: 10px;
            color: #0f0;
            background: rgba(0,0,0,0.5);
            padding: 10px;
            border-radius: 5px;
            pointer-events: none;
            min-width: 250px;
            z-index: 10;
        }
        #eta-box {
            position: absolute;
            top: 10px;
            left: 50%;
            transform: translateX(-50%);
            color: #0f0;
            background: rgba(0,0,0,0.5);
            padding: 10px;
            border-radius: 5px;
            font-size: 16px;
            font-weight: bold;
            pointer-events: none;
            z-index: 10;
        }
        #marker-layer {
            position: absolute;
            top: 0; left: 0; width: 100%; height: 100%;
            pointer-events: none;
            overflow: hidden;
        }
        .hud-marker {
            position: absolute;
            padding: 4px 8px;
            background: rgba(0, 0, 0, 0.7);
            color: white;
            border: 2px solid white;
            border-radius: 4px;
            font-size: 14px;
            font-weight: bold;
            white-space: nowrap;
            transform: translate(-50%, -50%);
            transition: opacity 0.2s;
        }
        .hud-marker::after {
            content: '';
            position: absolute;
            top: 50%; left: 50%;
            width: 0; height: 0;
        }
        #legend-layer {
            position: absolute;
            bottom: 20px;
            left: 10px;
            color: #fff;
            background: rgba(0,0,0,0.7);
            padding: 10px;
            border-radius: 5px;
            pointer-events: none;
            min-width: 150px;
            z-index: 10;
        }
        .legend-section { margin-bottom: 10px; border-bottom: 1px solid #555; padding-bottom: 5px; }
        .legend-item { display: flex; align-items: center; gap: 10px; margin-bottom: 5px; font-size: 12px; }
        .color-box { width: 15px; height: 15px; border: 1px solid #aaa; }
        .circle-box { width: 12px; height: 12px; border-radius: 50%; }
        .control-group { margin-bottom: 10px; pointer-events: auto; }
        label { display: block; font-size: 12px; color: #aaa; }
        input[type=range] { width: 100%; }
        .val-disp { float: right; color: #fff; }
        .btn-group { position: absolute; bottom: 20px; right: 20px; display: flex; gap: 10px; z-index: 10; }
        .hud-btn { padding: 10px 20px; background: #444; color: white; border: 2px solid #666; cursor: pointer; font-size: 16px; z-index: 999; }
        .hud-btn.active { background: #00aa00; border-color: #00ff00; }
        .key { color: #fff; font-weight: bold; border: 1px solid #666; padding: 2px 5px; border-radius: 3px; background: #333; }
        h3 { margin-top: 0; border-bottom: 1px solid #555; padding-bottom: 5px; }
        
        #detection-panel { position: absolute; top: 10px; right: 10px; width: auto; background: rgba(0,0,0,0.7); padding: 10px; border-radius: 5px; z-index: 10; transition: width 0.2s; }
        #detection-panel h3 { color: #0f0; margin-top: 0; }
        .gallery-item { cursor: pointer; }
        .gallery-item img { width: 100%; border: 2px solid #666; border-radius: 4px; transition: border-color 0.2s; }
        .gallery-item img:hover { border-color: #0f0; }
        
        #detection-modal { display: none; position: fixed; z-index: 1000; left: 0; top: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.9); align-items: center; justify-content: center; }
        #detection-modal img { max-width: 90%; max-height: 90%; border: 3px solid #0f0; }
        #modal-caption { position: absolute; bottom: 80px; color: #0f0; font-size: 18px; text-align: center; width: 100%; }
        #modal-open-btn { position: absolute; bottom: 30px; left: 50%; transform: translateX(-50%); padding: 10px 30px; background: #444; color: white; border: 2px solid #0f0; cursor: pointer; font-size: 16px; border-radius: 5px; }
        #modal-open-btn:hover { background: #00aa00; }
        
        .modal-close { position: absolute; top: 20px; right: 40px; color: #fff; font-size: 40px; font-weight: bold; cursor: pointer; }
        .modal-close:hover { color: #0f0; }
        
        .modal-nav { position: absolute; top: 50%; transform: translateY(-50%); color: #fff; font-size: 60px; font-weight: bold; cursor: pointer; padding: 20px; user-select: none; z-index: 1001; transition: color 0.2s; }
        .modal-nav:hover { color: #0f0; }
        .modal-nav.left { left: 20px; }
        .modal-nav.right { right: 20px; }
    </style>
    <script type="importmap">
    { 
        "imports": { 
            "three": "/lib/three.js", 
            "three/addons/controls/OrbitControls.js": "/lib/orbit.js" 
        } 
    }
    </script>
</head>
<body>
    <div id="ui-layer">
        <h3 id="settings-toggle" style="cursor: pointer; pointer-events: auto; margin: 0; border: none; padding: 0; user-select: none;">Settings &#9654;</h3>
        <div id="settings-content" style="display: none; margin-top: 10px; border-top: 1px solid #555; padding-top: 10px;">
            <div class="control-group">
                <label>Load Radius (m) <span id="val-rad" class="val-disp">50</span></label>
                <input type="range" id="sl-rad" min="10" max="200" value="50" step="10">
            </div>
            <div class="control-group">
                <label>Border Tol. (m) <span id="val-tol" class="val-disp">10</span></label>
                <input type="range" id="sl-tol" min="5" max="50" value="10" step="1">
            </div>
            <div class="control-group">
                <label>Min Score <span id="val-score" class="val-disp">0.0</span></label>
                <input type="range" id="sl-score" min="0.0" max="1.0" value="0.0" step="0.05">
            </div>
            <div id="status" style="margin-top:10px; color: #fff;">Status: Free Cam</div>
            <div id="stats" style="margin-top:5px; color: #aaa; font-size:12px;">Points: 0</div>
        </div>
    </div>

    <div id="eta-box">ETA: Calculating...</div>

    <div id="legend-layer">
        <div class="legend-section" id="det-legend">
            <strong>Detections</strong>
        </div>
        <div class="legend-section" id="state-legend">
            <strong>State Key</strong>
        </div>
    </div>
    
    <div id="detection-panel">
        <h3>Latest Detection</h3>
        <div id="detection-gallery-items"></div>
    </div>
    
    <div id="detection-modal" onclick="closeDetectionModal()">
        <span class="modal-close" onclick="closeDetectionModal()">&times;</span>
        <div class="modal-nav left" onclick="prevDetection(event)">&#10094;</div>
        <img id="modal-image" src="" alt="Detection" onclick="event.stopPropagation()">
        <div class="modal-nav right" onclick="nextDetection(event)">&#10095;</div>
        <div id="modal-caption"></div>
        <button id="modal-open-btn" onclick="event.stopPropagation()">Open in New Tab</button>
    </div>
    
    <div id="marker-layer"></div>

    <div class="btn-group">
        <button id="snap-btn" class="hud-btn" onclick="snapToRover()">Snap (Space)</button>
        <button id="follow-btn" class="hud-btn" onclick="toggleFollow()">Follow (F)</button>
    </div>

<script type="module">
    import * as THREE from 'three';
    import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

    let camera, scene, renderer, controls, roverMesh, pathLine, plannedPathLine, currentPoints;
    let waypointGroup, detectionGroup; 
    let markerLayer; 
    let activeWaypoints = []; 
    let leftArrow, rightArrow;
    let beaconGeo, detectionTex;

    let mapCenter = { x: 0, y: 0 }; 
    let cfgRadius = 50;
    let cfgTolerance = 10;
    let cfgMinScore = 0.0;
    let isFetchingMap = false;
    let isFollowing = false;
    let targetPos = new THREE.Vector3();
    let targetHeading = 0.0;
    let prevRoverPos = new THREE.Vector3();
    const keys = { w:false, a:false, s:false, d:false, q:false, e:false, shift:false };
    let lastTime = performance.now();

    // ETA Variables
    let lastTelemetryTime = 0;
    let lastTelemetryPos = new THREE.Vector3();
    let avgSpeed = 0.0;
    let pathDistance = 0.0;
    const speedHistory = [];
    let lastPathPoint = null;
    
    // Detection Gallery Tracking
    let detectionFilenames = [];
    let currentModalIndex = 0;
    
    const typeColors = {};
    const typeNames = {};

    // State Colors
    const stateColors = {
        0: { name: "Idle", color: "#888888" },
        1: { name: "Navigating", color: "#00ffff" },
        2: { name: "Search Pattern", color: "#0000ff" },
        3: { name: "Approach Marker", color: "#ffffff" },
        4: { name: "Approach Object", color: "#ffaa00" },
        5: { name: "Verify Pos", color: "#00550e" },
        6: { name: "Verify Marker", color: "#06ac00" },
        7: { name: "Verify Object", color: "#78ff66" },
        8: { name: "Reversing", color: "#ff0000" },
        9: { name: "Stuck", color: "#330000" }
    };

    // Detection Colors
    const detectColors = {
        10: { name: "Tag (Aruco)", color: "#aa00ff" }, // Purple
        11: { name: "Mallet", color: "#ffa500" },      // Orange
        12: { name: "Bottle", color: "#0088ff" },      // Blue
        13: { name: "Pick", color: "#ffee00" }         // Yellow
    };

    init();
    animate();

    function init() {
        markerLayer = document.getElementById('marker-layer');
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x111111);
        scene.add(new THREE.GridHelper(100, 100));
        scene.add(new THREE.AxesHelper(2));
        
        camera = new THREE.PerspectiveCamera(60, window.innerWidth/window.innerHeight, 0.1, 10000);
        camera.position.set(0, 10, -10); 
        
        renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(window.innerWidth, window.innerHeight);
        document.body.appendChild(renderer.domElement);
        
        controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;

        const geometry = new THREE.BoxGeometry(1, 0.5, 1.5);
        const material = new THREE.MeshBasicMaterial({ color: 0xff00ff, wireframe: true });
        roverMesh = new THREE.Mesh(geometry, material);
        scene.add(roverMesh);
        
        // Drive Vectors (Arrows)
        const arrowDir = new THREE.Vector3(0, 0, -1);
        const arrowOrigin = new THREE.Vector3(0, 0, 0);
        const arrowLen = 1;
        const arrowCol = 0xffff00;
        leftArrow = new THREE.ArrowHelper(arrowDir, arrowOrigin, arrowLen, arrowCol);
        rightArrow = new THREE.ArrowHelper(arrowDir, arrowOrigin, arrowLen, arrowCol);
        roverMesh.add(leftArrow);
        roverMesh.add(rightArrow);
        leftArrow.position.set(-0.6, 0, 0); 
        rightArrow.position.set(0.6, 0, 0);

        beaconGeo = new THREE.BoxGeometry(0.5, 10000, 0.5);
        const canvas = document.createElement('canvas');
        canvas.width = 32; canvas.height = 32;
        const ctx = canvas.getContext('2d');
        ctx.beginPath();
        ctx.arc(16,16,14,0,2*Math.PI);
        ctx.fillStyle = 'white';
        ctx.fill();
        detectionTex = new THREE.CanvasTexture(canvas);

        waypointGroup = new THREE.Group();
        scene.add(waypointGroup);
        
        detectionGroup = new THREE.Group(); // New Group for detections
        scene.add(detectionGroup);

        const config = [
            { id: 0, name: "NAV", color: "#00ffff" },
            { id: 1, name: "TAG", color: "#ffffff" },
            { id: 2, name: "MALLET", color: "#ffa500" },
            { id: 3, name: "BOTTLE", color: "#0088ff" },
            { id: 4, name: "PICK", color: "#ffee00" },
            { id: 5, name: "OBJ", color: "#aaaaaa" },
            { id: 6, name: "OBSTACLE", color: "#ff0000" },
            { id: 7, name: "UNKNOWN", color: "#000000" },
            { id: 8, name: "GOAL REACHED", color: "#00ff00" }
        ];

        config.forEach(c => {
            typeNames[c.id] = c.name;
            typeColors[c.id] = new THREE.Color(c.color);
        });

        // Legend: Detections
        const detLegendDiv = document.getElementById('det-legend');
        for (const [id, data] of Object.entries(detectColors)) {
            const item = document.createElement('div');
            item.className = 'legend-item';
            item.innerHTML = `<div class="circle-box" style="background:${data.color}"></div><span>${data.name}</span>`;
            detLegendDiv.appendChild(item);
        }

        // Legend: States
        const stateLegendDiv = document.getElementById('state-legend');
        for (const [id, data] of Object.entries(stateColors)) {
            const item = document.createElement('div');
            item.className = 'legend-item';
            item.innerHTML = `<div class="color-box" style="background:${data.color}"></div><span>${data.name}</span>`;
            stateLegendDiv.appendChild(item);
        }

        document.getElementById('sl-rad').oninput = (e) => { 
            cfgRadius = parseInt(e.target.value); 
            document.getElementById('val-rad').innerText = cfgRadius;
            checkBoundary(true); 
        };
        document.getElementById('sl-tol').oninput = (e) => { 
            cfgTolerance = parseInt(e.target.value); 
            document.getElementById('val-tol').innerText = cfgTolerance;
        };
        document.getElementById('sl-score').oninput = (e) => { 
            cfgMinScore = parseFloat(e.target.value); 
            document.getElementById('val-score').innerText = cfgMinScore.toFixed(2);
            checkBoundary(true); 
        };

        // UI Layer Collapsible Toggle
        const settingsToggle = document.getElementById('settings-toggle');
        const settingsContent = document.getElementById('settings-content');
        settingsToggle.addEventListener('click', () => {
            if (settingsContent.style.display === 'none') {
                settingsContent.style.display = 'block';
                settingsToggle.innerHTML = 'Settings &#9660;';
            } else {
                settingsContent.style.display = 'none';
                settingsToggle.innerHTML = 'Settings &#9654;';
            }
        });

        window.addEventListener('keydown', (e) => onKey(e, true));
        window.addEventListener('keyup', (e) => onKey(e, false));
        window.addEventListener('resize', onWindowResize);
        
        window.toggleFollow = () => {
            isFollowing = !isFollowing;
            document.getElementById('follow-btn').classList.toggle('active', isFollowing);
            document.getElementById('status').innerText = isFollowing ? "Status: Locked" : "Status: Free Cam";
            if(isFollowing) controls.target.copy(roverMesh.position);
        };
        window.snapToRover = () => {
            camera.position.copy(roverMesh.position).add(new THREE.Vector3(0,10,-10));
            controls.target.copy(roverMesh.position);
        };

        requestTelemetryLoop();
        setInterval(fetchPlannedPath, 2000); 
        setInterval(fetchWaypoints, 2000); 
        setInterval(fetchDetections, 1000); // Poll detections every second
        setInterval(fetchDetectionsList, 5000);
        fetchMapSquare(0, 0);
    }

    // --- RECURSIVE LOOP ---
    async function requestTelemetryLoop() {
        if (!document.hidden) await fetchTelemetry();
        setTimeout(requestTelemetryLoop, 50);
    }

    async function fetchTelemetry() {
        try {
            const response = await fetch('/api/telemetry');
            if (!response.ok) return; 
            const buffer = await response.arrayBuffer();
            updateTelemetry(buffer);
        } catch (e) { }
    }

    async function fetchPlannedPath() {
        try {
            const response = await fetch('/api/planned_path');
            const buffer = await response.arrayBuffer();
            updatePlannedPath(buffer);
        } catch(e) {}
    }

    async function fetchWaypoints() {
        try {
            const response = await fetch('/api/waypoints');
            const buffer = await response.arrayBuffer();
            updateWaypoints(buffer);
        } catch(e) {}
    }

    async function fetchDetections() {
        try {
            const response = await fetch('/api/detections');
            const buffer = await response.arrayBuffer();
            updateDetections(buffer);
        } catch(e) {}
    }

    async function fetchDetectionsList() {
        try {
            const response = await fetch('/api/detection_list');
            const filenames = await response.json();
            updateDetectionGallery(filenames);
        } catch(e) {
            console.error('Failed to fetch detection list:', e);
        }
    }
    
    function updateDetectionGallery(filenames) {
        detectionFilenames = filenames;
        const panel = document.getElementById('detection-panel');
        const gallery = document.getElementById('detection-gallery-items');
        const header = panel ? panel.querySelector('h3') : null;
        if (!gallery) return;
        
        gallery.innerHTML = '';
        
        if (filenames.length === 0) {
            if (panel) panel.style.width = 'auto';
            if (header) header.style.display = 'none';
            gallery.innerHTML = '<div style="color:#aaa; font-size:12px; text-align:center;">No detections</div>';
            return;
        }

        if (panel) panel.style.width = '250px';
        if (header) header.style.display = 'block';

        // Get the latest detection (assumed to be the last one in the array)
        const latestIndex = filenames.length - 1;
        const filename = filenames[latestIndex];

        const item = document.createElement('div');
        item.className = 'gallery-item';
        
        const img = document.createElement('img');
        img.src = `/detections/${filename}`;
        img.alt = filename;
        img.title = "Click to view full gallery";
        img.addEventListener('click', () => showDetectionModal(latestIndex));
        
        const info = document.createElement('div');
        info.style.color = '#fff';
        info.style.fontSize = '14px';
        info.style.textAlign = 'center';
        info.style.marginTop = '8px';
        info.innerText = `View all ${filenames.length} images`;
        
        item.appendChild(img);
        item.appendChild(info);
        gallery.appendChild(item);
    }
    
    window.showDetectionModal = function(index) {
        const modal = document.getElementById('detection-modal');
        const img = document.getElementById('modal-image');
        const caption = document.getElementById('modal-caption');
        const openBtn = document.getElementById('modal-open-btn');
        
        if (modal && img && caption && detectionFilenames.length > 0) {
            currentModalIndex = index;
            const filename = detectionFilenames[currentModalIndex];
            const src = `/detections/${filename}`;

            img.src = src;
            caption.innerText = `${filename} (${currentModalIndex + 1} of ${detectionFilenames.length})`;
            
            if (openBtn) {
                openBtn.onclick = () => window.open(src, '_blank');
            }
            modal.style.display = 'flex';
        }
    }
    
    window.closeDetectionModal = function() {
        const modal = document.getElementById('detection-modal');
        if (modal) modal.style.display = 'none';
    }

    window.nextDetection = function(e) {
        e.stopPropagation();
        if (detectionFilenames.length === 0) return;
        let newIdx = currentModalIndex + 1;
        if (newIdx >= detectionFilenames.length) newIdx = 0;
        showDetectionModal(newIdx);
    }

    window.prevDetection = function(e) {
        e.stopPropagation();
        if (detectionFilenames.length === 0) return;
        let newIdx = currentModalIndex - 1;
        if (newIdx < 0) newIdx = detectionFilenames.length - 1;
        showDetectionModal(newIdx);
    }
    
    function updateArrow(arrow, power) {
        const absPwr = Math.abs(power);
        const dir = power >= 0 ? new THREE.Vector3(0, 0, -1) : new THREE.Vector3(0, 0, 1);
        arrow.setDirection(dir);
        arrow.setLength(Math.max(absPwr * 2.0, 0.001), 0.2, 0.1);
        const col = power >= 0 ? 0x00ff00 : 0xff0000;
        arrow.setColor(col);
    }

    function updateTelemetry(buffer) {
        const view = new DataView(buffer);
        const rx = view.getFloat32(0, true);
        const ry = view.getFloat32(4, true);
        const rz = view.getFloat32(8, true);
        const rh = view.getFloat32(12, true);

        // Calculate Speed
        const now = performance.now();
        const newPos = new THREE.Vector3(rx, ry, -rz);
        if (lastTelemetryTime > 0) {
            const dt = (now - lastTelemetryTime) / 1000.0;
            if (dt > 0.1) {
                const dist = newPos.distanceTo(lastTelemetryPos);
                const instSpeed = dist / dt;
                speedHistory.push(instSpeed);
                if (speedHistory.length > 20) speedHistory.shift();
                avgSpeed = speedHistory.reduce((a,b)=>a+b, 0) / speedHistory.length;
            }
        }
        lastTelemetryPos.copy(newPos);
        lastTelemetryTime = now;
        
        // Drive Powers
        const leftPwr = view.getFloat32(16, true);
        const rightPwr = view.getFloat32(20, true);
        updateArrow(leftArrow, leftPwr);
        updateArrow(rightArrow, rightPwr);
        
        targetPos.set(rx, ry, -rz); 
        targetHeading = -rh * (Math.PI / 180.0);

        checkBoundary(false);

        const pathCount = view.getUint32(24, true); // Offset 24
        if (pathCount > 0) {
            if (pathLine) {
                scene.remove(pathLine);
                if (pathLine.geometry) pathLine.geometry.dispose();
                if (pathLine.material) pathLine.material.dispose();
            }
            const floats = new Float32Array(buffer, 28, pathCount * 4); // Offset 28
            const vertices = [];
            const colors = [];
            const c = new THREE.Color();

            for(let i=0; i<floats.length; i+=4) {
                vertices.push(floats[i], floats[i+1], -floats[i+2]);
                const state = Math.floor(floats[i+3]);
                const hex = stateColors[state] ? stateColors[state].color : "#ffffff";
                c.set(hex);
                colors.push(c.r, c.g, c.b);
            }
            
            const pathGeo = new THREE.BufferGeometry();
            pathGeo.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
            pathGeo.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
            const mat = new THREE.LineBasicMaterial({ vertexColors: true, linewidth: 5 });
            pathLine = new THREE.Line(pathGeo, mat); 
            scene.add(pathLine);
        }
    }

    function updatePlannedPath(buffer) {
        const view = new DataView(buffer);
        const count = view.getUint32(0, true);
        if (plannedPathLine) {
            scene.remove(plannedPathLine);
            if (plannedPathLine.geometry) plannedPathLine.geometry.dispose();
            if (plannedPathLine.material) plannedPathLine.material.dispose();
            plannedPathLine = null;
        }

        pathDistance = 0.0;
        lastPathPoint = null;

        if (count > 0) {
            const floats = new Float32Array(buffer, 4, count * 3);
            const vertices = [];
            for(let i=0; i<floats.length; i+=3) {
                vertices.push(floats[i], floats[i+1], -floats[i+2]);
            }

            // Calculate total path distance (sum of segments)
            // Add distance from rover to first point
            if(vertices.length >= 3) {
                 const firstPt = new THREE.Vector3(vertices[0], vertices[1], vertices[2]);
                 pathDistance += targetPos.distanceTo(firstPt);
                 // Store Last Point
                 const lastIdx = vertices.length - 3;
                 lastPathPoint = new THREE.Vector3(vertices[lastIdx], vertices[lastIdx+1], vertices[lastIdx+2]);
            }
            // Add segments
            for(let i=0; i<vertices.length-3; i+=3) {
                const p1 = new THREE.Vector3(vertices[i], vertices[i+1], vertices[i+2]);
                const p2 = new THREE.Vector3(vertices[i+3], vertices[i+4], vertices[i+5]);
                pathDistance += p1.distanceTo(p2);
            }

            const geo = new THREE.BufferGeometry();
            geo.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
            plannedPathLine = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0xeeff00, linewidth: 6 }));
            scene.add(plannedPathLine);
        }
    }

    function updateWaypoints(buffer) {
        while(waypointGroup.children.length > 0){ 
            const child = waypointGroup.children[0];
            waypointGroup.remove(child); 
            if (child.material) child.material.dispose();
        }
        markerLayer.innerHTML = '';
        activeWaypoints = [];

        const view = new DataView(buffer);
        const count = view.getUint32(0, true);
        if(count === 0) return;

        let offset = 4;

        for(let i=0; i<count; i++) {
            const x = view.getFloat32(offset, true);
            const y = view.getFloat32(offset+4, true);
            const z = view.getFloat32(offset+8, true);
            const type = view.getInt32(offset+12, true);
            offset += 16;

            const col = typeColors[type] || typeColors[7]; 
            const beaconMat = new THREE.MeshBasicMaterial({
                color: col, 
                transparent: true, 
                opacity: 0.3,
                depthTest: false 
            });
            const beacon = new THREE.Mesh(beaconGeo, beaconMat);
            beacon.position.set(x, 0, -z);
            waypointGroup.add(beacon);

            const div = document.createElement('div');
            div.className = 'hud-marker';
            div.innerText = typeNames[type] || "UNK";
            div.style.borderColor = "#" + col.getHexString();
            markerLayer.appendChild(div);

            activeWaypoints.push({
                div: div,
                pos: new THREE.Vector3(x, 0, -z)
            });
        }
    }

    function updateDetections(buffer) {
        while(detectionGroup.children.length > 0){ 
            const child = detectionGroup.children[0];
            detectionGroup.remove(child); 
            if (child.geometry) child.geometry.dispose();
            if (child.material) child.material.dispose();
        }

        const view = new DataView(buffer);
        const count = view.getUint32(0, true);
        if(count === 0) return;

        let offset = 4;

        for(let i=0; i<count; i++) {
            const x = view.getFloat32(offset, true);
            const y = view.getFloat32(offset+4, true);
            const z = view.getFloat32(offset+8, true);
            const type = view.getInt32(offset+12, true);
            offset += 16;

            let col = "#ffffff";
            if(detectColors[type]) col = detectColors[type].color;

            const mat = new THREE.PointsMaterial({
                color: col,
                map: detectionTex,
                size: 2.0, // Large persistent dot
                sizeAttenuation: true,
                alphaTest: 0.5,
                transparent: true
            });
            const geo = new THREE.BufferGeometry();
            geo.setAttribute('position', new THREE.Float32BufferAttribute([x, y, -z], 3));
            
            const pt = new THREE.Points(geo, mat);
            detectionGroup.add(pt);
        }
    }

    function updateHUD() {
        const width = window.innerWidth;
        const height = window.innerHeight;
        const pad = 30; 

        // Update ETA Box
        const etaBox = document.getElementById('eta-box');
        
        // Reached End Logic
        let bReached = false;
        if (lastPathPoint && roverMesh.position.distanceTo(lastPathPoint) < 2.0) {
             bReached = true;
        }

        if (bReached) {
            etaBox.innerText = "Status: Reached End of Path";
            etaBox.style.color = "#00ff00"; // Green
        } else {
            etaBox.style.color = "#0f0"; // Default Green
            if (avgSpeed < 0.05) {
                etaBox.innerText = "ETA: Stopped";
            } else {
                const timeSec = pathDistance / avgSpeed;
                if (!isFinite(timeSec) || timeSec < 0) {
                     etaBox.innerText = "ETA: --:--";
                } else {
                     const min = Math.floor(timeSec / 60);
                     const sec = Math.floor(timeSec % 60);
                     etaBox.innerText = `ETA: ${min}m ${sec}s (${avgSpeed.toFixed(2)} m/s)`;
                }
            }
        }

        activeWaypoints.forEach(wp => {
            const target = wp.pos.clone();
            target.y = roverMesh.position.y + 3.0; 
            target.project(camera);

            let x = (target.x * .5 + .5) * width;
            let y = (target.y * -.5 + .5) * height;
            
            const isBehind = target.z > 1;

            if (isBehind) {
                x = width - x;
                y = height - y;
            }

            const cx = width / 2;
            const cy = height / 2;
            const dx = x - cx;
            const dy = y - cy;

            if (!isBehind && x >= pad && x <= width - pad && y >= pad && y <= height - pad) {
                y -= 40; 
                wp.div.style.opacity = "0.9";
            } else {
                let t = Infinity;
                if (dx > 0) t = Math.min(t, (width - pad - cx) / dx);
                if (dx < 0) t = Math.min(t, (pad - cx) / dx);
                if (dy > 0) t = Math.min(t, (height - pad - cy) / dy);
                if (dy < 0) t = Math.min(t, (pad - cy) / dy);

                x = cx + dx * t;
                y = cy + dy * t;
                wp.div.style.opacity = "0.6";
            }

            wp.div.style.left = x + 'px';
            wp.div.style.top = y + 'px';
            
            // Use 2D horizontal distance for distance display.
            const dxPos = roverMesh.position.x - wp.pos.x;
            const dzPos = roverMesh.position.z - wp.pos.z; 
            const dist = Math.sqrt(dxPos*dxPos + dzPos*dzPos);
            
            wp.div.innerText = `${wp.div.innerText.split(' ')[0]} ${Math.round(dist)}m`;
        });
    }

    function checkBoundary(force) {
        if(isFetchingMap && !force) return;
        const roverX = targetPos.x;
        const roverN = -targetPos.z; 
        const distE = Math.abs(roverX - mapCenter.x);
        const distN = Math.abs(roverN - mapCenter.y);
        const limit = cfgRadius - cfgTolerance;

        if (force || distE > limit || distN > limit) {
            fetchMapSquare(roverX, roverN);
        }
    }

    async function fetchMapSquare(x, y) {
        if(isFetchingMap) return;
        isFetchingMap = true;
        try {
            const url = `/api/map?x=${x.toFixed(2)}&y=${y.toFixed(2)}&r=${cfgRadius}&s=${cfgMinScore}`;
            const response = await fetch(url);
            const buffer = await response.arrayBuffer();
            loadMapPoints(buffer);
            mapCenter = { x: x, y: y };
        } catch(e) { console.error(e); }
        isFetchingMap = false;
    }

    function loadMapPoints(buffer) {
        const view = new DataView(buffer);
        const count = view.getUint32(0, true);
        
        if(currentPoints) {
            scene.remove(currentPoints);
            currentPoints.geometry.dispose();
            currentPoints.material.dispose();
            currentPoints = null;
        }

        if(count === 0) {
            document.getElementById('stats').innerText = "Points: 0";
            return;
        }

        const pts = [];
        const colors = [];
        const c = new THREE.Color();
        const floats = new Float32Array(buffer, 4, count * 4);

        for(let i=0; i<floats.length; i+=4) {
            pts.push(floats[i], floats[i+1], -floats[i+2]);
            c.setHSL(floats[i+3] * 0.33, 1.0, 0.5); 
            colors.push(c.r, c.g, c.b);
        }

        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
        geo.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
        const mat = new THREE.PointsMaterial({ size: 0.5, vertexColors: true });
        
        currentPoints = new THREE.Points(geo, mat);
        scene.add(currentPoints);
        document.getElementById('stats').innerText = "Points: " + count;
    }

    function onKey(e, p) { 
        if(keys.hasOwnProperty(e.key.toLowerCase())) keys[e.key.toLowerCase()] = p; 
        if(e.key === 'Shift') keys.shift = p;
        if(p && e.key==='f') window.toggleFollow();
        if(p && e.key===' ') window.snapToRover();
    }
    function onWindowResize() { camera.aspect = window.innerWidth / window.innerHeight; camera.updateProjectionMatrix(); renderer.setSize(window.innerWidth, window.innerHeight); }

    function animate() {
        requestAnimationFrame(animate);
        const now = performance.now();
        const dt = (now - lastTime)/1000;
        lastTime = now;

        prevRoverPos.copy(roverMesh.position);
        const lerpFactor = 5.0 * dt;
        roverMesh.position.lerp(targetPos, lerpFactor);
        const dRot = targetHeading - roverMesh.rotation.y;
        roverMesh.rotation.y += Math.atan2(Math.sin(dRot), Math.cos(dRot)) * lerpFactor;

        if(isFollowing) {
            camera.position.add(new THREE.Vector3().subVectors(roverMesh.position, prevRoverPos));
            controls.target.copy(roverMesh.position);
        } else {
            const spd = (keys.shift?15:5)*dt;
            const fwd = new THREE.Vector3(); camera.getWorldDirection(fwd); fwd.y=0; fwd.normalize();
            const rgt = new THREE.Vector3().crossVectors(fwd, camera.up).normalize();
            if(keys.w) camera.position.addScaledVector(fwd, spd);
            if(keys.s) camera.position.addScaledVector(fwd, -spd);
            if(keys.d) camera.position.addScaledVector(rgt, spd);
            if(keys.a) camera.position.addScaledVector(rgt, -spd);
            if(keys.q) camera.position.y += spd;
            if(keys.e) camera.position.y -= spd;
            controls.target.add(new THREE.Vector3(0,0,0).addScaledVector(fwd, (keys.w-keys.s)*spd).addScaledVector(rgt, (keys.d-keys.a)*spd));
        }
        controls.update();
        
        updateHUD();
        
        renderer.render(scene, camera);
    }
</script>
</body>
</html>
    )RAW_HTML";
}

/******************************************************************************
 * @brief Generates a static HTML page with embedded LiDAR data and embedded dependencies.
 *
 * @param vLidar - Vector of LiDAR point rows to include in the HTML.
 * @return std::string - The generated HTML content as a string.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
std::string VisualizationHandler::GenerateStaticHtml(const std::vector<LiDARHandler::PointRow>& vLidar)
{
    // 1. Load and Encode Assets
    std::string szThreeJS = Base64Encode(LoadFileToBuffer(constants::VISUALIZER_THREEJS_PATH));
    std::string szOrbitJS = Base64Encode(LoadFileToBuffer(constants::VISUALIZER_ORBITCONTROLS_PATH));

    if (szThreeJS.empty() || szOrbitJS.empty())
    {
        LOG_ERROR(logging::g_qSharedLogger, "VisualizationHandler: Cannot generate monolithic export. Missing assets.");
        return "<html><body>Error: Missing ThreeJS assets on rover. Ensure 'assets/three.module.js' and 'assets/OrbitControls.js' exist.</body></html>";
    }

    std::stringstream stdSS;
    stdSS << std::fixed << std::setprecision(3);

    // Write Header
    stdSS << R"RAW(
<!DOCTYPE html>
<html>
<head>
    <title>Mars Rover Static Export</title>
    <style>
        body { margin: 0; overflow: hidden; background: #222; font-family: sans-serif; }
        #ui-layer { position: absolute; top: 10px; left: 10px; color: #0f0; background: rgba(0,0,0,0.5); padding: 10px; border-radius: 5px; pointer-events: none; min-width: 250px; z-index: 10; }
        #marker-layer { position: absolute; top: 0; left: 0; width: 100%; height: 100%; pointer-events: none; overflow: hidden; }
        .hud-marker { position: absolute; padding: 4px 8px; background: rgba(0, 0, 0, 0.7); color: white; border: 2px solid white; border-radius: 4px; font-size: 14px; font-weight: bold; white-space: nowrap; transform: translate(-50%, -50%); transition: opacity 0.2s; }
        .hud-marker::after { content: ''; position: absolute; top: 50%; left: 50%; width: 0; height: 0; }
        #legend-layer { position: absolute; bottom: 20px; left: 10px; color: #fff; background: rgba(0,0,0,0.7); padding: 10px; border-radius: 5px; pointer-events: none; min-width: 150px; z-index: 10; }
        .legend-section { margin-bottom: 10px; border-bottom: 1px solid #555; padding-bottom: 5px; }
        .legend-item { display: flex; align-items: center; gap: 10px; margin-bottom: 5px; font-size: 12px; }
        .color-box { width: 15px; height: 15px; border: 1px solid #aaa; }
        .circle-box { width: 12px; height: 12px; border-radius: 50%; }
        .control-group { margin-bottom: 10px; pointer-events: auto; }
        label { display: block; font-size: 12px; color: #aaa; }
        input[type=range] { width: 100%; }
        .val-disp { float: right; color: #fff; }
        .btn-group { position: absolute; bottom: 20px; right: 20px; display: flex; gap: 10px; z-index: 10; }
        .hud-btn { padding: 10px 20px; background: #444; color: white; border: 2px solid #666; cursor: pointer; font-size: 16px; z-index: 999; }
        .hud-btn.active { background: #00aa00; border-color: #00ff00; }
        h3 { margin-top: 0; border-bottom: 1px solid #555; padding-bottom: 5px; }
    </style>
    <script type="importmap">
    { 
        "imports": { 
            "three": "data:text/javascript;base64,)RAW";

    // Inject ThreeJS Base64
    stdSS << szThreeJS;

    stdSS << R"RAW(", 
            "three/addons/controls/OrbitControls.js": "data:text/javascript;base64,)RAW";

    // Inject OrbitControls Base64
    stdSS << szOrbitJS;

    stdSS << R"RAW(" 
        } 
    }
    </script>
</head>
<body>
    <div id="ui-layer">
        <h3 id="settings-toggle" style="cursor: pointer; pointer-events: auto; margin: 0; border: none; padding: 0; user-select: none;">Static Export &#9654;</h3>
        <div id="settings-content" style="display: none; margin-top: 10px; border-top: 1px solid #555; padding-top: 10px;">
            <div class="control-group">
                <label>Min Score <span id="val-score" class="val-disp">0.0</span></label>
                <input type="range" id="sl-score" min="0.0" max="1.0" value="0.0" step="0.05">
            </div>
            <div id="stats" style="margin-top:5px; color: #aaa; font-size:12px;">Points: 0</div>
        </div>
    </div>
    <div id="legend-layer">
        <div class="legend-section" id="det-legend"><strong>Detections</strong></div>
        <div class="legend-section" id="state-legend"><strong>State Key</strong></div>
    </div>
    <div id="marker-layer"></div>
    <div class="btn-group">
        <button class="hud-btn" onclick="snapToRover()">Snap (Space)</button>
    </div>
)RAW";

    stdSS << "<script>\n";

    // LiDAR Data Loop.
    stdSS << "    const RAW_LIDAR = [";
    for (size_t siIter = 0; siIter < vLidar.size(); ++siIter)
    {
        const LiDARHandler::PointRow& stPoint = vLidar[siIter];
        stdSS << (stPoint.dEasting - m_stOriginUTM.dEasting) << "," << (stPoint.dAltitude - m_stOriginUTM.dAltitude) << ","
              << (stPoint.dNorthing - m_stOriginUTM.dNorthing) << "," << stPoint.dTraversalScore;
        if (siIter < vLidar.size() - 1)
            stdSS << ",";
    }
    stdSS << "];\n";

    // Path History Loop.
    {
        // Acquire lock for thread safety.
        std::lock_guard<std::mutex> lkPathLock(m_muPathMutex);
        stdSS << "    const RAW_PATH = [";

        // Loop through each point in the path history.
        for (size_t siIter = 0; siIter < m_vPathHistory.size(); ++siIter)
        {
            const DisplayPoint& stPoint = m_vPathHistory[siIter];
            stdSS << stPoint.fX << "," << stPoint.fY << "," << stPoint.fZ << "," << stPoint.nState;
            if (siIter < m_vPathHistory.size() - 1)
                stdSS << ",";
        }
        stdSS << "];\n";
    }

    // Planned Path Loop.
    {
        // Acquire lock for thread safety.
        std::lock_guard<std::mutex> lkPlannedPathLock(m_muPlannedPathMutex);
        stdSS << "    const RAW_PLANNED = [";

        // Loop through each point in the planned path.
        for (size_t siIter = 0; siIter < m_vPlannedPath.size(); ++siIter)
        {
            const DisplayPoint& stPoint = m_vPlannedPath[siIter];
            stdSS << stPoint.fX << "," << stPoint.fY << "," << stPoint.fZ;
            if (siIter < m_vPlannedPath.size() - 1)
                stdSS << ",";
        }
        stdSS << "];\n";
    }

    // Waypoints Loop.
    {
        // Acquire locks for thread safety.
        std::lock_guard<std::mutex> lkWaypointLock(m_muWaypointMutex);
        std::lock_guard<std::mutex> lkBeaconLock(m_muGoalBeaconMutex);
        stdSS << "    const RAW_WAYPOINTS = [";

        // Loop through each waypoint.
        for (const DisplayWaypoint& stWaypoint : m_vWaypoints)
        {
            stdSS << stWaypoint.fX << "," << stWaypoint.fY << "," << stWaypoint.fZ << "," << stWaypoint.nType << ",";
        }
        // Loop through each goal beacon.
        for (size_t siIter = 0; siIter < m_vGoalBeacons.size(); ++siIter)
        {
            const DisplayWaypoint& stBeacon = m_vGoalBeacons[siIter];
            stdSS << stBeacon.fX << "," << stBeacon.fY << "," << stBeacon.fZ << "," << stBeacon.nType;
            if (siIter < m_vGoalBeacons.size() - 1)
                stdSS << ",";
        }
        stdSS << "];\n";
    }

    // Detections Loop.
    {
        // Acquire lock for thread safety.
        std::lock_guard<std::mutex> lkDetectionLock(m_muDetectionMutex);
        stdSS << "    const RAW_DETECTIONS = [";

        // Loop through each detection.
        for (size_t siIter = 0; siIter < m_vDetections.size(); ++siIter)
        {
            const DisplayDetection& stDetection = m_vDetections[siIter];
            stdSS << stDetection.fX << "," << stDetection.fY << "," << stDetection.fZ << "," << stDetection.nType;
            if (siIter < m_vDetections.size() - 1)
                stdSS << ",";
        }
        stdSS << "];\n";
    }

    // Get Initial Rover Pose.
    geoops::RoverPose stPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();
    stdSS << "    const INIT_X = " << (stPose.GetUTMCoordinate().dEasting - m_stOriginUTM.dEasting) << ";\n";
    stdSS << "    const INIT_Y = " << (stPose.GetUTMCoordinate().dAltitude - m_stOriginUTM.dAltitude) << ";\n";
    stdSS << "    const INIT_Z = " << (stPose.GetUTMCoordinate().dNorthing - m_stOriginUTM.dNorthing) << ";\n";
    stdSS << "    const INIT_H = " << stPose.GetCompassHeading() << ";\n";
    stdSS << "</script>\n";

    // Write Logic Script directly to SS.
    stdSS << R"JS(
<script type="module">
    import * as THREE from 'three';
    import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

    let camera, scene, renderer, controls, roverMesh, pathLine, plannedPathLine, currentPoints;
    let waypointGroup, detectionGroup; 
    let markerLayer; 
    let activeWaypoints = []; 
    
    let cfgMinScore = 0.0;
    
    // Define keys object for static view
    const keys = { w:false, a:false, s:false, d:false, q:false, e:false, shift:false };

    const typeColors = {};
    const typeNames = {};

    const stateColors = {
        0: { name: "Idle", color: "#888888" },
        1: { name: "Navigating", color: "#00ffff" },
        2: { name: "Search Pattern", color: "#0000ff" },
        3: { name: "Approach Marker", color: "#ffffff" },
        4: { name: "Approach Object", color: "#ffaa00" },
        5: { name: "Verify Pos", color: "#00550e" },
        6: { name: "Verify Marker", color: "#06ac00" },
        7: { name: "Verify Object", color: "#78ff66" },
        8: { name: "Reversing", color: "#ff0000" },
        9: { name: "Stuck", color: "#330000" }
    };

    const detectColors = {
        10: { name: "Tag (Aruco)", color: "#aa00ff" },
        11: { name: "Mallet", color: "#ffa500" },
        12: { name: "Bottle", color: "#0088ff" },
        13: { name: "Pick", color: "#ffee00" }
    };

    init();
    animate();

    function init() {
        markerLayer = document.getElementById('marker-layer');
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x111111);
        scene.add(new THREE.GridHelper(100, 100));
        scene.add(new THREE.AxesHelper(2));
        
        camera = new THREE.PerspectiveCamera(60, window.innerWidth/window.innerHeight, 0.1, 10000);
        camera.position.set(INIT_X, INIT_Y + 10, -INIT_Z - 10); 
        
        renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(window.innerWidth, window.innerHeight);
        document.body.appendChild(renderer.domElement);
        
        controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.target.set(INIT_X, INIT_Y, -INIT_Z);

        const geometry = new THREE.BoxGeometry(1, 0.5, 1.5);
        const material = new THREE.MeshBasicMaterial({ color: 0xff00ff, wireframe: true });
        roverMesh = new THREE.Mesh(geometry, material);
        roverMesh.position.set(INIT_X, INIT_Y, -INIT_Z);
        roverMesh.rotation.y = -INIT_H * (Math.PI / 180.0);
        scene.add(roverMesh);

        waypointGroup = new THREE.Group();
        scene.add(waypointGroup);
        detectionGroup = new THREE.Group();
        scene.add(detectionGroup);

        const config = [
            { id: 0, name: "NAV", color: "#00ffff" },
            { id: 1, name: "TAG", color: "#ffffffff" },
            { id: 2, name: "MALLET", color: "#ffa500" },
            { id: 3, name: "BOTTLE", color: "#0088ff" },
            { id: 4, name: "PICK", color: "#ffee00" },
            { id: 5, name: "OBJ", color: "#aaaaaa" },
            { id: 6, name: "OBSTACLE", color: "#ff0000" },
            { id: 7, name: "UNKNOWN", color: "#000000" },
            { id: 8, name: "GOAL REACHED", color: "#00ff00" }
        ];
        config.forEach(c => {
            typeNames[c.id] = c.name;
            typeColors[c.id] = new THREE.Color(c.color);
        });

        const detLegendDiv = document.getElementById('det-legend');
        for (const [id, data] of Object.entries(detectColors)) {
            const item = document.createElement('div');
            item.className = 'legend-item';
            item.innerHTML = `<div class="circle-box" style="background:${data.color}"></div><span>${data.name}</span>`;
            detLegendDiv.appendChild(item);
        }
        const stateLegendDiv = document.getElementById('state-legend');
        for (const [id, data] of Object.entries(stateColors)) {
            const item = document.createElement('div');
            item.className = 'legend-item';
            item.innerHTML = `<div class="color-box" style="background:${data.color}"></div><span>${data.name}</span>`;
            stateLegendDiv.appendChild(item);
        }

        loadStaticLidar(0.0);
        loadStaticPath();
        loadStaticPlanned();
        loadStaticWaypoints();
        loadStaticDetections();

        document.getElementById('sl-score').oninput = (e) => { 
            cfgMinScore = parseFloat(e.target.value); 
            document.getElementById('val-score').innerText = cfgMinScore.toFixed(2);
            loadStaticLidar(cfgMinScore);
        };

        // UI Layer Collapsible Toggle
        const settingsToggle = document.getElementById('settings-toggle');
        const settingsContent = document.getElementById('settings-content');
        settingsToggle.addEventListener('click', () => {
            if (settingsContent.style.display === 'none') {
                settingsContent.style.display = 'block';
                settingsToggle.innerHTML = 'Static Export &#9660;';
            } else {
                settingsContent.style.display = 'none';
                settingsToggle.innerHTML = 'Static Export &#9654;';
            }
        });

        window.addEventListener('keydown', (e) => onKey(e, true));
        window.addEventListener('keyup', (e) => onKey(e, false));
        window.addEventListener('resize', onWindowResize);
        window.snapToRover = () => {
            camera.position.copy(roverMesh.position).add(new THREE.Vector3(0,10,-10));
            controls.target.copy(roverMesh.position);
        };
    }

    function loadStaticLidar(minScore) {
        if(currentPoints) {
            scene.remove(currentPoints);
            currentPoints.geometry.dispose();
            currentPoints.material.dispose();
        }
        const pts = [];
        const colors = [];
        const c = new THREE.Color();
        let count = 0;
        for(let i=0; i<RAW_LIDAR.length; i+=4) {
            const score = RAW_LIDAR[i+3];
            if(score >= minScore) {
                pts.push(RAW_LIDAR[i], RAW_LIDAR[i+1], -RAW_LIDAR[i+2]);
                c.setHSL(score * 0.33, 1.0, 0.5); 
                colors.push(c.r, c.g, c.b);
                count++;
            }
        }
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
        geo.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
        const mat = new THREE.PointsMaterial({ size: 0.5, vertexColors: true });
        currentPoints = new THREE.Points(geo, mat);
        scene.add(currentPoints);
        document.getElementById('stats').innerText = "Points: " + count;
    }

    function loadStaticPath() {
        const vertices = [];
        const colors = [];
        const c = new THREE.Color();
        for(let i=0; i<RAW_PATH.length; i+=4) {
            vertices.push(RAW_PATH[i], RAW_PATH[i+1], -RAW_PATH[i+2]);
            const state = Math.floor(RAW_PATH[i+3]);
            const hex = stateColors[state] ? stateColors[state].color : "#ffffff";
            c.set(hex);
            colors.push(c.r, c.g, c.b);
        }
        if(vertices.length > 0) {
            const geo = new THREE.BufferGeometry();
            geo.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
            geo.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
            const mat = new THREE.LineBasicMaterial({ vertexColors: true, linewidth: 5 });
            pathLine = new THREE.Line(geo, mat); 
            scene.add(pathLine);
        }
    }

    function loadStaticPlanned() {
        const vertices = [];
        for(let i=0; i<RAW_PLANNED.length; i+=3) {
            vertices.push(RAW_PLANNED[i], RAW_PLANNED[i+1], -RAW_PLANNED[i+2]);
        }
        if(vertices.length > 0) {
            const geo = new THREE.BufferGeometry();
            geo.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
            plannedPathLine = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0xeeff00, linewidth: 5 }));
            scene.add(plannedPathLine);
        }
    }

    function loadStaticWaypoints() {
        const beaconGeo = new THREE.BoxGeometry(0.5, 10000, 0.5); 
        for(let i=0; i<RAW_WAYPOINTS.length; i+=4) {
            const x = RAW_WAYPOINTS[i];
            const y = RAW_WAYPOINTS[i+1];
            const z = RAW_WAYPOINTS[i+2];
            const type = RAW_WAYPOINTS[i+3];
            const col = typeColors[type] || typeColors[7]; 
            const beaconMat = new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.3, depthTest: false });
            const beacon = new THREE.Mesh(beaconGeo, beaconMat);
            beacon.position.set(x, 0, -z);
            waypointGroup.add(beacon);
            const div = document.createElement('div');
            div.className = 'hud-marker';
            div.innerText = typeNames[type] || "UNK";
            div.style.borderColor = "#" + col.getHexString();
            markerLayer.appendChild(div);
            activeWaypoints.push({ div: div, pos: new THREE.Vector3(x, 0, -z) });
        }
    }

    function loadStaticDetections() {
        const canvas = document.createElement('canvas');
        canvas.width = 32; canvas.height = 32;
        const ctx = canvas.getContext('2d');
        ctx.beginPath(); ctx.arc(16,16,14,0,2*Math.PI); ctx.fillStyle = 'white'; ctx.fill();
        const tex = new THREE.CanvasTexture(canvas);
        for(let i=0; i<RAW_DETECTIONS.length; i+=4) {
            const x = RAW_DETECTIONS[i];
            const y = RAW_DETECTIONS[i+1];
            const z = RAW_DETECTIONS[i+2];
            const type = RAW_DETECTIONS[i+3];
            let col = "#ffffff";
            if(detectColors[type]) col = detectColors[type].color;
            const mat = new THREE.PointsMaterial({
                color: col, map: tex, size: 2.0, 
                sizeAttenuation: true, alphaTest: 0.5, transparent: true
            });
            const geo = new THREE.BufferGeometry();
            geo.setAttribute('position', new THREE.Float32BufferAttribute([x, y, -z], 3));
            detectionGroup.add(new THREE.Points(geo, mat));
        }
    }

    function updateHUD() {
        const width = window.innerWidth;
        const height = window.innerHeight;
        const pad = 30; 
        activeWaypoints.forEach(wp => {
            const target = wp.pos.clone();
            target.y = roverMesh.position.y + 3.0; 
            target.project(camera);
            let x = (target.x * .5 + .5) * width;
            let y = (target.y * -.5 + .5) * height;
            const isBehind = target.z > 1;
            if (isBehind) { x = width - x; y = height - y; }
            const cx = width / 2; const cy = height / 2;
            const dx = x - cx; const dy = y - cy;
            if (!isBehind && x >= pad && x <= width - pad && y >= pad && y <= height - pad) {
                y -= 40; wp.div.style.opacity = "0.9";
            } else {
                let t = Infinity;
                if (dx > 0) t = Math.min(t, (width - pad - cx) / dx);
                if (dx < 0) t = Math.min(t, (pad - cx) / dx);
                if (dy > 0) t = Math.min(t, (height - pad - cy) / dy);
                if (dy < 0) t = Math.min(t, (pad - cy) / dy);
                x = cx + dx * t; y = cy + dy * t; wp.div.style.opacity = "0.6";
            }
            wp.div.style.left = x + 'px'; wp.div.style.top = y + 'px';
            const dxPos = roverMesh.position.x - wp.pos.x;
            const dzPos = roverMesh.position.z - wp.pos.z; 
            const dist = Math.sqrt(dxPos*dxPos + dzPos*dzPos);
            wp.div.innerText = `${wp.div.innerText.split(' ')[0]} ${Math.round(dist)}m`;
        });
    }

    function onKey(e, p) { 
        if(keys.hasOwnProperty(e.key.toLowerCase())) keys[e.key.toLowerCase()] = p; 
        if(e.key === 'Shift') keys.shift = p;
        if(p && e.key===' ') window.snapToRover();
    }
    function onWindowResize() { camera.aspect = window.innerWidth / window.innerHeight; camera.updateProjectionMatrix(); renderer.setSize(window.innerWidth, window.innerHeight); }

    function animate() {
        requestAnimationFrame(animate);
        const spd = (keys.shift?15:5)*0.016; 
        const fwd = new THREE.Vector3(); camera.getWorldDirection(fwd); fwd.y=0; fwd.normalize();
        const rgt = new THREE.Vector3().crossVectors(fwd, camera.up).normalize();
        if(keys.w) camera.position.addScaledVector(fwd, spd);
        if(keys.s) camera.position.addScaledVector(fwd, -spd);
        if(keys.d) camera.position.addScaledVector(rgt, spd);
        if(keys.a) camera.position.addScaledVector(rgt, -spd);
        if(keys.q) camera.position.y += spd;
        if(keys.e) camera.position.y -= spd;
        controls.target.add(new THREE.Vector3(0,0,0).addScaledVector(fwd, (keys.w-keys.s)*spd).addScaledVector(rgt, (keys.d-keys.a)*spd));
        controls.update();
        updateHUD();
        renderer.render(scene, camera);
    }
</script>
</body>
</html>
    )JS";

    return stdSS.str();
}
