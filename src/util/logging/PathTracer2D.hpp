/******************************************************************************
 * @brief The PathTracer class is used to trace the path of the rover and
 *    plot the path on a 2D graph.
 *
 * @file PathTracer.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-08
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef PATH_TRACER_2D_HPP
#define PATH_TRACER_2D_HPP

#include "../GeospatialOperations.hpp"

/// \cond
#include <chrono>
#include <matplot/matplot.h>

/// \endcond

/******************************************************************************
 * @brief Namespace containing all global type/structs that will be used project wide
 *      for logging.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-08
 ******************************************************************************/
namespace logging
{
    /******************************************************************************
     * @brief Namespace containing all global type/structs that will be used project wide
     *      for graphing and plotting data with matplotlib. These graphing functions
     *      are built to be as feature rich as possible while still being easy to use.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-01-08
     ******************************************************************************/
    namespace graphing
    {
        /******************************************************************************
         * @brief The PathTracer class is used to trace the path of the rover and
         *      plot the path on a 2D graph.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-08
         ******************************************************************************/
        class PathTracer
        {
            public:
                /******************************************************************************
                 * @brief Construct a new Path Tracer object.
                 *
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                PathTracer(const std::string& szPlotTitle = "Graph")
                {
                    // Initialize member variables.
                    m_mtRoverPathPlot = matplot::figure(true);
                    m_mtRoverPathAxes = m_mtRoverPathPlot->current_axes();
                    m_szPlotTitle     = szPlotTitle;

                    // Check if a file with the same title name already exists. If so then append a number to the end of the file name and recheck.
                    std::string m_zePlotSavePath = logging::g_szLoggingOutputPath + "/path_plots/" + m_szPlotTitle;
                    int nFileNum                 = 0;
                    while (std::filesystem::exists(m_zePlotSavePath + std::to_string(nFileNum) + ".png"))
                    {
                        ++nFileNum;
                    }
                    // Add the file number to the file name.
                    m_zePlotSavePath = m_zePlotSavePath + std::to_string(nFileNum);
                    // Check if the final directory exists. If not then create it.
                    if (!std::filesystem::exists(logging::g_szLoggingOutputPath + "/path_plots"))
                    {
                        std::filesystem::create_directory(logging::g_szLoggingOutputPath + "/path_plots");
                    }

                    // Configure the matplotplusplus gnuplot backend to not display the plot, instead save it to a file.
                    m_mtRoverPathPlot->backend()->output(m_zePlotSavePath + ".png");

                    // Make sure plot title is not empty.
                    if (m_szPlotTitle.empty())
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "Plot title is empty. Setting title to default.");
                        m_szPlotTitle = "RoverPath";
                    }

                    // Configure plot.
                    m_mtRoverPathPlot->title(m_szPlotTitle);
                    m_mtRoverPathAxes->xlabel("Easting");
                    m_mtRoverPathAxes->ylabel("Northing");
                }

                /******************************************************************************
                 * @brief Destroy the Path Tracer object.
                 *
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                ~PathTracer()
                {
                    // Nothing to do yet.
                }

                /******************************************************************************
                 * @brief Add a new draw layer to the plot.
                 *
                 * @param szLayerName - The alias name of the layer.
                 * @param szStyleString - The style of the layer. Default is "-o" which is a blue line with blue dots.
                 *      Here are the full options for the style string:
                 *          Line Styles:
                 *              "-": Solid line
                 *              "--": Dashed line
                 *              "-.": Dash-dot line
                 *              ":": Dotted line
                 *              Marker Styles
                 *              "+": Plus sign
                 *              "o": Circle
                 *              "*": Asterisk
                 *              ".": Point
                 *              "x": Cross
                 *              "s" or "square": Square
                 *              "d" or "diamond": Diamond
                 *              "^": Upward-pointing triangle
                 *              "v" or "V": Downward-pointing triangle
                 *              ">": Custom marker (right arrow, ▶)
                 *              "<": Custom marker (left arrow, ◀)
                 *              "p" or "pentagram": Pentagram
                 *              "h" or "hexagram": Hexagram
                 *
                 *           Colors: (These colors can be used for line color, marker color, or marker face color. The letter corresponds to a color)
                 *              "b": Blue
                 *              "k": Black
                 *              "r": Red
                 *              "g": Green
                 *              "y": Yellow
                 *              "c": Cyan
                 *              "m": Magenta
                 *              "w": White
                 *           Additional Options:
                 *              "f" or "filled": Fills the marker's face (if the marker style supports it).
                 *              Line width and marker size can be adjusted programmatically, not directly in the style string.
                 *              Example Usage of Style Strings
                 *              You can combine line styles, marker styles, and colors into a single string:
                 *
                 *              "--o": Dashed line with circle markers.
                 *              ":x": Dotted line with cross markers.
                 *              "-r": Solid red line.
                 *              "o": Circle markers with default line style (solid).
                 *              "-om": Solid magenta line with circle markers.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void CreatePathLayer(const std::string& szLayerName, const std::string& szStyleString = "-o")
                {
                    // Check if the layer name exists in the map.
                    if (m_umPathMap.find(szLayerName) != m_umPathMap.end())
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "Layer already exists. Cannot create layer.");
                        return;
                    }

                    // Add the layer to the maps.
                    m_umPathMap[szLayerName]               = std::vector<std::pair<double, double>>();
                    m_umPathLineStyleMap[szLayerName]      = szStyleString;
                    m_umLastPlotUpdateTimeMap[szLayerName] = std::chrono::system_clock::now();
                }

                /******************************************************************************
                 * @brief Add a new draw layer to the plot.
                 *
                 * @param szLayerName - The alias name of the layer.
                 * @param szColorString - The color of the layer. The default is "blue".
                 *          Here are the full options for the color string:
                 *             "blue": Blue
                 *             "black": Black
                 *             "red": Red
                 *             "green": Green
                 *             "yellow": Yellow
                 *             "cyan": Cyan
                 *             "magenta": Magenta
                 *             "white": White
                 * @param bFillMarkerFace - Whether or not to fill the marker face. Default is true.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-27
                 ******************************************************************************/
                void CreateDotLayer(const std::string& szLayerName, const std::string& szColorString = "blue", const bool bFillMarkerFace = true)
                {
                    // Check if the layer name exists in the map.
                    if (m_umDotMap.find(szLayerName) != m_umDotMap.end())
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "Layer already exists. Cannot create layer.");
                        return;
                    }

                    // Add the layer to the maps.
                    m_umDotMap[szLayerName]               = std::vector<std::tuple<double, double, double>>();
                    m_umDotLineStyleMap[szLayerName]      = std::make_pair(szColorString, bFillMarkerFace);
                    m_umLastDotUpdateTimeMap[szLayerName] = std::chrono::system_clock::now();
                }

                /******************************************************************************
                 * @brief Delete a draw layer from the plot.
                 *
                 * @param szLayerName - The alias name of the layer.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void DeleteLayer(const std::string& szLayerName)
                {
                    // Check if the layer name exist in the path or dot maps.
                    if (m_umPathMap.find(szLayerName) == m_umPathMap.end() && m_umDotMap.find(szLayerName) == m_umDotMap.end())
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "Layer does not exist. Cannot delete layer.");
                        return;
                    }

                    // Remove the appropriate layers from the maps.
                    if (m_umPathMap.find(szLayerName) != m_umPathMap.end())
                    {
                        m_umPathMap.erase(szLayerName);
                        m_umPathLineStyleMap.erase(szLayerName);
                        m_umLastPlotUpdateTimeMap.erase(szLayerName);
                    }
                    if (m_umDotMap.find(szLayerName) != m_umDotMap.end())
                    {
                        m_umDotMap.erase(szLayerName);
                        m_umDotLineStyleMap.erase(szLayerName);
                        m_umLastDotUpdateTimeMap.erase(szLayerName);
                    }
                }

                /******************************************************************************
                 * @brief Clear the path or dots of a layer.
                 *
                 * @param szLayerName - The alias name of the layer.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void ClearLayer(const std::string& szLayerName)
                {
                    // Check if the layer name exist in the path or dot maps.
                    if (m_umPathMap.find(szLayerName) == m_umPathMap.end() && m_umDotMap.find(szLayerName) == m_umDotMap.end())
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "Layer does not exist. Cannot clear layer.");
                        return;
                    }

                    // Clear the appropriate layer.
                    if (m_umPathMap.find(szLayerName) != m_umPathMap.end())
                    {
                        m_umPathMap[szLayerName].clear();
                    }
                    if (m_umDotMap.find(szLayerName) != m_umDotMap.end())
                    {
                        m_umDotMap[szLayerName].clear();
                    }
                }

                /******************************************************************************
                 * @brief Add a waypoint to the path and plot the path. This method has a limit
                 *      to the number of waypoints that can be added per second. Set the maximum
                 *      number of waypoints per second to 0 for no limit.
                 *
                 * @param stWaypoint - The waypoint to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param nMaxWaypointsPerSecond - The maximum number of waypoints that can be added per second.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void AddPathPoint(const geoops::Waypoint& stWaypoint, const std::string& szLayerName, const uint nMaxWaypointsPerSecond = 1)
                {
                    // Check the update time.
                    if (!this->CheckPathUpdateTime(szLayerName, nMaxWaypointsPerSecond))
                    {
                        // Return if the maximum number of waypoints per second has been exceeded.
                        return;
                    }

                    // Add the waypoint to the path.
                    m_umPathMap[szLayerName].emplace_back(stWaypoint.GetUTMCoordinate().dEasting, stWaypoint.GetUTMCoordinate().dNorthing);

                    // Update the plot.
                    this->UpdatePlot();
                }

                /******************************************************************************
                 * @brief Add a waypoint to the path and plot the path. This method has a limit
                 *      to the number of waypoints that can be added per second. Set the maximum
                 *      number of waypoints per second to 0 for no limit.
                 *
                 * @param stCoordinate - The coordinate of the waypoint to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param nMaxWaypointsPerSecond - The maximum number of waypoints that can be added per second.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void AddPathPoint(const geoops::UTMCoordinate& stCoordinate, const std::string& szLayerName, const uint nMaxWaypointsPerSecond = 1)
                {
                    // Check the update time.
                    if (!this->CheckPathUpdateTime(szLayerName, nMaxWaypointsPerSecond))
                    {
                        // Return if the maximum number of waypoints per second has been exceeded.
                        return;
                    }

                    // Add the waypoint to the path.
                    m_umPathMap[szLayerName].emplace_back(stCoordinate.dEasting, stCoordinate.dNorthing);

                    // Update the plot.
                    this->UpdatePlot();
                }

                /******************************************************************************
                 * @brief Add a waypoint to the path and plot the path. This method has a limit
                 *      to the number of waypoints that can be added per second. Set the maximum
                 *      number of waypoints per second to 0 for no limit.
                 *
                 * @param stCoordinate - The coordinate of the waypoint to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param nMaxWaypointsPerSecond - The maximum number of waypoints that can be added per second.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void AddPathPoint(const geoops::GPSCoordinate& stCoordinate, const std::string& szLayerName, const uint nMaxWaypointsPerSecond = 1)
                {
                    // Check the update time.
                    if (!this->CheckPathUpdateTime(szLayerName, nMaxWaypointsPerSecond))
                    {
                        // Return if the maximum number of waypoints per second has been exceeded.
                        return;
                    }

                    // Create instance variables.
                    geoops::UTMCoordinate stUTMCoordinate = geoops::ConvertGPSToUTM(stCoordinate);

                    // Add the waypoint to the path.
                    m_umPathMap[szLayerName].emplace_back(stUTMCoordinate.dEasting, stUTMCoordinate.dNorthing);

                    // Update the plot.
                    this->UpdatePlot();
                }

                /******************************************************************************
                 * @brief Add a waypoint to the path and plot the path. This method has no limit
                 *      to the number of waypoints that can be added per call. But the
                 *    number of waypoints that can be added per second is limited. Set the
                 *    maximum number of waypoints per second to 0 for no limit.
                 *
                 * @param stWaypoints - The waypoints to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param unMaxUpdatesPerSecond - The maximum number of waypoints that can be added per second. Set to 0 for no limit.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void AddPathPoints(const std::vector<geoops::Waypoint>& stWaypoints, const std::string& szLayerName, const uint unMaxUpdatesPerSecond = 1)
                {
                    // Check the update time.
                    if (!this->CheckPathUpdateTime(szLayerName, unMaxUpdatesPerSecond))
                    {
                        // Return if the maximum number of waypoints per second has been exceeded.
                        return;
                    }

                    // Add the waypoints to the vector or double pairs at the given layer name in the map.
                    for (const geoops::Waypoint& stWaypoint : stWaypoints)
                    {
                        m_umPathMap[szLayerName].emplace_back(stWaypoint.GetUTMCoordinate().dEasting, stWaypoint.GetUTMCoordinate().dNorthing);
                    }

                    // Update the plot.
                    this->UpdatePlot();
                }

                /******************************************************************************
                 * @brief Add a waypoint to the path and plot the path. This method has no limit
                 *      to the number of waypoints that can be added per call. But the
                 *    number of waypoints that can be added per second is limited. Set the
                 *    maximum number of waypoints per second to 0 for no limit.
                 *
                 * @param vCoordinates - The coordinates to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param unMaxUpdatesPerSecond - The maximum number of waypoints that can be added per second. Set to 0 for no limit.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void AddPathPoints(const std::vector<geoops::UTMCoordinate>& vCoordinates, const std::string& szLayerName, const uint unMaxUpdatesPerSecond = 1)
                {
                    // Check the update time.
                    if (!this->CheckPathUpdateTime(szLayerName, unMaxUpdatesPerSecond))
                    {
                        // Return if the maximum number of waypoints per second has been exceeded.
                        return;
                    }

                    // Add the waypoints to the vector or double pairs at the given layer name in the map.
                    for (const geoops::UTMCoordinate& stCoordinate : vCoordinates)
                    {
                        m_umPathMap[szLayerName].emplace_back(stCoordinate.dEasting, stCoordinate.dNorthing);
                    }

                    // Update the plot.
                    this->UpdatePlot();
                }

                /******************************************************************************
                 * @brief Add a waypoint to the path and plot the path. This method has no limit
                 *    to the number of waypoints that can be added per one call. But the
                 *    number of waypoints that can be added per second is limited. Set the
                 *    maximum number of waypoints per second to 0 for no limit.
                 *
                 * @param vCoordinates - The coordinates to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param unMaxUpdatesPerSecond - The maximum number of waypoints that can be added per second. Set to 0 for no limit.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void AddPathPoints(const std::vector<geoops::GPSCoordinate>& vCoordinates, const std::string& szLayerName, const uint unMaxUpdatesPerSecond = 1)
                {
                    // Check the update time.
                    if (!this->CheckPathUpdateTime(szLayerName, unMaxUpdatesPerSecond))
                    {
                        // Return if the maximum number of waypoints per second has been exceeded.
                        return;
                    }

                    // Add the waypoints to the vector or double pairs at the given layer name in the map.
                    for (const geoops::GPSCoordinate& stCoordinate : vCoordinates)
                    {
                        geoops::UTMCoordinate stUTMCoordinate = geoops::ConvertGPSToUTM(stCoordinate);
                        m_umPathMap[szLayerName].emplace_back(stUTMCoordinate.dEasting, stUTMCoordinate.dNorthing);
                    }

                    // Update the plot.
                    this->UpdatePlot();
                }

                /******************************************************************************
                 * @brief Add a waypoint as a dot to the path. This method has a limit
                 *      to the number of waypoints that can be added per second. Set the maximum
                 *      number of waypoints per second to 0 for no limit.
                 *
                 * @param stWaypoint - The waypoint to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param nMaxWaypointsPerSecond - The maximum number of waypoints that can be added per second.
                 *
                 * @note Because this method uses Waypoint structs, the radius of the waypoint will be used as the size of the dot.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-27
                 ******************************************************************************/
                void AddDot(const geoops::Waypoint& stWaypoint, const std::string& szLayerName, const uint nMaxWaypointsPerSecond = 1)
                {
                    // Check the update time.
                    if (!this->CheckDotUpdateTime(szLayerName, nMaxWaypointsPerSecond))
                    {
                        // Return if the maximum number of waypoints per second has been exceeded.
                        return;
                    }

                    // Check the radius of the waypoint. It shouldn't be less than 0.
                    if (stWaypoint.dRadius <= 0)
                    {
                        m_umDotMap[szLayerName].emplace_back(stWaypoint.GetUTMCoordinate().dEasting, stWaypoint.GetUTMCoordinate().dNorthing, 5);
                    }
                    else
                    {
                        m_umDotMap[szLayerName].emplace_back(stWaypoint.GetUTMCoordinate().dEasting, stWaypoint.GetUTMCoordinate().dNorthing, stWaypoint.dRadius);
                    }

                    // Update the plot.
                    this->UpdatePlot();
                }

                /******************************************************************************
                 * @brief Add a waypoint as a dot to the path. This method has a limit
                 *     to the number of waypoints that can be added per second. Set the maximum
                 *     number of waypoints per second to 0 for no limit.
                 *
                 * @param stCoordinate - The coordinate of the waypoint to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param nMaxWaypointsPerSecond - The maximum number of waypoints that can be added per second.
                 * @param dDotRadius - The radius of the dot to add to the path.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-27
                 ******************************************************************************/
                void AddDot(const geoops::UTMCoordinate& stCoordinate, const std::string& szLayerName, const uint nMaxWaypointsPerSecond = 1, const double dDotRadius = 5)
                {
                    // Check the update time.
                    if (!this->CheckDotUpdateTime(szLayerName, nMaxWaypointsPerSecond))
                    {
                        // Return if the maximum number of waypoints per second has been exceeded.
                        return;
                    }

                    // Add the waypoint to the path.
                    m_umDotMap[szLayerName].emplace_back(stCoordinate.dEasting, stCoordinate.dNorthing, dDotRadius);

                    // Update the plot.
                    this->UpdatePlot();
                }

                /******************************************************************************
                 * @brief Add a waypoint as a dot to the path. This method has a limit
                 *     to the number of waypoints that can be added per second. Set the maximum
                 *     number of waypoints per second to 0 for no limit.
                 *
                 * @param stCoordinate - The coordinate of the waypoint to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param nMaxWaypointsPerSecond - The maximum number of waypoints that can be added per second.
                 * @param dDotRadius - The radius of the dot to add to the path.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-27
                 ******************************************************************************/
                void AddDot(const geoops::GPSCoordinate& stCoordinate, const std::string& szLayerName, const uint nMaxWaypointsPerSecond = 1, const double dDotRadius = 5)
                {
                    // Check the update time.
                    if (!this->CheckDotUpdateTime(szLayerName, nMaxWaypointsPerSecond))
                    {
                        // Return if the maximum number of waypoints per second has been exceeded.
                        return;
                    }

                    // Create instance variables.
                    geoops::UTMCoordinate stUTMCoordinate = geoops::ConvertGPSToUTM(stCoordinate);

                    // Add the waypoint to the path.
                    m_umDotMap[szLayerName].emplace_back(stUTMCoordinate.dEasting, stUTMCoordinate.dNorthing, dDotRadius);

                    // Update the plot.
                    this->UpdatePlot();
                }

                /******************************************************************************
                 * @brief Add a waypoint as a dot to the path. This method has no limit
                 *    to the number of waypoints that can be added per one call. But the
                 *    number of waypoints that can be added per second is limited. Set the
                 *    maximum number of waypoints per second to 0 for no limit.
                 *
                 * @param stWaypoints - The waypoints to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param unMaxUpdatesPerSecond - The maximum number of waypoints that can be added per second. Set to 0 for no limit.
                 *
                 * @note Because this method uses Waypoint structs, the radius of the waypoint will be used as the size of the dot.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-27
                 ******************************************************************************/
                void AddDots(const std::vector<geoops::Waypoint>& stWaypoints, const std::string& szLayerName, const uint unMaxUpdatesPerSecond = 1)
                {
                    // Check the update time.
                    if (!this->CheckDotUpdateTime(szLayerName, unMaxUpdatesPerSecond))
                    {
                        // Return if the maximum number of waypoints per second has been exceeded.
                        return;
                    }

                    // Add the waypoints to the vector or double pairs at the given layer name in the map.
                    for (const geoops::Waypoint& stWaypoint : stWaypoints)
                    {
                        // Check the radius of the waypoint. It shouldn't be less than 0.
                        if (stWaypoint.dRadius <= 0)
                        {
                            m_umDotMap[szLayerName].emplace_back(stWaypoint.GetUTMCoordinate().dEasting, stWaypoint.GetUTMCoordinate().dNorthing, 5);
                        }
                        else
                        {
                            m_umDotMap[szLayerName].emplace_back(stWaypoint.GetUTMCoordinate().dEasting, stWaypoint.GetUTMCoordinate().dNorthing, stWaypoint.dRadius);
                        }
                    }

                    // Update the plot.
                    this->UpdatePlot();
                }

                /******************************************************************************
                 * @brief Add a waypoint as a dot to the path. This method has no limit
                 *   to the number of waypoints that can be added per one call. But the
                 *   number of waypoints that can be added per second is limited. Set the
                 *   maximum number of waypoints per second to 0 for no limit.
                 *
                 * @param vCoordinates - The coordinates to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param unMaxUpdatesPerSecond - The maximum number of waypoints that can be added per second. Set to 0 for no limit.
                 * @param dDotRadius - The radius of the dot to add to the path.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-27
                 ******************************************************************************/
                void AddDots(const std::vector<geoops::UTMCoordinate>& vCoordinates,
                             const std::string& szLayerName,
                             const uint unMaxUpdatesPerSecond = 1,
                             const double dDotRadius          = 5)
                {
                    // Check the update time.
                    if (!this->CheckDotUpdateTime(szLayerName, unMaxUpdatesPerSecond))
                    {
                        // Return if the maximum number of waypoints per second has been exceeded.
                        return;
                    }

                    // Add the waypoints to the vector or double pairs at the given layer name in the map.
                    for (const geoops::UTMCoordinate& stCoordinate : vCoordinates)
                    {
                        m_umDotMap[szLayerName].emplace_back(stCoordinate.dEasting, stCoordinate.dNorthing, dDotRadius);
                    }

                    // Update the plot.
                    this->UpdatePlot();
                }

                /******************************************************************************
                 * @brief Add a waypoint as a dot to the path. This method has no limit
                 *      to the number of waypoints that can be added per one call. But the
                 *      number of waypoints that can be added per second is limited. Set the
                 *      maximum number of waypoints per second to 0 for no limit.
                 *
                 * @param vCoordinates - The coordinates to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param unMaxUpdatesPerSecond - The maximum number of waypoints that can be added per second. Set to 0 for no limit.
                 * @param dDotRadius - The radius of the dot to add to the path.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-27
                 ******************************************************************************/
                void AddDots(const std::vector<geoops::GPSCoordinate>& vCoordinates,
                             const std::string& szLayerName,
                             const uint unMaxUpdatesPerSecond = 1,
                             const double dDotRadius          = 5)
                {
                    // Check the update time.
                    if (!this->CheckDotUpdateTime(szLayerName, unMaxUpdatesPerSecond))
                    {
                        // Return if the maximum number of waypoints per second has been exceeded.
                        return;
                    }

                    // Add the waypoints to the vector or double pairs at the given layer name in the map.
                    for (const geoops::GPSCoordinate& stCoordinate : vCoordinates)
                    {
                        geoops::UTMCoordinate stUTMCoordinate = geoops::ConvertGPSToUTM(stCoordinate);
                        m_umDotMap[szLayerName].emplace_back(stUTMCoordinate.dEasting, stUTMCoordinate.dNorthing, dDotRadius);
                    }

                    // Update the plot.
                    this->UpdatePlot();
                }

            private:
                // Declare private member variables.
                matplot::figure_handle m_mtRoverPathPlot;
                matplot::axes_handle m_mtRoverPathAxes;
                std::unordered_map<std::string, std::string> m_umPathLineStyleMap;
                std::unordered_map<std::string, std::pair<std::string, bool>> m_umDotLineStyleMap;
                std::unordered_map<std::string, std::chrono::system_clock::time_point> m_umLastPlotUpdateTimeMap;
                std::unordered_map<std::string, std::chrono::system_clock::time_point> m_umLastDotUpdateTimeMap;
                std::unordered_map<std::string, std::vector<std::pair<double, double>>> m_umPathMap;
                std::unordered_map<std::string, std::vector<std::tuple<double, double, double>>> m_umDotMap;
                std::string m_szPlotTitle;
                std::string m_zePlotSavePath;

                /******************************************************************************
                 * @brief Update the plot with the new waypoints and redraw the plot.
                 *
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void UpdatePlot()
                {
                    // Create instance variables.
                    std::vector<std::string> vLayerNames;

                    // Clear the plot.
                    m_mtRoverPathAxes->clear();

                    /*
                        PATHS
                    */
                    // Loop through each of the layer name keys in the map.
                    for (const std::pair<const std::string, const std::string>& stdLayer : m_umPathLineStyleMap)
                    {
                        // Add the layer name to the vector.
                        vLayerNames.push_back(stdLayer.first);

                        // Check if the vector or coordinates has more than one point.
                        if (m_umPathMap[stdLayer.first].size() > 1)
                        {
                            // Get the x and y coordinates for the layer.
                            std::vector<double> vEasting, vNorthing;
                            for (const std::pair<double, double>& stCoordinate : m_umPathMap[stdLayer.first])
                            {
                                vEasting.push_back(stCoordinate.first);
                                vNorthing.push_back(stCoordinate.second);
                            }

                            // Plot the path.
                            m_mtRoverPathAxes->plot(vEasting, vNorthing, std::string_view(stdLayer.second));
                            // Set the hold to true.
                            m_mtRoverPathAxes->hold(true);
                        }
                    }

                    /*
                        DOTS
                    */
                    // Loop through each of the layer name keys in the map.
                    for (const std::pair<const std::string, const std::pair<std::string, bool>>& stdLayer : m_umDotLineStyleMap)
                    {
                        // Add the layer name to the vector.
                        vLayerNames.push_back(stdLayer.first);

                        // Check if the vector or coordinates has more than one point.
                        if (m_umDotMap[stdLayer.first].size() > 1)
                        {
                            // Get the x and y coordinates for the layer.
                            std::vector<double> vEasting, vNorthing, vRadius;
                            for (const std::tuple<double, double, double>& stCoordinate : m_umDotMap[stdLayer.first])
                            {
                                vEasting.push_back(std::get<0>(stCoordinate));
                                vNorthing.push_back(std::get<1>(stCoordinate));
                                vRadius.push_back(std::get<2>(stCoordinate));
                            }

                            // Plot the path.
                            matplot::line_handle mtLineHandle = m_mtRoverPathAxes->scatter(vEasting, vNorthing, vRadius);
                            mtLineHandle->color(stdLayer.second.first);
                            mtLineHandle->marker_face(stdLayer.second.second);
                            // Set the hold to true.
                            m_mtRoverPathAxes->hold(true);
                        }
                    }

                    // Update legend names.
                    m_mtRoverPathAxes->legend(vLayerNames);
                    // Set the hold to false.
                    m_mtRoverPathAxes->hold(false);
                    // Plot the path.
                    m_mtRoverPathPlot->draw();
                }

                /******************************************************************************
                 * @brief Checks the unordered map of last update times for a given layer name
                 *      and returns true if the time since the last update is greater than the
                 *      maximum updates per second, then it updates the time in the map.
                 *      If the layer name does not exist in the map then it returns false.
                 *      If the given update time is 0, then it will just check if the layer name
                 *      exists in the map.
                 *
                 * @param szLayerName - The name of the layer to check the update time for.
                 * @param unMaxUpdatesPerSecond - The maximum number of updates per second.
                 * @return true - The time since the last update is greater than the maximum updates per second.
                 * @return false - The time since the last update is less than the maximum updates per second.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-09
                 ******************************************************************************/
                bool CheckPathUpdateTime(const std::string& szLayerName, const uint unMaxUpdatesPerSecond)
                {
                    // Check if the layer name exists in the map.
                    if (m_umLastPlotUpdateTimeMap.find(szLayerName) == m_umLastPlotUpdateTimeMap.end())
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "Layer does not exist. Cannot add waypoints.");
                        return false;
                    }

                    // Check if the maximum number of waypoints per second has been exceeded.
                    if (unMaxUpdatesPerSecond != 0 &&
                        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - m_umLastPlotUpdateTimeMap[szLayerName]).count() <
                            1.0 / unMaxUpdatesPerSecond)
                    {
                        // Return if the maximum number of waypoints per second has been exceeded.
                        return false;
                    }

                    // Update the last plot time.
                    m_umLastPlotUpdateTimeMap[szLayerName] = std::chrono::system_clock::now();

                    // Return true.
                    return true;
                }

                /******************************************************************************
                 * @brief Checks the unordered map of last update times for a given layer name
                 *     and returns true if the time since the last update is greater than the
                 *     maximum updates per second, then it updates the time in the map.
                 *     If the layer name does not exist in the map then it returns false.
                 *     If the given update time is 0, then it will just check if the layer name
                 *     exists in the map.
                 *
                 * @param szLayerName - The name of the layer to check the update time for.
                 * @param unMaxUpdatesPerSecond - The maximum number of updates per second.
                 * @return true - The time since the last update is greater than the maximum updates per second.
                 * @return false - The time since the last update is less than the maximum updates per second.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-27
                 ******************************************************************************/
                bool CheckDotUpdateTime(const std::string& szLayerName, const uint unMaxUpdatesPerSecond)
                {
                    // Check if the layer name exists in the map.
                    if (m_umLastDotUpdateTimeMap.find(szLayerName) == m_umLastDotUpdateTimeMap.end())
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "Layer does not exist. Cannot add waypoints.");
                        return false;
                    }

                    // Check if the maximum number of waypoints per second has been exceeded.
                    if (unMaxUpdatesPerSecond != 0 &&
                        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - m_umLastDotUpdateTimeMap[szLayerName]).count() <
                            1.0 / unMaxUpdatesPerSecond)
                    {
                        // Return if the maximum number of waypoints per second has been exceeded.
                        return false;
                    }

                    // Update the last plot time.
                    m_umLastDotUpdateTimeMap[szLayerName] = std::chrono::system_clock::now();

                    // Return true.
                    return true;
                }
        };
    }    // namespace graphing
}    // namespace logging

#endif
