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
#include "./PlotsAndGraphs.hpp"

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
                PathTracer(const std::string& szPlotTitle)
                {
                    // Initialize member variables.
                    m_mtRoverPathPlot = matplot::figure(true);
                    m_mtRoverPathAxes = m_mtRoverPathPlot->current_axes();
                    m_szPlotTitle     = szPlotTitle;

                    // Make sur plot title is not empty.
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
                    // Check if a file with the same title name already exists. If so then append a number to the end of the file name and recheck.
                    std::string szFileName = logging::g_szLoggingOutputPath + "/path_plots/" + m_szPlotTitle;
                    int nFileNum           = 0;
                    while (std::filesystem::exists(szFileName + std::to_string(nFileNum) + ".png"))
                    {
                        ++nFileNum;
                    }
                    // Add the file number to the file name.
                    szFileName = szFileName + std::to_string(nFileNum);

                    // Check if the final directory exists. If not then create it.
                    if (!std::filesystem::exists(logging::g_szLoggingOutputPath + "/path_plots"))
                    {
                        std::filesystem::create_directory(logging::g_szLoggingOutputPath + "/path_plots");
                    }
                    // Save the plot
                    m_mtRoverPathPlot->save(szFileName + ".png");
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
                void CreateLayer(const std::string& szLayerName, const std::string& szStyleString = "-o")
                {
                    // Check if that layer already exists in the map.
                    if (m_umPathMap.find(szLayerName) != m_umPathMap.end())
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "Layer already exists. Cannot create layer.");
                        return;
                    }

                    // Check if the layer name exists in the map.
                    if (m_umPathMap.find(szLayerName) != m_umPathMap.end())
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "Layer already exists. Cannot create layer.");
                        return;
                    }

                    // Add the layer to the maps.
                    m_umPathMap[szLayerName]               = std::vector<std::pair<double, double>>();
                    m_umLineStyleMap[szLayerName]          = szStyleString;
                    m_umLastPlotUpdateTimeMap[szLayerName] = std::chrono::system_clock::now();
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
                    // Check if the layer name exists in the map.
                    if (m_umPathMap.find(szLayerName) == m_umPathMap.end())
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "Layer does not exist. Cannot delete layer.");
                        return;
                    }

                    // Remove the layer from the maps.
                    m_umPathMap.erase(szLayerName);
                    m_umLineStyleMap.erase(szLayerName);
                }

                /******************************************************************************
                 * @brief Clear the path of a layer.
                 *
                 * @param szLayerName - The alias name of the layer.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void ClearLayerPath(const std::string& szLayerName)
                {
                    // Check if the layer name exists in the map.
                    if (m_umPathMap.find(szLayerName) == m_umPathMap.end())
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "Layer does not exist. Cannot clear layer path.");
                        return;
                    }

                    // Clear the layer path.
                    m_umPathMap[szLayerName].clear();
                }

                /******************************************************************************
                 * @brief Add a waypoint to the path and plot the path. This method has a limit
                 *      to the number of waypoints that can be added per second.
                 *
                 * @param stWaypoint - The waypoint to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param nMaxWaypointsPerSecond - The maximum number of waypoints that can be added per second.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void AddPoint(const geoops::Waypoint& stWaypoint, const std::string& szLayerName, const uint nMaxWaypointsPerSecond = 1)
                {
                    // Check the update time.
                    if (!this->CheckUpdateTime(szLayerName, nMaxWaypointsPerSecond))
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
                 * @brief Add a waypoint to the path and plot the path. This method has no limit
                 *      to the number of waypoints that can be added per second.
                 *
                 * @param stCoordinate - The coordinate of the waypoint to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param nMaxWaypointsPerSecond - The maximum number of waypoints that can be added per second.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void AddPoint(const geoops::UTMCoordinate& stCoordinate, const std::string& szLayerName, const uint nMaxWaypointsPerSecond = 1)
                {
                    // Check the update time.
                    if (!this->CheckUpdateTime(szLayerName, nMaxWaypointsPerSecond))
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
                 * @brief Add a waypoint to the path and plot the path. This method has no limit
                 *      to the number of waypoints that can be added per second.
                 *
                 * @param stCoordinate - The coordinate of the waypoint to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param nMaxWaypointsPerSecond - The maximum number of waypoints that can be added per second.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void AddPoint(const geoops::GPSCoordinate& stCoordinate, const std::string& szLayerName, const uint nMaxWaypointsPerSecond = 1)
                {
                    // Check the update time.
                    if (!this->CheckUpdateTime(szLayerName, nMaxWaypointsPerSecond))
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
                 *      to the number of waypoints that can be added per second.
                 *
                 * @param stWaypoints - The waypoints to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param unMaxUpdatesPerSecond - The maximum number of waypoints that can be added per second. Set to 0 for no limit.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void AddPoints(const std::vector<geoops::Waypoint>& stWaypoints, const std::string& szLayerName, const uint unMaxUpdatesPerSecond = 1)
                {
                    // Check the update time.
                    if (!this->CheckUpdateTime(szLayerName, unMaxUpdatesPerSecond))
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
                 *      to the number of waypoints that can be added per second.
                 *
                 * @param vCoordinates - The coordinates to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param unMaxUpdatesPerSecond - The maximum number of waypoints that can be added per second. Set to 0 for no limit.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void AddPoints(const std::vector<geoops::UTMCoordinate>& vCoordinates, const std::string& szLayerName, const uint unMaxUpdatesPerSecond = 1)
                {
                    // Check the update time.
                    if (!this->CheckUpdateTime(szLayerName, unMaxUpdatesPerSecond))
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
                 *    to the number of waypoints that can be added per second.
                 *
                 * @param vCoordinates - The coordinates to add to the path.
                 * @param szLayerName - The name of the layer to add the waypoints to.
                 * @param unMaxUpdatesPerSecond - The maximum number of waypoints that can be added per second. Set to 0 for no limit.
                 *
                 * @author clayjay3 (claytonraycowen@gmail.com)
                 * @date 2025-01-08
                 ******************************************************************************/
                void AddPoints(const std::vector<geoops::GPSCoordinate>& vCoordinates, const std::string& szLayerName, const uint unMaxUpdatesPerSecond = 1)
                {
                    // Check the update time.
                    if (!this->CheckUpdateTime(szLayerName, unMaxUpdatesPerSecond))
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

            private:
                // Declare private member variables.
                matplot::figure_handle m_mtRoverPathPlot;
                matplot::axes_handle m_mtRoverPathAxes;
                std::unordered_map<std::string, std::string> m_umLineStyleMap;
                std::unordered_map<std::string, std::chrono::system_clock::time_point> m_umLastPlotUpdateTimeMap;
                std::unordered_map<std::string, std::vector<std::pair<double, double>>> m_umPathMap;
                std::string m_szPlotTitle;

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

                    // Loop through each of the layer name keys in the map.
                    for (const std::pair<const std::string, std::string>& stdLayer : m_umLineStyleMap)
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
                bool CheckUpdateTime(const std::string& szLayerName, const uint unMaxUpdatesPerSecond)
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
        };
    }    // namespace graphing
}    // namespace logging

#endif
