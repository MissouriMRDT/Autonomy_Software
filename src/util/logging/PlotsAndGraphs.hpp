/******************************************************************************
 * @brief This file contains the implementation of useful utility functions
 *       for plotting and graphing certain data with matplotlib.
 *
 * @file PlotsAndGraphs.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-07
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef PLOTS_AND_GRAPHS_HPP
#define PLOTS_AND_GRAPHS_HPP

#include "../../../external/matplotlib-cpp/matplotlibcpp.h"
#include "../GeospatialOperations.hpp"

/// \cond

/// \endcond

/******************************************************************************
 * @brief Namespace containing all global type/structs that will be used project wide
 *      for logging.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-07
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
     * @date 2025-01-07
     ******************************************************************************/
    namespace graphing
    {
        /******************************************************************************
         * @brief Plot a 2D graph of UTM coordinates.
         *
         * @param vCoordinates - The vector of UTM coordinates to plot.
         * @param szTitle - The title of the plot. This will be the name of the file as well.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-07
         ******************************************************************************/
        inline void PlotCoordinates2D(const std::vector<geoops::UTMCoordinate>& vCoordinates, const std::string& szTitle = "UTMCoordinatePlot")
        {
            // Create instance variables.
            std::string szPlotTitle = szTitle;

            // Check if the plot title is empty.
            if (szPlotTitle.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Plot title is empty. Setting title to default.");
                szPlotTitle = "UTMCoordinatePlot";
            }

            // Use matplotlib-cpp to plot the UTM coordinates as red dots with blue lines connecting them.
            std::vector<double> vEasting, vNorthing;
            for (const geoops::UTMCoordinate& stCoordinate : vCoordinates)
            {
                vEasting.push_back(stCoordinate.dEasting);
                vNorthing.push_back(stCoordinate.dNorthing);
            }
            matplotlibcpp::plot(vEasting, vNorthing, "bo-");    // blue lines with red dots
            matplotlibcpp::title(szPlotTitle);
            matplotlibcpp::xlabel("Easting");
            matplotlibcpp::ylabel("Northing");

            // Calculate and annotate distances between points
            for (size_t i = 1; i < vCoordinates.size(); ++i)
            {
                double dDistance    = std::sqrt(std::pow(vCoordinates[i].dEasting - vCoordinates[i - 1].dEasting, 2) +
                                             std::pow(vCoordinates[i].dNorthing - vCoordinates[i - 1].dNorthing, 2));
                double dMidEasting  = (vCoordinates[i].dEasting + vCoordinates[i - 1].dEasting) / 2;
                double dMidNorthing = (vCoordinates[i].dNorthing + vCoordinates[i - 1].dNorthing) / 2;
                matplotlibcpp::annotate(std::to_string(dDistance) + " m", dMidEasting, dMidNorthing);
            }

            // Check if a file with the same title name already exists. If so then append a number to the end of the file name and recheck.
            std::string szFileName = szPlotTitle;
            int nFileNum           = 0;
            while (std::filesystem::exists(szFileName + ".png"))
            {
                szFileName = szPlotTitle + std::to_string(nFileNum);
                ++nFileNum;
            }

            // Save the plot to a file.
            matplotlibcpp::save(logging::g_szLoggingOutputPath + "/path_plots/" + szFileName + ".png");
        }

        /******************************************************************************
         * @brief Plot a 2D graph of GPS coordinates.
         *
         * @param vCoordinates - The vector of GPS coordinates to plot.
         * @param szTitle - The title of the plot. This will be the name of the file as well.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-07
         ******************************************************************************/
        inline void PlotCoordinates2D(const std::vector<geoops::GPSCoordinate>& vCoordinates, const std::string& szTitle = "GPSCoordinatePlot")
        {
            // Create instance variables.
            std::string szPlotTitle = szTitle;

            // Check if the plot title is empty.
            if (szPlotTitle.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Plot title is empty. Setting title to default.");
                szPlotTitle = "GPSCoordinatePlot";
            }

            // Use matplotlib-cpp to plot the UTM coordinates as red dots with blue lines connecting them.
            std::vector<double> vLatitude, vLongitude;
            for (const geoops::GPSCoordinate& stCoordinate : vCoordinates)
            {
                vLatitude.push_back(stCoordinate.dLatitude);
                vLongitude.push_back(stCoordinate.dLongitude);
            }
            matplotlibcpp::plot(vLatitude, vLongitude, "bo-");    // blue lines with red dots
            matplotlibcpp::title(szPlotTitle);
            matplotlibcpp::xlabel("Latitude");
            matplotlibcpp::ylabel("Longitude");

            // Calculate and annotate distances between points
            for (size_t i = 1; i < vCoordinates.size(); ++i)
            {
                double dDistance     = geoops::CalculateGeoMeasurement(vCoordinates[i - 1], vCoordinates[i]).dDistanceMeters;
                double dMidLatitude  = (vCoordinates[i].dLatitude + vCoordinates[i - 1].dLatitude) / 2;
                double dMidLongitude = (vCoordinates[i].dLongitude + vCoordinates[i - 1].dLongitude) / 2;
                matplotlibcpp::annotate(std::to_string(dDistance) + " m", dMidLatitude, dMidLongitude);
            }

            // Check if a file with the same title name already exists. If so then append a number to the end of the file name and recheck.
            std::string szFileName = szPlotTitle;
            int nFileNum           = 0;
            while (std::filesystem::exists(szFileName + ".png"))
            {
                szFileName = szPlotTitle + std::to_string(nFileNum);
                ++nFileNum;
            }

            // Save the plot to a file.
            matplotlibcpp::save(logging::g_szLoggingOutputPath + "/path_plots/" + szFileName + ".png");
        }

        /******************************************************************************
         * @brief Plot a 2D graph of waypoints.
         *
         * @param vWaypoints - The vector of waypoints to plot.
         * @param szTitle - The title of the plot. This will be the name of the file as well.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-07
         ******************************************************************************/
        inline void PlotCoordinates2D(const std::vector<geoops::Waypoint>& vWaypoints, const std::string& szTitle = "WaypointPlot")
        {
            // Create instance variables.
            std::string szPlotTitle = szTitle;

            // Check if the plot title is empty.
            if (szPlotTitle.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Plot title is empty. Setting title to default.");
                szPlotTitle = "WaypointPlot";
            }

            // Use matplotlib-cpp to plot the UTM coordinates as red dots with blue lines connecting them.
            std::vector<double> vEasting, vNorthing;
            for (const geoops::Waypoint& stWaypoint : vWaypoints)
            {
                vEasting.push_back(stWaypoint.GetGPSCoordinate().dLatitude);
                vNorthing.push_back(stWaypoint.GetGPSCoordinate().dLongitude);
            }
            matplotlibcpp::plot(vEasting, vNorthing, "bo-");    // blue lines with red dots
            matplotlibcpp::title(szPlotTitle);
            matplotlibcpp::xlabel("Latitude");
            matplotlibcpp::ylabel("Longitude");

            // Calculate and annotate distances between points
            for (size_t i = 1; i < vWaypoints.size(); ++i)
            {
                double dDistance     = geoops::CalculateGeoMeasurement(vWaypoints[i - 1].GetGPSCoordinate(), vWaypoints[i].GetGPSCoordinate()).dDistanceMeters;
                double dMidLatitude  = (vWaypoints[i].GetGPSCoordinate().dLatitude + vWaypoints[i - 1].GetGPSCoordinate().dLatitude) / 2;
                double dMidLongitude = (vWaypoints[i].GetGPSCoordinate().dLongitude + vWaypoints[i - 1].GetGPSCoordinate().dLongitude) / 2;
                matplotlibcpp::annotate(std::to_string(dDistance) + " m", dMidLatitude, dMidLongitude);
            }

            // Check if a file with the same title name already exists. If so then append a number to the end of the file name and recheck.
            std::string szFileName = szPlotTitle;
            int nFileNum           = 0;
            while (std::filesystem::exists(szFileName + ".png"))
            {
                szFileName = szPlotTitle + std::to_string(nFileNum);
                ++nFileNum;
            }

            // Save the plot
            matplotlibcpp::save(logging::g_szLoggingOutputPath + "/path_plots/" + szFileName + ".png");
        }
    }    // namespace graphing
}    // namespace logging
#endif    // PLOTS_AND_GRAPHS_HPP
