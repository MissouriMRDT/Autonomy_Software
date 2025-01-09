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

#include "../GeospatialOperations.hpp"

/// \cond
#include <matplot/matplot.h>

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
            std::string szPlotTitle       = szTitle;
            matplot::figure_handle mtPlot = matplot::figure();
            matplot::axes_handle mtAxes   = mtPlot->current_axes();

            // Check if the coordinates vector is empty.
            if (vCoordinates.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Coordinates vector is empty. Cannot plot.");
                return;
            }

            // Check if the plot title is empty.
            if (szPlotTitle.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Plot title is empty. Setting title to default.");
                szPlotTitle = "UTMCoordinatePlot";
            }

            // Use matplotplusplus to plot the UTM coordinates as red dots with blue lines connecting them.
            std::vector<double> vEasting, vNorthing;
            for (const geoops::UTMCoordinate& stCoordinate : vCoordinates)
            {
                vEasting.push_back(stCoordinate.dEasting);
                vNorthing.push_back(stCoordinate.dNorthing);
            }
            mtAxes->plot(vEasting, vNorthing, "-o");    // blue lines with red dots
            mtPlot->title(szPlotTitle);
            mtAxes->xlabel("Easting");
            mtAxes->ylabel("Northing");

            // Calculate and annotate distances between points
            for (size_t i = 1; i < vCoordinates.size(); ++i)
            {
                double dDistance    = std::sqrt(std::pow(vCoordinates[i].dEasting - vCoordinates[i - 1].dEasting, 2) +
                                             std::pow(vCoordinates[i].dNorthing - vCoordinates[i - 1].dNorthing, 2));
                double dMidEasting  = (vCoordinates[i].dEasting + vCoordinates[i - 1].dEasting) / 2;
                double dMidNorthing = (vCoordinates[i].dNorthing + vCoordinates[i - 1].dNorthing) / 2;
                mtAxes->text(dMidEasting, dMidNorthing, std::to_string(int(dDistance)) + " m");
            }

            // Check if a file with the same title name already exists. If so then append a number to the end of the file name and recheck.
            std::string szFileName = logging::g_szLoggingOutputPath + "/path_plots/" + szTitle;
            int nFileNum           = 0;
            while (std::filesystem::exists(szFileName + ".png"))
            {
                szFileName = szFileName + std::to_string(nFileNum);
                ++nFileNum;
            }

            // Check if the final directory exists. If not then create it.
            if (!std::filesystem::exists(logging::g_szLoggingOutputPath + "/path_plots"))
            {
                std::filesystem::create_directory(logging::g_szLoggingOutputPath + "/path_plots");
            }
            // Save the plot
            mtPlot->save(szFileName + ".png");
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
            std::string szPlotTitle       = szTitle;
            matplot::figure_handle mtPlot = matplot::figure();
            matplot::axes_handle mtAxes   = mtPlot->current_axes();

            // Check if the coordinates vector is empty.
            if (vCoordinates.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Coordinates vector is empty. Cannot plot.");
                return;
            }

            // Check if the plot title is empty.
            if (szPlotTitle.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Plot title is empty. Setting title to default.");
                szPlotTitle = "GPSCoordinatePlot";
            }

            // Use matplotplusplus to plot the UTM coordinates as red dots with blue lines connecting them.
            std::vector<double> vLatitude, vLongitude;
            for (const geoops::GPSCoordinate& stCoordinate : vCoordinates)
            {
                vLatitude.push_back(stCoordinate.dLatitude);
                vLongitude.push_back(stCoordinate.dLongitude);
            }
            mtAxes->plot(vLatitude, vLongitude, "-o");    // blue lines with red dots
            mtPlot->title(szPlotTitle);
            mtAxes->xlabel("Latitude");
            mtAxes->ylabel("Longitude");

            // Calculate and annotate distances between points
            for (size_t i = 1; i < vCoordinates.size(); ++i)
            {
                double dDistance     = geoops::CalculateGeoMeasurement(vCoordinates[i - 1], vCoordinates[i]).dDistanceMeters;
                double dMidLatitude  = (vCoordinates[i].dLatitude + vCoordinates[i - 1].dLatitude) / 2;
                double dMidLongitude = (vCoordinates[i].dLongitude + vCoordinates[i - 1].dLongitude) / 2;
                mtAxes->text(dMidLatitude, dMidLongitude, std::to_string(int(dDistance)) + " m");
            }

            // Check if a file with the same title name already exists. If so then append a number to the end of the file name and recheck.
            std::string szFileName = logging::g_szLoggingOutputPath + "/path_plots/" + szTitle;
            int nFileNum           = 0;
            while (std::filesystem::exists(szFileName + ".png"))
            {
                szFileName = szFileName + std::to_string(nFileNum);
                ++nFileNum;
            }

            // Check if the final directory exists. If not then create it.
            if (!std::filesystem::exists(logging::g_szLoggingOutputPath + "/path_plots"))
            {
                std::filesystem::create_directory(logging::g_szLoggingOutputPath + "/path_plots");
            }
            // Save the plot
            mtPlot->save(szFileName + ".png");
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
            std::string szPlotTitle       = szTitle;
            matplot::figure_handle mtPlot = matplot::figure();
            matplot::axes_handle mtAxes   = mtPlot->current_axes();

            // Check if the coordinates vector is empty.
            if (vWaypoints.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Waypoints vector is empty. Cannot plot.");
                return;
            }

            // Check if the plot title is empty.
            if (szPlotTitle.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Plot title is empty. Setting title to default.");
                szPlotTitle = "WaypointPlot";
            }

            // Use matplotplusplus to plot the UTM coordinates as red dots with blue lines connecting them.
            std::vector<double> vEasting, vNorthing;
            for (const geoops::Waypoint& stWaypoint : vWaypoints)
            {
                vEasting.push_back(stWaypoint.GetUTMCoordinate().dEasting);
                vNorthing.push_back(stWaypoint.GetUTMCoordinate().dNorthing);
            }
            mtAxes->plot(vEasting, vNorthing, "-o");
            mtPlot->title(szPlotTitle);
            mtAxes->xlabel("Easting");
            mtAxes->ylabel("Northing");

            // Calculate and annotate distances between points.
            for (size_t i = 1; i < vWaypoints.size(); ++i)
            {
                double dDistance    = geoops::CalculateGeoMeasurement(vWaypoints[i - 1].GetUTMCoordinate(), vWaypoints[i].GetUTMCoordinate()).dDistanceMeters;
                double dMidEasting  = (vWaypoints[i].GetUTMCoordinate().dEasting + vWaypoints[i - 1].GetUTMCoordinate().dEasting) / 2;
                double dMidNorthing = (vWaypoints[i].GetUTMCoordinate().dNorthing + vWaypoints[i - 1].GetUTMCoordinate().dNorthing) / 2;
                mtAxes->text(dMidEasting, dMidNorthing, std::to_string(int(dDistance)) + " m");
            }

            // Check if a file with the same title name already exists. If so then append a number to the end of the file name and recheck.
            std::string szFileName = logging::g_szLoggingOutputPath + "/path_plots/" + szTitle;
            int nFileNum           = 0;
            while (std::filesystem::exists(szFileName + ".png"))
            {
                szFileName = szFileName + std::to_string(nFileNum);
                ++nFileNum;
            }

            // Check if the final directory exists. If not then create it.
            if (!std::filesystem::exists(logging::g_szLoggingOutputPath + "/path_plots"))
            {
                std::filesystem::create_directory(logging::g_szLoggingOutputPath + "/path_plots");
            }
            // Save the plot
            mtPlot->save(szFileName + ".png");
        }

        /******************************************************************************
         * @brief Plot a 3D graph of UTM coordinates.
         *
         * @param vCoordinates - The vector of UTM coordinates to plot.
         * @param szTitle - The title of the plot. This will be the name of the file as well.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-08
         ******************************************************************************/
        inline void PlotCoordinates3D(const std::vector<geoops::UTMCoordinate>& vCoordinates, const std::string& szTitle = "UTMCoordinatePlot")
        {
            // Create instance variables.
            std::string szPlotTitle       = szTitle;
            matplot::figure_handle mtPlot = matplot::figure();
            matplot::axes_handle mtAxes   = mtPlot->current_axes();

            // Check if the coordinates vector is empty.
            if (vCoordinates.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Coordinates vector is empty. Cannot plot.");
                return;
            }

            // Check if the plot title is empty.
            if (szPlotTitle.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Plot title is empty. Setting title to default.");
                szPlotTitle = "UTMCoordinatePlot";
            }

            // Use matplotplusplus to plot the UTM coordinates as red dots with blue lines connecting them.
            std::vector<double> vEasting, vNorthing, vAltitude;
            for (const geoops::UTMCoordinate& stCoordinate : vCoordinates)
            {
                vEasting.push_back(stCoordinate.dEasting);
                vNorthing.push_back(stCoordinate.dNorthing);
                vAltitude.push_back(stCoordinate.dAltitude);
            }
            // Create a 3D plot
            mtAxes->plot3(vEasting, vNorthing, vAltitude, "-o");
            mtPlot->title(szPlotTitle);
            mtAxes->xlabel("Easting");
            mtAxes->ylabel("Northing");
            mtAxes->zlabel("Altitude");

            // Check if a file with the same title name already exists. If so then append a number to the end of the file name and recheck.
            std::string szFileName = logging::g_szLoggingOutputPath + "/path_plots/" + szTitle;
            int nFileNum           = 0;
            while (std::filesystem::exists(szFileName + ".png"))
            {
                szFileName = szFileName + std::to_string(nFileNum);
                ++nFileNum;
            }

            // Check if the final directory exists. If not then create it.
            if (!std::filesystem::exists(logging::g_szLoggingOutputPath + "/path_plots"))
            {
                std::filesystem::create_directory(logging::g_szLoggingOutputPath + "/path_plots");
            }
            // Save the plot
            mtPlot->save(szFileName + ".png");
        }

        /******************************************************************************
         * @brief Plot a 3D graph of GPS coordinates.
         *
         * @param vCoordinates - The vector of GPS coordinates to plot.
         * @param szTitle - The title of the plot. This will be the name of the file as well.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-08
         ******************************************************************************/
        inline void PlotCoordinates3D(const std::vector<geoops::GPSCoordinate>& vCoordinates, const std::string& szTitle = "GPSCoordinatePlot")
        {
            // Create instance variables.
            std::string szPlotTitle       = szTitle;
            matplot::figure_handle mtPlot = matplot::figure();
            matplot::axes_handle mtAxes   = mtPlot->current_axes();

            // Check if the coordinates vector is empty.
            if (vCoordinates.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Coordinates vector is empty. Cannot plot.");
                return;
            }

            // Check if the plot title is empty.
            if (szPlotTitle.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Plot title is empty. Setting title to default.");
                szPlotTitle = "GPSCoordinatePlot";
            }

            // Use matplotplusplus to plot the UTM coordinates as red dots with blue lines connecting them.
            std::vector<double> vLatitude, vLongitude, vAltitude;
            for (const geoops::GPSCoordinate& stCoordinate : vCoordinates)
            {
                vLatitude.push_back(stCoordinate.dLatitude);
                vLongitude.push_back(stCoordinate.dLongitude);
                vAltitude.push_back(stCoordinate.dAltitude);
            }
            // Create a 3D plot
            mtAxes->plot3(vLatitude, vLongitude, vAltitude, "-o");
            mtPlot->title(szPlotTitle);
            mtAxes->xlabel("Latitude");
            mtAxes->ylabel("Longitude");
            mtAxes->zlabel("Altitude");

            // Check if a file with the same title name already exists. If so then append a number to the end of the file name and recheck.
            std::string szFileName = logging::g_szLoggingOutputPath + "/path_plots/" + szTitle;
            int nFileNum           = 0;
            while (std::filesystem::exists(szFileName + ".png"))
            {
                szFileName = szFileName + std::to_string(nFileNum);
                ++nFileNum;
            }

            // Check if the final directory exists. If not then create it.
            if (!std::filesystem::exists(logging::g_szLoggingOutputPath + "/path_plots"))
            {
                std::filesystem::create_directory(logging::g_szLoggingOutputPath + "/path_plots");
            }
            // Save the plot
            mtPlot->save(szFileName + ".png");
        }

        /******************************************************************************
         * @brief Plot a 3D graph of waypoints.
         *
         * @param vWaypoints - The vector of waypoints to plot.
         * @param szTitle - The title of the plot. This will be the name of the file as well.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-08
         ******************************************************************************/
        inline void PlotCoordinates3D(const std::vector<geoops::Waypoint>& vWaypoints, const std::string& szTitle = "WaypointPlot")
        {
            // Create instance variables.
            std::string szPlotTitle       = szTitle;
            matplot::figure_handle mtPlot = matplot::figure();
            matplot::axes_handle mtAxes   = mtPlot->current_axes();

            // Check if the coordinates vector is empty.
            if (vWaypoints.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Waypoints vector is empty. Cannot plot.");
                return;
            }

            // Check if the plot title is empty.
            if (szPlotTitle.empty())
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "Plot title is empty. Setting title to default.");
                szPlotTitle = "WaypointPlot";
            }

            // Use matplotplusplus to plot the UTM coordinates as red dots with blue lines connecting them.
            std::vector<double> vEasting, vNorthing, vAltitude;
            for (const geoops::Waypoint& stWaypoint : vWaypoints)
            {
                vEasting.push_back(stWaypoint.GetUTMCoordinate().dEasting);
                vNorthing.push_back(stWaypoint.GetUTMCoordinate().dNorthing);
                vAltitude.push_back(stWaypoint.GetUTMCoordinate().dAltitude);
            }
            // Create a 3D plot
            mtAxes->plot3(vEasting, vNorthing, vAltitude, "-o");
            mtPlot->title(szPlotTitle);
            mtAxes->xlabel("Easting");
            mtAxes->ylabel("Northing");
            mtAxes->zlabel("Altitude");

            // Check if a file with the same title name already exists. If so then append a number to the end of the file name and recheck.
            std::string szFileName = logging::g_szLoggingOutputPath + "/path_plots/" + szTitle;
            int nFileNum           = 0;
            while (std::filesystem::exists(szFileName + ".png"))
            {
                szFileName = szFileName + std::to_string(nFileNum);
                ++nFileNum;
            }

            // Check if the final directory exists. If not then create it.
            if (!std::filesystem::exists(logging::g_szLoggingOutputPath + "/path_plots"))
            {
                std::filesystem::create_directory(logging::g_szLoggingOutputPath + "/path_plots");
            }
            // Save the plot
            mtPlot->save(szFileName + ".png");
        }
    }    // namespace graphing
}    // namespace logging
#endif    // PLOTS_AND_GRAPHS_HPP
