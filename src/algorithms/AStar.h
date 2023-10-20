#include "AutonomyGlobals.h"
#include "Node.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

// TODO: geographicLib or other library

std::tuple<double, double> coords_obstacle(double distMeters, double lat1, double lon1, double bearing)
{    // TODO: get coords of obstacle given current position
}

std::vector<std::array<double, 2>> return_path(Node* current_node, std::array<double, 2>& utm_zone, bool return_gps = false)
{
    // Create instance variables.
    std::vector<std::array<double, 2>> path;
    const Node* current = current_node;

    // Loop backwards through each parent node until none exist.
    while (current != nullptr)
    {
        // Get coords from the node.
        std::array<double, 2> coords = {current->GetPositionFirst(), current->GetPositionSecond()};

        // Check if we should convert UTM to GPS.
        if (return_gps)
        {
            // TODO: convert UTM to GPS
        }

        // Append the current node's position.
        path.push_back(coords);

        // Set the current node equal to the current node's parent node.
        current = current->GetParent();
    }

    // Return the reversed path.
    std::reverse(path.begin(), path.end());
    return path;
}

class ASTAR
{
    public:
        std::vector<std::array<double, 2>> obstacleCoords;
        std::array<double, 2> utmZone;
        Node start;
        Node end;
        int max_queue_length;

        ASTAR(int max_queue_length) : max_queue_length(max_queue_length), utmZone({0, 0}) {}

        void update_obstacles(std::vector<std::array<double, 2>> obstacleLocations,
                              double min_object_distance = 1.0,
                              double max_object_distance = 15.0,
                              int min_object_angle       = -40,
                              int max_object_angle       = 40)
        {
            for (std::array<double, 2>& obstacle : obstacleLocations)
            {
                if ((obstacle[1] > min_object_distance && obstacle[1] < max_object_distance) &&    // If obstacle in angle and distance
                    (obstacle[0] > min_object_angle && obstacle[0] < max_object_angle))
                {
                    double angle = fmod(g_pNavigationBoardInterface.GetDData(NBPC_HEADING) + obstacle[0], 360.0);

                    double obstacle_lat, obstacle_lon;
                    try
                    {
                        std::tie(obstacle_lat, obstacle_lon) = coords_obstacle(obstacle[1],
                                                                               g_pNavigationBoardInterface.GetSData(NBPCC_LOCATION).dLatitude,
                                                                               g_pNavigationBoardInterface.GetSData(NBPCC_LOCATION).dLongitude,
                                                                               angle);
                    }
                    catch (...)
                    {
                        continue;
                    }

                    double obstacle_easting, obstacle_northing;
                    std::tie(obstacle_easting, obstacle_northing, std::ignore, std::ignore) =
                        utm.from_latlon(obstacle_lat, obstacle_lon);    // TODO: function to convert lat lon to utm

                    std::array<double, 2> utmCoords = {obstacle_easting, obstacle_northing};
                    obstacleCoords.push_back(utmCoords);

                    if (obstacleCoords.size() > max_queue_length)
                    {
                        obstacleCoords.erase(obstacleCoords.begin());
                    }
                }
            }
        }

        void update_obstacle_coords(std::vector<std::array<double, 2>> obstacleLocations, bool input_gps = true)
        {
            for (std::array<double, 2>& obstacleCoord : obstacleLocations)
            {
                if (input_gps)
                {
                    double obstacle_easting, obstacle_northing;
                    std::tie(obstacle_easting, obstacle_northing, std::ignore, std::ignore) =
                        utm.from_latlon(obstacleCoord[0], obstacleCoord[1]);    // TODO: function to convert lat lon to utm
                    std::array<double, 2> utmCoords = {obstacle_easting, obstacle_northing};
                    obstacleCoords.push_back(utmCoords);
                }
                else
                {
                    obstacleCoords.push_back(obstacleCoord);
                }

                if (obstacleCoords.size() > max_queue_length)
                {
                    obstacleCoords.erase(obstacleCoords.begin());
                }
            }
        }

        void clear_obstacles() { obstacleCoords.clear(); }

        std::vector<std::array<double, 2>> get_obstacle_coords()
        {
            std::vector<std::array<double, 2>> coords;

            if (utmZone[0] <= 0)
            {
                std::array<double, 2> current_gps_pos = {g_pNavigationBoardInterface.GetSData(NBPCC_LOCATION).dLatitude,
                                                         g_pNavigationBoardInterface.GetSData(NBPCC_LOCATION).dLongitude};
                auto current_utm_pos                  = utm.from_latlon(current_gps_pos[0], current_gps_pos.second);    // TODO: function to convert lat lon to utm
                utmZone                               = {current_utm_pos[0], current_utm_pos[1]};
            }

            for (std::array<double, 2>& obstacle : obstacleCoords)
            {
                auto coord = utm.to_latlon(obstacle[0], obstacle[1], utmZone[0], utmZone[1]);
                coords.push_back(coord);
            }

            return coords;
        }

        std::vector<std::array<double, 2>> ASTAR::plan_astar_avoidance_route(int max_route_size              = 10,
                                                                             double near_object_threshold    = 2.0,
                                                                             std::array<double, 2> start_gps = {0.0, 0.0},
                                                                             bool return_gps                 = false,
                                                                             double waypoint_thresh          = 0.5    // TODO: change to constant
        )
        {
            // Determine start position.
            std::array<double, 2> current_gps_pos;

            if (start_gps[0] == 0.0 && start_gps[1] == 0.0)
            {
                // Get current gps position.
                current_gps_pos = {g_pNavigationBoardInterface.GetSData(NBPCC_LOCATION).dLatitude, g_pNavigationBoardInterface.GetSData(NBPCC_LOCATION).dLongitude};
            }
            else
            {
                // Use the given start position.
                current_gps_pos = start_gps;
            }

            // Convert the gps coords to UTM coords. These coords are in meters and they are easier to work with.
            std::array<double, 2> current_utm_pos = utm.from_latlon(current_gps_pos[0], current_gps_pos[1]);    // TODO: utm conversion
            utmZone                               = {current_utm_pos[0], current_utm_pos[1]};

            // Create start and end node.
            std::array<double, 2> utmCoords = {current_utm_pos[0], current_utm_pos[1]};
            start                           = Node(nullptr, utmCoords);

            // Calculate gps coord fixed distance in front of the rover.
            auto gps_data                      = core.waypoint_handler.get_waypoint();    // TODO change to get data correctly
            auto waypoint_goal                 = gps_data.data().first;
            auto goal_utm_pos                  = utm.from_latlon(waypoint_goal.first, waypoint_goal.second);
            std::array<double, 2> goal_utm_pos = {goal_utm_pos.first, goal_utm_pos.second};
            end                                = Node(nullptr, goal_utm_pos);

            // Create open and closed list.
            std::vector<Node> open_list;
            std::vector<Node> closed_list;

            open_list.push_back(start);
            make_heap(open_list.begin(), open_list.end());

            // Define a stop condition.
            int outer_iterations = 0;
            int max_interations  = max_route_size * max_route_size / 2;

            // Define movement search pattern. In this case, check in a grid pattern 0.5 meters away from the current position.
            double offset = 0.1;    // TODO: change to constant
            std::vector<std::array<double, 2>> adjacent_movements =
                {{0.0, -offset}, {0.0, offset}, {-offset, 0.0}, {offset, 0.0}, {-offset, -offset}, {-offset, offset}, {offset, -offset}, {offset, offset}};

            // Loop until the algorithm has found the end.
            while (!open_list.empty())
            {
                // Increment counter.
                outer_iterations++;

                // Get the current node.
                Node current_node = open_list.front();
                pop_heap(open_list.begin(), open_list.end());
                open_list.pop_back();
                closed_list.push_back(current_node);

                // Check if we have hit the maximum number of iterations.
                if (outer_iterations > max_interations)
                {
                    // Print info message.
                    LOG_WARNING(g_qSharedLogger, "Unable to solve path: too many iterations.");
                    return return_path(&current_node, utmZone, return_gps);
                }

                // Found the goal.
                if (std::fabs(current_node.GetPositionFirst() - end.GetPositionFirst()) <= waypoint_thresh &&
                    std::fabs(current_node.GetPositionSecond() - end.GetPositionSecond()) <= waypoint_thresh)
                {
                    return return_path(&current_node, utmZone, return_gps);
                }

                // Generate children locations for the current node.
                std::vector<Node> children;
                for (const std::array<double, 2>& new_position : adjacent_movements)
                {
                    // Calculate child node position.
                    std::array<double, 2> child_node_pos = {current_node.GetPositionFirst() + new_position[0], current_node.GetPositionSecond() + new_position[1]};

                    // Check if the new child node is within range of our specified area.
                    if (std::fabs(child_node_pos[0] - start.GetPositionFirst()) > max_route_size ||
                        std::fabs(child_node_pos[1] - start.GetPositionSecond()) > max_route_size)
                    {
                        continue;
                    }

                    // Make sure we are not close to another obstacle.
                    bool coord_too_close = false;
                    for (const std::array<double, 2>& coord : obstacleCoords)
                    {
                        // Calculate the straight-line distance of the robot position from the obstacle.
                        double robot_distance_from_obstacle =
                            std::sqrt(std::pow(current_utm_pos[0] - child_node_pos[0], 2) + std::pow(current_utm_pos[1] - child_node_pos[1], 2));
                        // Calculate the straight-line distance of the new node from the obstacle.
                        double node_distance_from_obstacle = std::sqrt(std::pow(coord[0] - child_node_pos[0], 2) + std::pow(coord[1] - child_node_pos[1], 2));
                        // Check if we are getting closer to the obstacle.
                        if (node_distance_from_obstacle <= near_object_threshold)
                        {
                            coord_too_close = true;
                        }
                        // Check if the robot is within a circle radius of the obstacle and pick the point that will move us away from it.
                        if (robot_distance_from_obstacle < near_object_threshold && node_distance_from_obstacle > robot_distance_from_obstacle)
                        {
                            coord_too_close = false;
                        }
                    }
                    // If the current child node is too close to the obstacle, skip it.
                    if (coord_too_close)
                    {
                        continue;
                    }

                    // Create a new node with child properties.
                    Node new_node(&current_node, child_node_pos);

                    // If everything checks out, add the node to the list.
                    children.push_back(new_node);
                }

                // Loop through children, calculate cost, and make a move.
                for (Node& child : children)
                {
                    // Check if the child is already on the closed list.

                    for (const Node& closed_child : closed_list)
                    {
                        if (child == closed_child)
                        {
                            continue;
                        }
                    }

                    // Calculate f, g, and h values.
                    child.SetGValue(child.GetGValue() + offset);
                    child.SetHValue(std::pow(child.GetPositionFirst() - end.GetPositionFirst(), 2) + std::pow(child.GetPositionSecond() - end.GetPositionSecond(), 2));

                    // Check if the child is already in the open list or a child exists that has a greater cost.
                    for (const Node& open_node : open_list)
                    {
                        if (child == open_node && child.GetGValue() > open_node.GetGValue())
                        {
                            continue;
                        }
                    }

                    // Add the child to the open list.
                    open_list.push_back(child);
                    push_heap(open_list.begin(), open_list.end());
                }
            }
            // If unable to calulate path, then return nothing
            LOG_WARNING(g_qSharedLogger, "Couldn't find path around obstacle to destination.");
            std::vector<std::array<double, 2>> failReturn = {};
            return failReturn;
        }
};
