#include "lib/area_division.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <map>
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "area_division_node");
    ros::NodeHandle nh;

    area_division ad;

    // Larger sample occupancy grid (20x20)
    
    vector<signed char> grid = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0
    };

    // Initialize map
    // The map is a 20x20 grid
    int map_width = 20;
    int map_height = 20;
    float resolution = 1.0;
    ad.initialize_map(map_height, map_width, grid);
    int num_robots = 3;

    // Generate start_positions, removed using strings for robot names, just iterate
    std::map<std::string, std::vector<int>> cps_positions;
    for (int i = 1; i <= num_robots; i++) {
        std::string robot_name = "robot" + std::to_string(i);
        int x = rand() % map_height;
        int y = rand() % map_width;
        cps_positions[robot_name] = {x, y};
    }
    ad.initialize_cps(cps_positions);


    // Access the coordinates of the robots
    std::map<std::string, vector<int>> coordinates;
    for (int i = 1; i <= num_robots; i++) {
        std::string robot_name = "robot" + std::to_string(i);
        coordinates[robot_name] = cps_positions[robot_name];
    }
    

    // Create geometry_msgs::Point messages for starting positions
    std::map<std::string, geometry_msgs::Point> points;
    for (int i = 1; i <= num_robots; i++) {
        std::string robot_name = "robot" + std::to_string(i);
        geometry_msgs::Point point;
        point.x = coordinates[robot_name][0];
        point.y = coordinates[robot_name][1];
        points[robot_name] = point;
    }

    // Perform area division
    ad.divide();

    nav_msgs::OccupancyGrid map;
    map.info.width = map_height;
    map.info.height = map_width;
    map.info.resolution = resolution;  // Each cell represents 1x1 meter
    map.data = grid;

    // Publishers for the divided maps
    std::map<std::string, ros::Publisher> robot_pubs;
    std::map<std::string, ros::Publisher> start_pos_pubs;
    std::map<std::string, nav_msgs::OccupancyGrid> robot_grids;

    for (int i = 1; i <= num_robots; i++) {
        std::string robot_name = "robot" + std::to_string(i);

        nav_msgs::OccupancyGrid robot_grid = ad.get_grid(map, robot_name);
        ros::Publisher robot_pub = nh.advertise<nav_msgs::OccupancyGrid>(robot_name + "_grid", 10);
        ros::Publisher start_pos_pub = nh.advertise<geometry_msgs::Point>(robot_name + "_starting_pos", 1);

        robot_pubs[robot_name] = robot_pub;
        start_pos_pubs[robot_name] = start_pos_pub;
        robot_grids[robot_name] = robot_grid;
    }

    ros::Rate loop_rate(1); // 1 Hz
    while (ros::ok()) {
        for (int i = 1; i <= num_robots; i++) {
            std::string robot_name = "robot" + std::to_string(i);
            std::string robot_grid_name = robot_name + "_grid";
            
            robot_pubs[robot_name].publish(robot_grids[robot_name]);
            start_pos_pubs[robot_name].publish(points[robot_name]);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
