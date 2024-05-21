#include "lib/area_division.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map>

int main(int argc, char **argv) {
    ros::init(argc, argv, "area_division_node");
    ros::NodeHandle nh;

    area_division ad;

    // Larger sample occupancy grid (20x20)
    vector<signed char> grid = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0,
        0, 0, 100, 100, 0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0
    };

    // Initialize map
    ad.initialize_map(20, 20, grid);

    // Initialize CPS positions
    map<string, vector<int>> cps_positions = {
        {"robot1", {0, 0}},
        {"robot2", {19, 19}}, 
        {"robot3", {1,1}}
    };
    ad.initialize_cps(cps_positions);

    // Perform area division
    ad.divide();

    // Publishers for the divided maps
    ros::Publisher robot1_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot1_grid", 10);
    ros::Publisher robot2_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot2_grid", 10);
    ros::Publisher robot3_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot3_grid", 10);
    
    // Example usage of get_grid and publish the maps
    nav_msgs::OccupancyGrid map;
    map.info.width = 20;
    map.info.height = 20;
    map.info.resolution = 1.0;  // Each cell represents 1x1 meter
    map.data = grid;

    nav_msgs::OccupancyGrid robot1_grid = ad.get_grid(map, "robot1");
    nav_msgs::OccupancyGrid robot2_grid = ad.get_grid(map, "robot2");
    nav_msgs::OccupancyGrid robot3_grid = ad.get_grid(map, "robot3");

    ros::Rate loop_rate(1); // 1 Hz
    while (ros::ok()) {
        robot1_pub.publish(robot1_grid);
        robot2_pub.publish(robot2_grid);
        robot3_pub.publish(robot3_grid);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
