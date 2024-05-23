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
    ad.initialize_map(20, 20, grid);

    // Initialize CPS positions
    map<string, vector<int>> cps_positions = {
        {"robot1", {0, 0}},
        {"robot2", {19, 19}}, 
        {"robot3", {1,1}}
    };
    ad.initialize_cps(cps_positions);

    // Access the coordinates of the robots
    std::vector<int> coordinates_R1 = cps_positions["robot1"];
    std::vector<int> coordinates_R2 = cps_positions["robot2"];
    std::vector<int> coordinates_R3 = cps_positions["robot3"];

    // Create geometry_msgs::Point messages for starting positions
    geometry_msgs::Point point_R1, point_R2, point_R3;
    point_R1.x = coordinates_R1[0];
    point_R1.y = coordinates_R1[1];
    point_R2.x = coordinates_R2[0];
    point_R2.y = coordinates_R2[1];
    point_R3.x = coordinates_R3[0];
    point_R3.y = coordinates_R3[1];

    // Perform area division
    ad.divide();

    // Publishers for the divided maps
    ros::Publisher robot1_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot1_grid", 10);
    ros::Publisher robot2_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot2_grid", 10);
    ros::Publisher robot3_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot3_grid", 10);

    //Publishers for the starting points
    ros::Publisher R1_start_pos_pub = nh.advertise<geometry_msgs::Point>("R1_starting_pos", 1);
    ros::Publisher R2_start_pos_pub = nh.advertise<geometry_msgs::Point>("R2_starting_pos", 1);
    ros::Publisher R3_start_pos_pub = nh.advertise<geometry_msgs::Point>("R3_starting_pos", 1);
    
    
    // Example usage of get_grid and publish the maps
    // This will be replaced by the perception map
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

        R1_start_pos_pub.publish(point_R1);
        R2_start_pos_pub.publish(point_R2);
        R3_start_pos_pub.publish(point_R3);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
