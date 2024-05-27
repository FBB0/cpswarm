#include "lib/area_division.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <map>
#include <string>

class GridMap {
    public:
        nav_msgs::OccupancyGrid occup_grid;
        std::vector<signed char> grid;
        // Dummy variables so nothing breaks
        int width = 10;
        int height = 10;
        float resolution = 1.0;
        ros::NodeHandle nh;
        ros::Subscriber sub;

        GridMap(ros::NodeHandle nodehandle) : nh(nodehandle) {
            ROS_INFO("Subscribing to map topic");
            sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &GridMap::grid_callback, this)
        }

    void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        occup_grid = msg;
        width = msg.info.width;
        height = msg.info.height;
        resolution = msg.info.resolution;
        grid = msg.data;
        ROS_INFO("Received map!");
        ROS_INFO("Width: %d, Height: %d, Resolution: %f", map.info.width, map.info.height, map.info.resolution);
    }
};




int main(int argc, char **argv) {
    ros::init(argc, argv, "area_division_node");
    ros::NodeHandle nh;

    area_division ad;

    // Larger sample occupancy grid (20x20)
    // TODO load occupancy grid from file
    GridMap gridmap(nh);

    // Initialize map

    ad.initialize_map(gridmap.height, gridmap.width, gridmap.grid);
    int num_robots = 2;

    // Generate start_positions, removed using strings for robot names, just iterate
    map<string, vector<int>> cps_positions = {
        {"robot0", {11, 11}},
        {"robot1", {10, 0}}
    };

    ad.initialize_cps(cps_positions);


    // Access the coordinates of the robots
    std::map<std::string, vector<int>> coordinates;
    for (int i = 0; i < num_robots; i++) {
        std::string robot_name = "robot" + std::to_string(i);
        coordinates[robot_name] = cps_positions[robot_name];
    }
    

    // Create geometry_msgs::Point messages for starting positions
    std::map<std::string, geometry_msgs::Point> points;
    for (int i = 0; i < num_robots; i++) {
        std::string robot_name = "robot" + std::to_string(i);
        geometry_msgs::Point point;
        point.x = coordinates[robot_name][0];
        point.y = coordinates[robot_name][1];
        points[robot_name] = point;
    }

    // Perform area division
    ad.divide();



    // Publishers for the divided maps
    std::map<std::string, ros::Publisher> robot_pubs;
    std::map<std::string, ros::Publisher> start_pos_pubs;
    std::map<std::string, nav_msgs::OccupancyGrid> robot_grids;

    for (int i = 0; i < num_robots; i++) {
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
        for (int i = 0; i < num_robots; i++) {
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
