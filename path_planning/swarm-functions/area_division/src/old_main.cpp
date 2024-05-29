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
        // Dummy variables so nothing breaks
        int width = 20;
        int height = 20;
        float resolution = 1.0;
        ros::NodeHandle nh;
        ros::Subscriber sub;
        area_division ad;
        map<string, vector<int>> cps_positions = {
            {"robot0", {100, 40}}
            // {"robot1", {0, 0}}
        };

        GridMap(ros::NodeHandle nodehandle) : nh(nodehandle) {
            ROS_INFO("Subscribing to map topic");
            sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &GridMap::grid_callback, this);
            ROS_INFO("Initializing map");
            ad.initialize_map(height, width, grid);

            ROS_INFO("Initializing CPSs");
            ad.initialize_cps(cps_positions);
        }

        void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
            occup_grid = *msg;
            width = occup_grid.info.width;
            height = occup_grid.info.height;
            resolution = occup_grid.info.resolution;
            
            // Remove all -1, code does not account of unexplored areas
            for (int i = 0; i < occup_grid.data.size(); ++i) {
            if (occup_grid.data[i] == -1) {
                occup_grid.data[i] = 100;
            }
            }
            grid = occup_grid.data;
            ROS_INFO("Received map!");
            

            ROS_INFO("Width: %d, Height: %d, Resolution: %f", width, height, resolution);

            // Initialize map
            ROS_INFO("Initializing map");
            ad.initialize_map(height, width, grid);

            ROS_INFO("Initializing CPSs");
            ad.initialize_cps(cps_positions);
        }

        nav_msgs::OccupancyGrid get_occupancy_grid() {
            // for some reason this works and returnignt the message immediately doesn't
            // so I am gonna use this ugly fix for now
            nav_msgs::OccupancyGrid temp;
            temp.info.width = width;
            temp.info.height = height;
            temp.info.resolution = resolution;
            temp.data = grid;
            return temp;
        }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "area_division_node");
    ros::NodeHandle nh;
    

    // Start the grid map susbscriber to the /map topic
    GridMap gridmap(nh);
    int num_robots = 1;


    ROS_INFO("Create publishers");
    // Publishers for the divided maps
    std::map<std::string, ros::Publisher> robot_pubs;
    std::map<std::string, ros::Publisher> start_pos_pubs;
    std::map<std::string, nav_msgs::OccupancyGrid> robot_grids;

    for (int i = 0; i < num_robots; i++) {
        std::string robot_name = "robot" + std::to_string(i);

        ros::Publisher robot_pub = nh.advertise<nav_msgs::OccupancyGrid>(robot_name + "_grid", 10);
        ros::Publisher start_pos_pub = nh.advertise<geometry_msgs::Point>(robot_name + "_starting_pos", 1);

        robot_pubs[robot_name] = robot_pub;
        start_pos_pubs[robot_name] = start_pos_pub;
    }

    ros::Rate loop_rate(1); // 1 Hz
    while (ros::ok()) {
        // Generate start_positions, removed using strings for robot names, just iterate
        ROS_INFO("Access the coordinates of the robots");
        // Access the coordinates of the robots
        std::map<std::string, vector<int>> coordinates;
        for (int i = 0; i < num_robots; i++) {
            std::string robot_name = "robot" + std::to_string(i);
            coordinates[robot_name] = gridmap.cps_positions[robot_name];
        }
        
        ROS_INFO("Create geometry_msgs::Point messages for starting positions");
        // Create geometry_msgs::Point messages for starting positions
        std::map<std::string, geometry_msgs::Point> points;
        for (int i = 0; i < num_robots; i++) {
            std::string robot_name = "robot" + std::to_string(i);
            geometry_msgs::Point point;
            point.x = coordinates[robot_name][0];
            point.y = coordinates[robot_name][1];
            points[robot_name] = point;
        }

        ROS_INFO("Perform area division");
        // Perform area division
        gridmap.ad.divide();
        ROS_INFO("Divided areas");
        for (int i = 0; i < num_robots; i++) {
            std::string robot_name = "robot" + std::to_string(i);
            std::string robot_grid_name = robot_name + "_grid";
            nav_msgs::OccupancyGrid total_occupancy_grid = gridmap.get_occupancy_grid();
            ROS_INFO("Got large occupancy map");
            nav_msgs::OccupancyGrid robot_grid = gridmap.ad.get_grid(total_occupancy_grid, robot_name);
            ROS_INFO("Created individual robot grid");
            
            robot_grids[robot_name] = robot_grid;
            robot_pubs[robot_name].publish(robot_grids[robot_name]);
            start_pos_pubs[robot_name].publish(points[robot_name]);
            

        }
        ROS_INFO("Publishing found area's");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
