#include "lib/area_division.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <map>
#include <string>


class GlobalGrid {
    public:
        nav_msgs::OccupancyGrid occup_grid;
        bool map_received = false;
        ros::Subscriber grid_sub;
        ros::NodeHandle nh;

        GlobalGrid (ros::NodeHandle nodehandle) : nh(nodehandle) {
            grid_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &GlobalGrid::grid_callback, this);
        }

        void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
            occup_grid = *msg;
            for (int i = 0; i < occup_grid.data.size(); i++) {
                ROS_INFO("Data: %d", occup_grid.data[i]);
                if (occup_grid.data[i] == -1) {
                    occup_grid.data[i] = 100;
                }
            }
            map_received = true;
            ROS_INFO("Map received: width=%d, height=%d", occup_grid.info.width, occup_grid.info.height);
        }

};

class Robot {
    public:
        ros::Publisher robot_pub;
        ros::Publisher start_pos_pub;
        ros::NodeHandle nh;
        nav_msgs::OccupancyGrid robot_grid;
        int id;
        geometry_msgs::Point position;

    Robot (int id, ros::NodeHandle nodehandle, geometry_msgs::Point position) : id(id),  nh(nodehandle), position(position){
        robot_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot" + std::to_string(id) + "_grid", 10);
        start_pos_pub = nh.advertise<geometry_msgs::Point>("robot" + std::to_string(id) + "_starting_pos", 1);
    }

    void publish_grid() {
        robot_pub.publish(robot_grid);
    }
    void publish_start_pos() {
        start_pos_pub.publish(position);
    }
};

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "area_division_node");
    ros::NodeHandle nh;

    // Get the number of robots from the parameter server or default to 1
    int num_robots;
    nh.param("num_robots", num_robots, 1);

    // Initialize area division object
    area_division ad;

    // Create publishers for each robot
    std::vector<Robot> robots;
    for (int i = 0; i < num_robots; i++) {
        geometry_msgs::Point temp;
        temp.x = 214.0;
        temp.y = 2.0;
        robots.push_back(Robot(i, nh, temp));
    }

    // For now manually set the positin of a robot
    geometry_msgs::Point temp;
    temp.x = 120.0;
    temp.y = 30.0;
    robots[0].position = temp;

    GlobalGrid global_grid(nh);

    ros::Rate loop_rate(1); // 1 Hz
    while (ros::ok()) {
        if (global_grid.map_received) {
            ROS_INFO("Processing received map...");

            ad.initialize_map(global_grid.occup_grid.info.width, global_grid.occup_grid.info.height, global_grid.occup_grid.data);

            // Initialize CPS positions (ensure it's updated based on the number of robots)
            std::map<std::string, std::vector<int>> cps_positions;
            for (int i = 0; i < num_robots; i++) {
                std::string robot_name = "robot" + std::to_string(i);
                int x = int(robots[i].position.x);
                int y = int(robots[i].position.y);
                std::vector<int> pos = {x, y};
                ROS_INFO("Robot %s position: %d, %d", robot_name.c_str(), x, y);
                cps_positions[robot_name] = pos;
            }
            ad.initialize_cps(cps_positions);

            ROS_INFO("Dividing area");
            // Divide the area
            ad.divide();

            for (int i = 0; i < num_robots; i++) {
                std::string robot_name = "robot" + std::to_string(i);
                robots[i].robot_grid = ad.get_grid(global_grid.occup_grid, robot_name);
                robots[i].publish_grid();
                robots[i].publish_start_pos();
            }
            ROS_INFO("Processed map and published information");
        }
        loop_rate.sleep();
        global_grid.map_received = false;
        ros::spinOnce();
    }
    return 0;
}