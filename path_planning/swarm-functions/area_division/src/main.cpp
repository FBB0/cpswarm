#include "lib/area_division.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

            map_received = true;
            ROS_INFO("Map received: width=%d, height=%d", occup_grid.info.width, occup_grid.info.height);
        }

};

class Robot {
    public:
        ros::Publisher robot_pub;
        ros::Publisher start_pos_pub;
        ros::Subscriber amcl_pos_sub;
        ros::NodeHandle nh;
        nav_msgs::OccupancyGrid robot_grid;
        int id;
        geometry_msgs::Point position;

    Robot (int id, ros::NodeHandle nodehandle, geometry_msgs::Point position) : id(id),  nh(nodehandle), position(position){
        robot_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot" + std::to_string(id) + "_grid", 10);
        start_pos_pub = nh.advertise<geometry_msgs::Point>("robot" + std::to_string(id) + "_starting_pos", 1);
        amcl_pos_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &Robot::amcl_pos_callback, this);
    }

    void publish_grid() {
        robot_pub.publish(robot_grid);
    }
    void publish_start_pos() {
        start_pos_pub.publish(position);
    }

    void amcl_pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        position = msg->pose.pose.position;
        ROS_INFO("Robot %d position: x=%f, y=%f", id, position.x, position.y);
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

    // // For now manually set the positin of a robot
    // geometry_msgs::Point temp;
    // temp.x = 120.0;
    // temp.y = 30.0;
    // // temp.x = 1.0;
    // // temp.y = 1.0;
    // robots[0].position = temp;

    // // geometry_msgs::Point temp2;
    // // temp2.x = 19.0;
    // // temp2.y = 19.0;
    // // robots[1].position = temp2;

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
                ROS_INFO("%s position: %d, %d", robot_name.c_str(), x, y);
                cps_positions[robot_name] = pos;
            }
            ad.initialize_cps(cps_positions);

            ROS_INFO("Dividing area");
            // Divide the area
            ad.divide();

            for (int i = 0; i < num_robots; i++) {
                std::string robot_name = "robot" + std::to_string(i);
                robots[i].robot_grid = ad.get_grid(global_grid.occup_grid, robot_name);
                for (int j = 0; j < global_grid.occup_grid.data.size(); ++j) {
                    if (global_grid.occup_grid.data[j] == -1) {
                        robots[i].robot_grid.data[j] = 100;
                    }
                }
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