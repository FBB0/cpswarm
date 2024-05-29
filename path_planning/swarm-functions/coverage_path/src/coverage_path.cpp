#include "coverage_path.h"
#include <mutex>
#include <vector>
#include <boost/bind.hpp>

std::mutex grid_mutex;  // Mutex for thread-safe access to grid variables
 
int num_robots = 2;

class PathGenerator {
    public:
        int robot_id;
        ros::Subscriber robotGridSub;
        ros::Subscriber robotStartPositionSub;
        ros::Publisher path_publisher;
        geometry_msgs::Point start_position;
        nav_msgs::OccupancyGrid latest_robot_grid;
        ros::NodeHandle nh;

        //Constructor
        PathGenerator(int id, ros::NodeHandle nodehandle) : robot_id(id), nh(nodehandle){
            ROS_INFO_STREAM("/area_division/robot" + std::to_string(robot_id) + "_grid");
            ROS_INFO_STREAM("/area_division/robot" + std::to_string(robot_id) + "_starting_pos");
            robotStartPositionSub = nh.subscribe("/area_division/robot" + std::to_string(robot_id) + "_starting_pos", 10, &PathGenerator::robotStartPositionCallback, this);
            robotGridSub = nh.subscribe("/area_division/robot" + std::to_string(robot_id) + "_grid", 10, &PathGenerator::robotGridCallback, this);
            path_publisher = nh.advertise<nav_msgs::Path>("/coverage_path/path" + std::to_string(robot_id), 1, true);
        }

        //Callback functions
        void robotGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(grid_mutex);
            if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0) {
                ROS_WARN("Received an invalid grid. Robot %i will not generate a path.", robot_id);
                return;
            }
            ROS_INFO("Received Grid for Robot %i", robot_id);
            std::cout << robot_id << std::endl;
            // Process the grid data for robot
            latest_robot_grid = *msg; // Store the latest grid

            // generate_path(start_position, latest_robot_grid, path_publisher);
            generate_path();
        }
        void robotStartPositionCallback(const geometry_msgs::Point::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(grid_mutex);
            start_position = *msg;
            ROS_INFO("Robot %i", robot_id);
            ROS_INFO("Starting position for Robot %i received - x: %f, y: %f", robot_id, msg->x, msg->y);
        }

        /**
         * @brief Generate an optimal coverage path for a given area.
         * @param start The starting position of the path.
         * @param roi Pointer to a ROI to cover. Optional, if not given mission area is covered.
         * @return Whether the path has been generated successfully.
         */
        // bool generate_path (geometry_msgs::Point start, const nav_msgs::OccupancyGrid& robot_occupancy_map, ros::Publisher& path_publisher)
        bool generate_path ()
        {
            ROS_DEBUG("Starting at (%.2f,%.2f)", start_position.x, start_position.y);

            // get area divided per robot
            ROS_DEBUG("Get map of divided area...");

        
            nav_msgs::OccupancyGrid area = latest_robot_grid;

            // ROS_INFO("Occupancy Grid Info:");
            // ROS_INFO("  Width: %d", area.info.width);
            // ROS_INFO("  Height: %d", area.info.height);
            // ROS_INFO("  Resolution: %f meters/cell", area.info.resolution);
            // ROS_INFO("  Origin: (x: %f, y: %f, z: %f)", area.info.origin.position.x, area.info.origin.position.y, area.info.origin.position.z);

            ROS_INFO("Generate new coverage path...");

            // construct minimum spanning tree
            ROS_DEBUG("Construct minimum-spanning-tree...");
            spanning_tree tree;
            tree.initialize_graph(area, vertical);
            tree.construct();

            // visualize path
            if (visualize)
                mst_publisher.publish(tree.get_tree());

            // generate path
            ROS_DEBUG("Generate coverage path...");
            path.initialize_map(area, 0, vertical);
            // Check here
            path.initialize_tree(tree.get_mst_edges());
            if (!path.generate_path(start_position)) {
                ROS_WARN("Failed to generate path for the robot starting at (%.2f,%.2f)", start_position.x, start_position.y);
                return false;
                }
            if (turning_points)
                path.reduce();

            // visualize path
            // if (visualize)
            ROS_INFO("Visualize new coverage path...");

            path_publisher.publish(path.get_path());
            return true;
        }
};


/**
 * @brief A ROS node that computes the optimal paths for area coverage with a swarm of CPSs.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main (int argc, char **argv)
{
    // init ros node
    init(argc, argv, "coverage_path");
    NodeHandle nh;
    std::vector<PathGenerator*> path_objects;

    for (int i = 0; i < num_robots; i++) {
        path_objects.push_back(new PathGenerator(i, nh));
        ROS_INFO("PathGenerator %i created", i );
    }
    ros::Rate loop_rate(0.2);
    while(ros::ok()) {
        ROS_INFO("Path generation action server available");
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

