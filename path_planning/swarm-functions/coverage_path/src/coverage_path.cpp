#include "coverage_path.h"
#include <mutex>

std::mutex grid_mutex;  // Mutex for thread-safe access to grid variables
 

//Publishers
ros::Publisher path_publisher1;
ros::Publisher path_publisher2;
ros::Publisher path_publisher3;

// Shared variable to store the latest grid data
nav_msgs::OccupancyGrid latest_robot1_grid;
nav_msgs::OccupancyGrid latest_robot2_grid;
nav_msgs::OccupancyGrid latest_robot3_grid;

//Shared variable to store starting positions
geometry_msgs::Point start_position1;
geometry_msgs::Point start_position2;
geometry_msgs::Point start_position3;


// Callback functions to handle incoming grid data for each robot
void robot1GridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    

    std::lock_guard<std::mutex> lock(grid_mutex);
    if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0) {
        ROS_WARN("Received an invalid grid.");
        return;
    }
    geometry_msgs::Point start_position1;
    start_position1.x = 0.0;
    start_position1.y = 0.0;

     // Process the grid data for robot1
    latest_robot1_grid = *msg; // Store the latest grid


    generate_path(start_position1, latest_robot1_grid, path_publisher1);
}

// Callback for Robot 1 starting position
void robot1StartPositionCallback(const geometry_msgs::Point::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(grid_mutex);
    start_position1 = *msg;
    ROS_INFO("Starting position for Robot 1 received - x: %f, y: %f", msg->x, msg->y);
}

void robot2GridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    ROS_INFO("Grid for Robot 2 received with width %d and height %d", msg->info.width, msg->info.height);
    std::lock_guard<std::mutex> lock(grid_mutex);
    if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0) {
        ROS_WARN("Received an invalid grid.");
        ROS_INFO("RRRRRRRRRRRRR2");
        return;
    }
    // geometry_msgs::Point start_position2;
    // start_position2.x = 19.0;
    // start_position2.y = 19.0;

    // Process the grid data for robot1
    latest_robot2_grid = *msg; // Store the latest grid
    


    generate_path(start_position2, latest_robot2_grid, path_publisher2);
}

// Callback for Robot 2 starting position
void robot2StartPositionCallback(const geometry_msgs::Point::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(grid_mutex);
    start_position2 = *msg;
    ROS_INFO("Starting position for Robot 2 received - x: %f, y: %f", msg->x, msg->y);
}

void robot3GridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(grid_mutex);
    if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0) {
        ROS_WARN("Received an invalid grid.");
        ROS_INFO("RRRRRRRRRRRRR3");
        return;
    }

    // Process the grid data for robot1
    latest_robot3_grid = *msg; // Store the latest grid


    generate_path(start_position3, latest_robot3_grid, path_publisher3);
}

// Callback for Robot 3 starting position
void robot3StartPositionCallback(const geometry_msgs::Point::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(grid_mutex);
    start_position3 = *msg;
    ROS_INFO("Starting position for Robot 3 received - x: %f, y: %f", msg->x, msg->y);
}
/**
 * @brief Generate an optimal coverage path for a given area.
 * @param start The starting position of the path.
 * @param roi Pointer to a ROI to cover. Optional, if not given mission area is covered.
 * @return Whether the path has been generated successfully.
 */
bool generate_path (geometry_msgs::Point start, const nav_msgs::OccupancyGrid& robot_occupancy_map, ros::Publisher& path_publisher)
{
    NodeHandle nh;

    ROS_DEBUG("Starting at (%.2f,%.2f)", start.x, start.y);

    // get area divided per robot
    ROS_DEBUG("Get map of divided area...");

 
    nav_msgs::OccupancyGrid area = robot_occupancy_map;

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
    if (!path.generate_path(start)) {
        ROS_WARN("Failed to generate path for the robottarting at (%.2f,%.2f)", start.x, start.y);
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

    path_publisher1 = nh.advertise<nav_msgs::Path>("coverage_path/path1", 1, true);
    path_publisher2 = nh.advertise<nav_msgs::Path>("coverage_path/path2", 1, true);
    path_publisher3 = nh.advertise<nav_msgs::Path>("coverage_path/path3", 1, true);
  

    // Subscribers for the divided maps
    ros::Subscriber robot1_sub = nh.subscribe("robot1_grid", 10, robot1GridCallback);
    ros::Subscriber robot2_sub = nh.subscribe("robot2_grid", 10, robot2GridCallback);
    ros::Subscriber robot3_sub = nh.subscribe("robot3_grid", 10, robot3GridCallback);

    // Subscriber for the starting position of the Robots
    ros::Subscriber startPosSub_R1 = nh.subscribe("R1_starting_pos", 10, robot1StartPositionCallback);
    ros::Subscriber startPosSub_R2 = nh.subscribe("R2_starting_pos", 10, robot2StartPositionCallback);
    ros::Subscriber startPosSub_R3 = nh.subscribe("R3_starting_pos", 10, robot3StartPositionCallback);


    ROS_INFO("Path generation action server available");

    spin();

    return 0;
}
