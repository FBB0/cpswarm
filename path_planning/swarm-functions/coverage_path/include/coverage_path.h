#ifndef COVERAGE_PATH_H
#define COVERAGE_PATH_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>
#include "lib/spanning_tree.h"
#include "lib/mst_path.h"

using namespace std;
using namespace ros;


/**
 * @brief Publisher to visualize the coverage path.
 */
Publisher path_publisher;

// /**
//  * @brief Publisher to visualize the current waypoint.
//  */
// Publisher wp_publisher;

/**
 * @brief Publisher to visualize the minimum spanning tree.
 */
Publisher mst_publisher;


/**
 * @brief The coverage path.
 */
mst_path path;


/**
 * @brief The spacing between two adjacent coverage path legs in meter.
 */
double resolution;

/**
 * @brief Whether to publish the coverage path on a topic for visualization.
 */
bool visualize;

/**
 * @brief Whether to divide the area among the CPSs in the swarm before generating the path. Joining or leaving swarm members will trigger regeneration of the path.
 */
bool divide_area;

/**
 * @brief Whether the sweeping pattern is vertical or horizontal.
 */
bool vertical;

/**
 * @brief Whether there are only waypoints at turning points of the path or also waypoints regularly spaced on straight line segments of the path.
 */
bool turning_points;

/**
 * @brief Generate an optimal coverage path for a given area.
 * @param start The starting position of the path.
 * @param robot_occupancy_map The occupancy grid to use for path planning.
 * @return Whether the path has been generated successfully.
 */
bool generate_path(geometry_msgs::Point start, const nav_msgs::OccupancyGrid& robot_occupancy_map, ros::Publisher& path_publisher);



// External publisher declarations
extern Publisher path_publisher1;
extern Publisher path_publisher2;
extern Publisher path_publisher3;

// /**
//  * @brief Whether the swarm configuration has changed which requires a replanning of the path.
//  */
// bool reconfigure;

#endif // COVERAGE_PATH_H
