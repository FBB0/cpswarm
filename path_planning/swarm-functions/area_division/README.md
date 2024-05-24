# area_division
This package is based on the area_division package from cpswarm (https://github.com/cpswarm/swarm_functions/tree/noetic-devel/area_division) and has been adapted by group 13 and group 15 for RO47007 Multidisciplinary Project 2023-2024 of the TU Delft.

This package was adapted by Felipe Bononi Bello (fbononibello@gmail.com) and Ming Da Yang (m.d.yang@student.tudelft.nl).


This package divides the available environment area among multiple robots in a swarm. 

## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [nav_msgs](https://wiki.ros.org/nav_msgs)
* [geometry_msgs](http://wiki.ros.org/geometry_msgs)

The area_division package needs an occupancy grid, which will be received from the perception team. The occupancy grid will show where the static obstacles in the barn are.

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)

## Execution
Run the launch file
```
roslaunch area_division area_division.launch
```
to launch the `area_division` node.

Then in another terminal launch rviz.
## Nodes

### area_division
The `area_division` divides the environment area among multiple robots. The division algorithm is based on the [DARP algorithm](https://github.com/athakapo/DARP) which tries to divide the area optimally. Each robot is assigned an equal share of the environment that includes its current position. 

For further development the area_division could be made dynamic, by accounting for the amount of manure on the floor.

#### Subscribed Topics
<!-- * `state` ([cpswarm_msgs/StateEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/StateEvent.html))
  The behavior state of this CPS.
* `swarm_state` ([cpswarm_msgs/ArrayOfStates](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfStates.html))
  The behavior states of the other CPSs.-->
<!-- * `pos_provider/pose` ([geometry_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)) 
  The current position of this CPS. -->
* `area/map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
  The map to be divided.
<!-- * `bridge/uuid` ([swarmros/String](https://cpswarm.github.io/swarmio/swarmros/msg/String.html))
  The UUID of this CPS.
* `bridge/events/area_division` ([cpswarm_msgs/AreaDivisionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/AreaDivisionEvent.html))
  The area division requests from other CPSs. Messages are exchanged between CPSs using the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio). -->

#### Published Topics
<!-- * `pos_controller/goal_position` ([geometry_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The topic for stoping the CPS.
* `area_division` ([cpswarm_msgs/AreaDivisionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/AreaDivisionEvent.html))
  The topic for requesting area division among the available CPSs in the swarm. The request is forwarded by the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio) to the other swarm members.
* `area/assigned` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
  The area assigned to this CPS.
* `area/rotated` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
  The area to be divided, rotated so the lower boundary is horizontal. For visualization purposes, only published if the parameter `visualize` is set to true.
* `area/downsampled` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
  The area to be divided, downsampled to a lower resolution. For visualization purposes, only published if the parameter `visualize` is set to true. -->
* ` nav_msgs/OccupancyGrid` per robot there exists a robot{i}_grid i for the number of the robot.

* `geometry_msgs/Point` per robot there exists a robot{i}_starting_pos i for the number of robots, to publish the starting point.

<!-- #### Services Called
* `area/get_rotation` ([cpswarm_msgs/GetDouble](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetDouble.html))
  Get the rotation required to align the lower boundary of the area horizontally. -->

<!-- #### Parameters
* `~loop_rate` (real, default: `1.5`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `10`)
  The size of the message queue used for publishing and subscribing to topics.
* `resolution` (real, default: `1.0`)
  The grid map underlying the area division will be downsampled to this resolution in meter / cell.
* `~swarm_timeout` (real, default: `5.0`)
  The time in seconds communication in the swarm can be delayed at most. Used to wait after an area division event before starting the area division or time after which it is assumed that a swarm member has left the swarm if no position update has been received.
* `~visualize` (boolean, default: `false`)
  Whether to publish the area division on a topic for visualization.
* `~states` (string list, default: `[]`)
  Only CPSs in these states divide the area among each other.
* `~/optimizer/iterations` (integer, default: `10`)
  Maximum number of iterations of the optimization algorithm.
* `~/optimizer/variate_weight` (real, default: `0.01`)
  Maximum variate weight of connected components.
* `~/optimizer/discrepancy` (integer, default: `30`)
  Maximum difference between number of assigned grid cells to each CPS. -->

## Code API
For the original swarm_code there exists an API documentation
[area_division package code API documentation](https://cpswarm.github.io/swarm_functions/area_division/docs/html/files.html)