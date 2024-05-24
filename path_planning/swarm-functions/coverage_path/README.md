# coverage_path
[![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__coverage_path__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__coverage_path__ubuntu_xenial__source/)

This package generates an optimal path to cover a given area with a cyber physical system (CPS). This package has been adapted by group 13 and 15 for RO47007 Multidisciplinary Project 2023-2024 of the TU Delft.

This package was adapted by Felipe Bononi Bello (fbononibello@gmail.com) and Ming Da Yang (m.d.yang@student.tudelft.nl).


## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [nav_msgs](https://wiki.ros.org/nav_msgs)

The following packages of the [swarm functions library](https://github.com/cpswarm/swarm_functions) are required:
* area_division 


Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)
* [tf2](https://wiki.ros.org/tf2/)

## Execution
Run the launch file
```
roslaunch coverage_path coverage_path.launch
```
to launch the `coverage_path` node.

The launch file can be configured with following parameters:
* `id` (integer, default: `1`)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: `screen`)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

In the `param` subdirectory there is the parameter file `coverage_path.yaml` that allows to configure the behavior of the `coverage_path` node.

## Nodes

### coverage_path
The `coverage_path` node generates a path that allows a CPS to cover a given area. The area can be provided in one way:

1. If `divide_area=true`, the map is retrieved from the `area_division` package. It divides the area to be covered among multiple CPSs and provides the grid map of the area assigned to this CPS. When the swarm composition changes, the map is retrieved again and the path is regenerated.

The generated coverage path is based on a [minimum spanning tree](https://en.wikipedia.org/wiki/Minimum_spanning_tree) to optimally sweep the area. The path can be retrieved as a whole (action). In the latter case, the current waypoint is returned, based on the current position of the CPS.


#### Published Topics
* `coverage_path/path{i}` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
  The generated path for visualization purposes. All published topics: `/coverage_path/path1`,`/coverage_path/path2`,`/coverage_path/path3`

#### Parameters
Not all parameters are currently being used in our adapted implementation.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.
* `~resolution` (real, default: `1.0`)
  The spacing between two adjacent coverage path legs in meter.
* `~visualize` (boolean, default: `false`)
  Whether to publish the coverage path on a topic for visualization.
* `~divide_area` (boolean, default: `false`)
  Whether to divide the area among the CPSs in the swarm before generating the path. Joining or leaving swarm members will trigger regeneration of the path.
* `~vertical` (boolean, default: `false`)
  Whether the sweeping pattern is vertical or horizontal.
* `~turning_points` (boolean, default: `true`)
  Whether there are only waypoints at turning points of the path or also waypoints regularly spaced on straight line segments of the path.
* `swarm_timeout` (real, default: `5.0`)
  The time in seconds after which it is assumed that a swarm member has left the swarm if no messages have been received. Only in combination with `divide_area=true`.
* `states` (string list, default `[]`)
  Only CPSs in these states are considered part of the swarm. Only in combination with `divide_area=true`.

## Legacy Code API

[coverage_path package code API documentation](https://cpswarm.github.io/swarm_functions/coverage_path/docs/html/files.html)
