## This is the repository of group 13 from the Multi Disciplinary Project
In this document, all instructions to build the code and explanation about the implementation can be found.

### Path planning

To run the path planning section one must create a mirte workspace in which you clone this repository inside of the `src` folder.
```
source devel/setup.bash
catkin_make
```
To launch the seperate packages (area_division and coverage_path) write the following commands in different terminal windows.

```
roslaunch area_division area_division.launch
```

```
roslaunch coverage_path coverage_path.launch
```

To launch both the area_division and coverage_path packages write the following commands:

```
roslaunch area_division area_path.launch
```

Currently you have to hard code the amount of robots by changing the `num_robots` integer varable in both the packages source files `coverage_path.cpp main.cpp` and change the robot locations in `main.cpp` of the area_division package. Make sure that the amountof robot locations in `main.cpp` is equal to `num_robots` otherwise the code will not work.



<!-- As input publish to topic `area_division/robots' -->

