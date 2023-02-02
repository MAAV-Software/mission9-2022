# MAAV Software Onboarding Project 2022

This document describes the onboarding project for new software members. This project focuses on developing an understanding of how the Gazebo simulation environment works, solid ROS skills, and the interface between ROS and the PX4 flight controller. Collaboration with others is highly encouraged and expected. 

There are three main parts to the onboarding project:
- `Simple waypoint following` - an expansion on the "starter code" given in the mission8_2022/software_ws/ folder. This part should be completed by all members.
- `Trajectory Following` - Building off simple waypoint following, trajectory following allows the drone to follow much more complicated paths. This part should be completed by members interested in the control and navigation of the drone.
- `Simple computer vision` - This task will require the drone to make simple decisions based on its sensor input from a depth camera. The depth camera will be added to the drone in the Gazebo simulator. This part should be completed by members interested in compter vision.

By the end of this onboarding project, you'll have the skills necesarry to be able to work on code directly for our vehicle! If you have absoluely any issues or questions throughout this process please reach out to me (Drew) and we can work through it together!

**NOTE**: It is expected that you have completed the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials) in C++ before beginning this onboarding project.

Happy coding!

## Git and version control
To get you familiar with git, we would for you to create your own branch off of the current master branch to keep track of your code. Please follow the naming conventions of your branch to follow the format `[uniqname]/[branch_name]` (i.e drewskis/onboarding). 

Please refer to  [these git resources](../README.md) for more information about git. You can never commit/push too much so please do so often!

## Running the simulation with ROS
In this section I will demonstrate what is required to boot up a simulation and run the code provided in the "example" package in our catkin_ws. This package is located at /mission9-2022/software_ws/example and has simple waypoint generation code already written. 

To run code from this package, or any other package you create, you will need to `open up 4 different terminals`.

### **In Terminal 1, run:**
```
$ cd /px4_sitl/PX4-Autopilot
$ make px4_sitl gazebo
```
This starts the gazebo simulation with an Iris drone running the PX4 flight controller code. We will interact with this drone in simulation with our code. 
### **In Terminal 2, run:**
```
$ roscore
```
### **In Terminal 3, run:**
Only need to run the first command once...
```
$ sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh 
$ roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14540"
```
This starts the mavros node and initializes communication with the PX4 flight controller. It basically publishes a bunch of topics from the PX4 to ROS so that we can interact with it in our code
### **In Terminal 4, run:**
```
$ cd /mission9-2022/software_ws
$ catkin_make
$ source ./devel/setup.bash
$ rosrun example takeoff_n_land 
```
This actually starts running our code! You should see the drone in Gazebo responding to commands! Note that if you created your own package, you would instead run "rosrun [package_name] [node_name]"


## Simple Waypoint Following 
The first task in this onboarding project is simple waypoint following. Before beginning to code, please take a look at the code for the example waypoint navigation package that we observed above. 

```
$ cd /mission9-2022/software_ws/src/example
```

In the /src folder you should find a file named `takeoff_n_land.cpp` which you can inspect. This file employs many of the paradigms you learned in the ROS tutorials (i.e. subscibers, publishers, callbacks, services, clients, etc). Make sure you understand what every part of the code is doing.

One thing that you may notice is that the "waypoint following" is quite gross. 
1. All of the waypoints are hard coded 
2. This is an `open-loop` which means it has effectively no way of knowing it actually got to the waypoint. It's completely based on a timer. 
3. Current position is not considered when calculating if a waypoint is reached

### Your Task
What I would like you to do for this part of the onboarding project is basically fix all the problems in the current waypoint following algorithm. What would be ideal is a program where we do the following:
- Encode a set of waypoints (say, in a vector) and iteratively go from waypoint to waypoint
- Keep track of of the current position of the drone somehow
- Only move on to the next waypoint when the drone has gotten close enough to the current waypoint.

You may use `takeoff_n_land.cpp` as a starting point but please create a new ros package and develop your code there. It's valuable to go through that again.

You should create a new ros package that depends on mavros by doing the following: 
```
$ cd /mission9-2022/software_ws/src/
$ catkin_create_pkg [package_name_here] mavros
```

If you are still fuzzy on how to set up and link C++ programs to your package please reference the ROS tutorials. 

Every time you write or change you code, remember to run "catkin_make". You can do this by:
```
$ cd /mission9-2022/software_ws/
$ catkin_make
```

**Hints:**
- I recommend taking advantage of `rostopic list`, `rostopic show [name]`, and `rostopic echo [name]` to figure out what data you have access to!
- If you're stuck, it's probably my fault for writing crappy documentation lol. So reach out to me!

## Trajectory Following
After completing simple waypoint following, you are ready to move on to trajectory following with ROS. At the end of this section you should be able to move the px4 drone in a more complicated pattern (i.e. in a circle, in an S pattern, etc). 

The way trajectory following works is actually pretty simple. There are basically two ways to go about it:

1. `The carrot following algorithm` break down your desired path into many smaller, intermediate waypoints. You'll need to do some geometry for finding what the "heading direction" should be at that particular point. 
2. `Bézier curves` which you can learn about [here](https://en.wikipedia.org/wiki/B%C3%A9zier_curve). PX4 should have an automatic way to publish direct Bézier curves which you can learn about [here](https://docs.px4.io/v1.12/en/computer_vision/path_planning_interface.html).

Additionally, you can have a pretty big impact on the flight "fluidity" by simply adjusting the range you consider close enough to the current waypoint.

**Note: Successfully completing this part of the onboarding project would seriously improve the software stack so thank you for your work!!**

## Simple Computer Vision
First, set up the camera used in simulation by following along with [this documentation](./SimCamera.md)

Next, Ask Drew, still working out some stuff.

\
\
\
\

Wanna hear a joke?
... \
Knock Knock ...
