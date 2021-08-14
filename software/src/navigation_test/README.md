# Waypoint Navigation
This package is a very barebones and basic waypoint navigation system. The system is split up into two nodes: offboard_node (defined in src/offboard_node.cpp), and 
waypoint_publisher (defined in src/pub_setpoints.cpp). 

https://user-images.githubusercontent.com/54146662/129458301-06262b32-ba18-42e9-a051-a12a52237a19.mp4

As you can see... the landing is a bit nonexistent currently.


## Setup the environment
After cloning the [maav mission9](https://github.com/MAAV-Software/mission9-2021) repo and setting up the docker container, navigate to the project folder, open 5 terminals, and run the docker image
in each of the terminals with this command.
```bash
docker-compose run --rm linux
```
If you are not on linux, replace "linux" with "wsl" or "mac" respectively.

In the first terminal run the following commands to start up gazebo with a vtol model.
```bash
cd px4_sitl/PX4-Autopilot
make px4_sitl gazebo_standard_vtol
```

In the second terminal start the ros master
```bash
roscore
```

In the third terminal run this to start/link mavros & px4
```bash
/opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14540"
```

In the last two terminals source the setup.bash script with
```bash
cd mission9-2021/software
. ./devel/setup.bash
```

Now everything should be setup!!

You may want to install vim as well so that you can view/edit code in the docker container
```bash
sudo apt install vim
```


## The "waypoint_publisher" node
There are two types of waypoints: Local waypoints (x, y, z relative to some point), and Global waypoints (lat, lon, alt). The waypoint_publisher can handle both
(albeit it's not very pretty) which may be useful for things like obstable avoidance. The way the publisher works is that it first makes some list of 
waypoints. For Global waypoints this can be done in the program QGroundControl where users can place waypoints on a map and get accurate GPS coordinates as shown
in the picture below. 
![plan_view_overview](https://user-images.githubusercontent.com/54146662/129457147-8ac9873c-1b89-40f0-9005-82e448c7f956.jpeg)

We can save the JSON of the mission plans from this program and parse it to create waypoint lists. This is implimented in the ```loadWaypointsFromFile()```
function. You can look at the sample JSON file in the ```missions/``` directory.

Once we have a list of waypoints, we can then call the ```runMission()``` function. Because the node is subscribed to the drone's current position, the function
constantly checks if the drone is at the ```curr_wp``` plus or minus a couple meters. It rapidly publishes this ```curr_wp``` (which can be thought of as
the next waypoint the drone has to go to) to either the ```next_local_waypoint``` or ```next_global_waypoint``` topic that we created. Later, the
"offboard_node" subscribes to this topic and sends the waypoints to the drone. If the drone gets close enough to the waypoint that is being published, 
the ```curr_wp``` is set to be the next waypoint in the list until the mission is complete.

## The "offboard_node" node
The offboard node is basically the 'driver' of the system. It has a few main tasks. Basically, it connects to the drone, arms & switches the mode to the autonomous 
"OFFBOARD" mode, takes off to a set height and hovers until its state is set to FIXED_WING, then proceeds to follow the waypoints provided by the 
waypoint_publisher node. 

### Notes
- Had a lot of trouble getting VTOL transition to work b/c of outdated documentation. The way it is written is the only way I tried that worked.
- In order to stay in OFFBOARD mode, waypoints need to be sent at a rate of at least 2/sec AND waypoints need to already be in the queue before switching 
to OFFBOARD. This is why you see 100 waypoints sent before even taking off
- There were multiple issues relating to the altitude setpoints. From what I gathered, subscribing to the drone's current GPS position through
the ```/mavros/global_position/global``` was not enough to get an accurate altitude measurement. I think you also have to subscribe to ```/mavros/altitude```
which is about 50m different for some reason.
- Currently can't land very well which isn't the best ig



## Running the sim
To run the sim yourself all you have to do is run these two commands in the two free terminals in the docker container.
```bash
rosrun navigation_test waypoint_publisher
```

```bash
rosrun navigation_test offboard_node
```

