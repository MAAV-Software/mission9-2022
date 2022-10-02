# Setup Simulation Camera in Gazebo

By the end of this tutorial, you should have a simluated realsense camera in the gazebo simulation that publishes image data, depth data, and point-cloud data. This is currently a work in progress so don't get salty and ask Drew is you have questions. 

## Modify the quadcopter sdf file

Please make sure that a .sdf has been generated for the drone that you are simulating. By default the drone that Gazebo is simulating is called the `iris drone`. You can search the whole system for sdf files (robot description xml similar to `urdf`) using the following command. See [here](https://newscrewdriver.com/2018/07/31/ros-notes-urdf-vs-gazebo-sdf/#:~:text=My%20understanding%20can%20be%20boiled,can%20be%20represented%20in%20URDF.) for a description of the differences between sdf and urdf.

```
$ find / -name "*.sdf"          # all sdf files
$ find / -name "iris.sdf"       # for iris drone specifically
```
The iris file should be in /px4_sitl/PX4-Autopilot/Tools/sitl_gazebo/models/iris/iris.sdf

Edit the .sdf file using the code editor of your choice (e.g. vim, nano, etc).

```
$ vim iris.sdf
```

We are going to add our own plugin to the robot model! At the bottom of the file, add the following code between the last `</plugin>` tag and the `</model>` tag.

<ins>New Plugin XML:</ins>
```
  <!--depth camera -->
  <include>
    <uri>model://depth_camera</uri>
    <pose>0.15 0 0.00 0 0 0</pose>
  </include>
  <joint name="depth_camera_joint" type="fixed">
    <child>depth_camera::link</child>
    <parent>iris::base_link</parent>
    <axis>
      <xyz>0 0 1</xyz>
      <limit>
        <upper>0</upper>
        <lower>0</lower>
      </limit>
    </axis>
  </joint>
```

Once you add to and save the file you can run the simulator again and look at the new camera model that was added to the front of the drone:
```
$ cd /px4_sitl/PX4-Autopilot
$ make px4_sitl gazebo
```


## Visualize the camera output and link with mavros

In order to view the output of the camera, we have to start the simulation in a lightly differnt way. Close out of all your current terminals follow the steps below.

**NOTE:** make sure that you are root by running:
```
$ sudo -i
```

### **Terminal 1:**
First, we will start the simulation with ROS wrappers by running the following. 

```
$ cd /px4_sitl/PX4-Autopilot
$ DONT_RUN=1 make px4_sitl_default gazebo
$ source /mission9-2022/software_ws/devel/setup.bash
$ source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
$ roslaunch px4 posix_sitl.launch
```

I have also included a script named [gazebo_ros_wrap.sh](../scripts/install_everything.sh) in `/mission9-2022/scripts`

### **Terminal 2:**
run the rvis node in a new terminal:
```
$ . /mission9-2022/software_ws/devel/setup.bash
$ rosrun rviz rviz
```

### **Terminal 3:**
Now make sure that you justify the camera frame with the following command
If you have an error, this command will likely fix it.
**NOTE** This is likely not actually correct but it's okay for the time being.
```
$ rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map [my_frame] 100
```



Happy Coding!
