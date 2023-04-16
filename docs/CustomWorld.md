# Setup Custom World Script in Gazebo

Alternate way to start gazebo that uses a custom world and uses a simulation camera

**Make sure that you are in SUDO mode on the following terminals**

### **Terminal 1:**
First, we will start the simulation with ROS wrappers by running the following. 

```
cd /mission9-2022/scripts
source start_gazebo.bash
roslaunch px4 posix_sitl.launch world:=/mission9-2022/mast.world
```




Happy Coding!
