#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>int main(int argc, char **argv)
{
    ros::init(argc, argv, "takeoff_and_hover_node");
    ros::NodeHandle nh;    // Create service clients for arming, taking off, and setting mode
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");    // Wait for MAVROS services to become available
    ros::Duration(5).sleep();    // Arm the drone
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (!arming_client.call(arm_cmd) || !arm_cmd.response.success) {
        ROS_ERROR("Failed to arm the drone.");
        return 1;
    }
    ROS_INFO("Drone armed.");    // Take off to a specified altitude
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = 2.0; // Set the desired takeoff altitude in meters
    if (!takeoff_client.call(takeoff_cmd) || !takeoff_cmd.response.success) {
        ROS_ERROR("Failed to take off.");
        return 1;
    }
    ROS_INFO("Drone taking off to %.2f meters.", takeoff_cmd.request.altitude);    // Hover for a specified time (e.g., 30 seconds)
    ros::Duration(30).sleep();    // Land the drone
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.altitude = 0; // Land at ground level
    if (!takeoff_client.call(land_cmd) || !land_cmd.response.success) {
        ROS_ERROR("Failed to land the drone.");
        return 1;
    }
    ROS_INFO("Drone landing.");    // Disarm the drone
    arm_cmd.request.value = false;
    if (!arming_client.call(arm_cmd) || !arm_cmd.response.success) {
        ROS_ERROR("Failed to disarm the drone.");
        return 1;
    }
    ROS_INFO("Drone disarmed.");    return 0;
}
