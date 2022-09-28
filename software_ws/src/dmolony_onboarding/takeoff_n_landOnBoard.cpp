/**
 * Michigan Autonomous Aerial Vehicles 2022
 * file: takeoff_n_land.cpp
 * brief: simple ros node to control px4 with local waypoints
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>
#define FLIGHT_ALTITUDE 1.5f
using namespace std;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
geometry_msgs::PoseStamped current_pos;
void state_position(const geometry_msgs::PoseStamped msg){
    current_pos = msg;
}
struct coordinate{
    double x;
    double y;
    double z;
};
double distance(coordinate myTarget)
{
    return abs(myTarget.x - current_pos.pose.position.x)+abs(myTarget.y - current_pos.pose.position.y)+abs(myTarget.z - current_pos.pose.position.z);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
      ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber global_pos = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose",10, state_position);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT...");
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    ros::Time last_request = ros::Time::now();

    // change to offboard mode and arm
    while(ros::ok() && !current_state.armed){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
          ROS_INFO(current_state.mode.c_str());
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // go to the first waypoint
   
    vector<coordinate> myWayPoints;
    myWayPoints.push_back({0, 0, FLIGHT_ALTITUDE});
    myWayPoints.push_back({0, 2, FLIGHT_ALTITUDE});
    myWayPoints.push_back({2, 2, FLIGHT_ALTITUDE});
    myWayPoints.push_back({2, 0, FLIGHT_ALTITUDE});
    pose.pose.position.x = myWayPoints[0].x;
    pose.pose.position.y = myWayPoints[0].y;
    pose.pose.position.z = myWayPoints[0].z;

    ROS_INFO("going to the first way point");
    while(ros::ok() &&  distance(myWayPoints[0]) > .05){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("first way point finished!");


    //go to the second waypoint
    pose.pose.position.x = myWayPoints[1].x;
    pose.pose.position.y = myWayPoints[1].y;
    pose.pose.position.z = myWayPoints[1].z;

    //send setpoints for 10 seconds
    ROS_INFO("going to second way point");
    while(ros::ok() && distance(myWayPoints[1]) > .05){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("second way point finished!");

    // go to the third waypoint
    pose.pose.position.x = myWayPoints[2].x;
    pose.pose.position.y = myWayPoints[2].y;
    pose.pose.position.z = myWayPoints[2].z;
    //send setpoints for 10 seconds
    ROS_INFO("going to third way point");
    while(ros::ok() && distance(myWayPoints[2]) > .05){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("third way point finished!");
    
    // go to the forth waypoint
    pose.pose.position.x = myWayPoints[3].x;
    pose.pose.position.y = myWayPoints[3].y;
    pose.pose.position.z = myWayPoints[3].z;
    //send setpoints for 10 seconds
    ROS_INFO("going to forth way point");
    while(ros::ok() && distance(myWayPoints[3]) > .05){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("forth way point finished!");
    
    pose.pose.position.x = myWayPoints[0].x;
    pose.pose.position.y = myWayPoints[0].y;
    pose.pose.position.z = myWayPoints[0].z;
    ROS_INFO("going back to the first point!");
    //send setpoints for 10 seconds
    while(ros::ok() && distance(myWayPoints[0]) > .05){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }

    ROS_INFO("tring to land");
    while (!(land_client.call(land_cmd) &&
            land_cmd.response.success)){
      //local_pos_pub.publish(pose);
      ROS_INFO("tring to land");
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
//substribe to local position or smth i think
