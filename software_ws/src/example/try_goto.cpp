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
#include <cmath>
#include <geometry_msgs/TwistStamped.h>

#define FLIGHT_ALTITUDE 1.5f

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void current_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
    //ROS_INFO("current pose: [%f]", current_pose.pose.position.x);
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

    // subscribe to the current position ?
    ros::Subscriber current_pos = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, current_pose_cb);
    // publisher that just post the velocity ?
    ros::Publisher local_vel_body = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_velocity",10);

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


    // set some waypoints in vector, x, y, z
    std::vector<float> point_one = {1,1,FLIGHT_ALTITUDE};

    // simple test for take off and land
    // go to the first waypoint
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    //send setpoints for 10 seconds
    ROS_INFO("going to the first way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("first way point finished!");

    // try using vector to set the waypoint
    float dis_x = fabs(point_one[0] - current_pose.pose.position.x);
    float dis_y = fabs(point_one[1] - current_pose.pose.position.y);
    float dis_z = fabs(point_one[2] - current_pose.pose.position.z);

    // ROS_INFO("Go to First vector way point");
    // while (dis_x > 0.1 || dis_y > 0.1 || dis_z > 0.1) {
    //     ROS_INFO("Dis_x: [%f]", dis_x);
    //     pose.pose.position.x = point_one[0];
    //     pose.pose.position.y = point_one[1];
    //     pose.pose.position.z = point_one[2];      
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // ROS_INFO("First vector way point");

    // try give velocity for 5 second
    ROS_INFO("set velocity");
    geometry_msgs::TwistStamped vel;
    vel.twist.linear.x = 0.0;
    vel.twist.linear.y = 0.0;
    vel.twist.linear.z = 0.8;

    for(int i = 0; ros::ok() && i < 5*20; ++i){
      local_vel_body.publish(vel);
      ros::spinOnce();
      rate.sleep();
    }
    vel.twist.linear.x = 0;
    vel.twist.linear.y = 0;
    vel.twist.linear.z = 0;
    local_vel_body.publish(vel);

    ROS_INFO("finish velocity!");

    // going back
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    //send setpoints for 10 seconds
    ROS_INFO("going back");
    for(int i = 0; ros::ok() && i < 10*20; ++i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("finish going back");

    // land
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
