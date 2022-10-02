/**
 * Michigan Autonomous Aerial Vehicles 2022
 * file: waypoint_following.cpp
 * brief: simple ros node to control px4 with local waypoints
 */

#include <string>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define FLIGHT_ALTITUDE 1.5f

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = *msg;
}

bool within_range(geometry_msgs::PoseStamped &current, geometry_msgs::PoseStamped &target, float range){
    return (fabs(current.pose.position.x - target.pose.position.x) < range) && (fabs(current.pose.position.y - target.pose.position.y) < range) && (fabs(current.pose.position.z - FLIGHT_ALTITUDE) < range);
}

void go_to_waypoint(ros::Rate &rate, ros::Publisher &pos_pub, geometry_msgs::PoseStamped &pose){
    while (!within_range(current_pos, pose, 0.1f)) {
        pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT");
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
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
                ROS_INFO("offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    std::vector<std::vector<float>> coords{
      {0, 0},
      {0, 4},
      {5, 7},
      {3, 9},
      {-1, -9},
      {-4, 9},
      {5, -3},
      {0, 0}
    };

    geometry_msgs::PoseStamped pose;

    for (int i = 0; i < coords.size(); i++) {
      pose.pose.position.x = coords[i][0];
      pose.pose.position.y = coords[i][1];
      pose.pose.position.z = FLIGHT_ALTITUDE;

      std::string wp = std::to_string(i + 1);
      std::string s = "going to waypoint " + wp;
      ROS_INFO(s.c_str());
      go_to_waypoint(rate, local_pos_pub, pose);
      s = "reached waypoint " + wp;
      ROS_INFO(s.c_str());
    }

    ROS_INFO("trying to land");
    while (!(land_client.call(land_cmd) &&
            land_cmd.response.success)){
      ROS_INFO("trying to land");
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
