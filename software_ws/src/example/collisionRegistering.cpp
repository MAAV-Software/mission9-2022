/**
 * Michigan Autonomous Aerial Vehicles 2023
 * file: mast_mover.cpp
 * brief: ______
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ContactsState.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tuple>
#include <cstdlib>
#include <math.h>
#include <cmath>
#include <string>
#include <cstring> 
#include <cassert>
#include <algorithm> 
#include <tf/tf.h>
// #include <Vector3.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace tf2;
gazebo_msgs::ContactState current_contact;
void state_cb(const gazebo_msgs::ContactState& msg){
    current_contact = msg;
    ROS_INFO("Sonar Seq: [%s]", msg.collision2_name.c_str());
}


int main(int argc, char **argv)
{
     ros::init(argc, argv, "collision_registration");
     ros::NodeHandle nh;
     ros::Subscriber state_sub = nh.subscribe("waypoint_contact_senor", 10, state_cb);
     // ros::Rate rate(20.0);
     // // /gazebo/default/iris/base_link/waypoint_contact/contacts
     // ofstream myfile;
     // myfile.open ("contacts.txt");
     // myfile << "Writing this to a file.\n";
     // while(true)
     // {
     //      // gazebo_msgs::ContactState myContact;
     //      // current_state.
     //      string s = current_contact.collision2_name;
     //      if(s.size() > 1)
     //      {
     //           ROS_INFO("HI");
     //      }
     //      if(s.c_str() == "asphalt_plane::link::collision")
     //      {
     //           ROS_INFO("HI!");
     //      }
     //      if(s.c_str() == "\"asphalt_plane::link::collision\"")
     //      {
     //           ROS_INFO("HI?");
     //      }
     //      if(s[0] == 'a')
     //      {
     //           ROS_INFO("JI:");
     //      }
     //      if(s[0] == '"')
     //      {
     //           ROS_INFO("JI!");
     //      }
          // string temp = current_contact.collision2_name;
          // ROS_INFO(temp);
          // ROS_DEBUG("Hi %s", temp.c_str());
          // current_contact.collision2_name
          // cout << current_contact.collision2_name;
          // ROS_INFO("Hi %s", temp.c_str());
          // myfile << current_contact.collision2_name;
          ros::spin();
          // rate.sleep();
     // }
     // myfile.close();
     return 0;
}