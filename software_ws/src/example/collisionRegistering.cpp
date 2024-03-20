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
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SpawnModel.h>
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
// #include <gazebo/gazebo.hh>
// #include <rendering/Visual.hh>
// #include <common/common.hh>
// #include <Vector3.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace tf2;
gazebo_msgs::ContactsState current_contact;
void state_cb(const gazebo_msgs::ContactsState::ConstPtr& msg){
    current_contact = *msg;
//     ROS_INFO("Sonar Seq: [%s]", );
}
void remove(std::string model_name)
{
     ros::NodeHandle other;
     ros::service::waitForService("gazebo/delete_model");
     ros::ServiceClient deleteModelClient = other.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
     gazebo_msgs::DeleteModel deleteModel;
     deleteModel.request.model_name = model_name;
     deleteModelClient.call(deleteModel);
}
// void spawn(std::string model_name)
// {
//      ros::NodeHandle other1;
//      ros::service::waitForService("gazebo/spawn_model");
//      ros::ServiceClient spawnClient = other1.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_model");
//      gazebo_msgs::SpawnModel spawnModel;
//      spawnModel.request.model_name = model_name;
//      spawnModel.request.model_xml = "waypoints.world";
//      spawnClient.call(spawnModel);
// }

int main(int argc, char **argv)
{
     ros::init(argc, argv, "collision_registration");
     ros::NodeHandle nh;
     ros::Subscriber state_sub = nh.subscribe<gazebo_msgs::ContactsState>("/waypoint_contact_senor", 1, state_cb);
     ros::Rate rate(20.0);

     // reading in number of waypoints from way_points.txt
     // ifstream wpFile("way_points.txt");
     // string junk = "";
     // int numWaypoints=  0;
     // getline(wpFile, junk);
     // wpFile >> junk >> junk >> numWaypoints;
     // vector<bool> wpReached(numWaypoints, false);
     

     // ros::ServiceClient client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
     // // /gazebo/default/iris/base_link/waypoint_contact/contacts
     // ofstream myfile;
     // myfile.open ("contacts.txt");
     // myfile << "Writing this to a file.\n";
     while(true)
     {
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
          auto myIterator = current_contact.states.begin();
          
          while(myIterator != current_contact.states.end())
          {
               // ROS_INFO("Hi %s", (*myIterator).collision2_name.c_str());
               auto temp = (*myIterator).collision2_name;
               // myIterator++;
               // get from the start of collision two name to the first :
               size_t colonPos = temp.find(':');
               if (colonPos != std::string::npos) {
                    // Extract the first part using substr
                    std::string firstPart = temp.substr(0, colonPos);
                    // ROS_INFO("Hi %s", firstPart.c_str());
                    std::string compare = "asphalt_plane";
                    // ROS_INFO("Hi %s", compare.c_str());
                    // ROS_INFO("Hi %d", compare == firstPart);
                    if(firstPart != compare)
                    {
                         remove(firstPart);
                         // spawn(firstPart);
                    }
                    
                    // Print or use the first part
                    // std::cout << "First part: " << firstPart << std::endl;
               }
               myIterator++; 
          }
          // ros::spinOnce();
          
          // ROS_INFO("Hi %s", current_contact.collision2_name.c_str());
          // myfile << current_contact.collision2_name;
          ros::spinOnce();
          rate.sleep();
     }
     // myfile.close();
     return 0;
}