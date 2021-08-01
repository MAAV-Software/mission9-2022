#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

// Boost for JSON parsing
#include <boost/bind.hpp>
//#include <boost/cstdfloat.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <exception>
#include <iostream>
#include <set>
#include <string>

namespace bpt = boost::property_tree;

//Global Vars 
//ik it's a no no but oh well
bool g_is_home_gps_set = false;
sensor_msgs::NavSatFix home_gps;
mavros_msgs::State current_state;

void getWaypointsFromQGCPlan(const std::string& qgc_plan_file, mavros_msgs::WaypointPush* wp_list)
{
    std::ifstream file(qgc_plan_file);
    std::stringstream ss;

    ss << file.rdbuf();
    file.close();

    bpt::ptree mission_pt;
    bpt::read_json(ss, mission_pt);

    bool first = true;
    
    for (auto& mi : mission_pt.get_child("mission.items"))
    {
      ROS_INFO("Doing waypoint...");
      mavros_msgs::Waypoint wp{};
      
      wp.frame = mi.second.get<int>("frame");
      wp.command = mi.second.get<int>("command");
      wp.autocontinue = mi.second.get<bool>("autoContinue");
      wp.is_current = first ? true : false;
      first = false;
     
      // Parameters
      std::vector<double> params;
      for (auto& p : mi.second.get_child("params")){
	try{
	    params.push_back(p.second.get<double>(""));
	}catch(...){ // Throws if there is a null in a param so I just changed it to 0
	    params.push_back(0);
	}
      }

      wp.param1 = params[0];
      wp.param2 = params[1];
      wp.param3 = params[2];
      wp.param4 = params[3]; // This one is sometimes null for some reason. See line 58 
      wp.x_lat = params[4];
      wp.y_long = params[5];
      wp.z_alt = params[6];
      wp_list->request.waypoints.push_back(wp);
    }
}

void set_state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

//Global home pos in lat long --> Mainly for debugging
void set_home_cb(const sensor_msgs::NavSatFixConstPtr& msg){
  home_gps = *msg;
  g_is_home_gps_set = true;
  ROS_INFO("Received Home: %lf, %lf, %lf", home_gps.latitude, home_gps.longitude, home_gps.altitude);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aero_flymission");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, set_state_cb);
  ros::Subscriber home_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, set_home_cb);


  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
 
  mavros_msgs::CommandBool arm_cmd;  
  mavros_msgs::SetMode set_mode;
  

  //Populate the waypoints
  mavros_msgs::WaypointPush wp_list{};
  getWaypointsFromQGCPlan("/mission9-2021/software/src/navigation_test/first_mission.plan", &wp_list);

  ros::Rate rate(20.0);

  ROS_INFO("Waiting for Home to be set...");
  while (ros::ok() && !g_is_home_gps_set)
  {
    ros::spinOnce();
    rate.sleep();
  }

  home_gps_sub.shutdown();

  // Send WPs to Vehicle (This part appears to work)
  ros::ServiceClient wp_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
  ROS_INFO("Now, Sending WPs to Vehicle...");
  while (ros::ok())
  {
    if (wp_client.call(wp_list))
    {
      if (!wp_list.response.success)
      {
        // Wait until success
        ros::spinOnce();
        rate.sleep();
      }
      else
      {
        ROS_INFO("WPs sent to Vehicle");
        break;
      }
    }
  }


  //SET SOME SETPOINTS (so we can switch to OFFBOARD)
  geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.5;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

	
  arm_cmd.request.value = true;
  set_mode.request.custom_mode = "OFFBOARD";

  //Below I tried to get the takeoff command to work. Unsuccessfully ;( lol
  /*
  mavros_msgs::CommandTOL takeoff_cmd;
  takeoff_cmd.request.yaw = 0;
  takeoff_cmd.request.latitude = 0;
  takeoff_cmd.request.latitude = 0;
  takeoff_cmd.request.altitude = 5;

  while (!(arming_client.call(arm_cmd) && arm_cmd.response.success)){
         ROS_INFO("The drone is armed...");
 }

  while (!(takeoff_client.call(takeoff_cmd) && takeoff_cmd.response.success)){
  	ROS_INFO("TRYING TO TAKEOFF");
	ros::spinOnce();
	rate.sleep();
  }

  while (ros::ok()){
  	ros::spinOnce();
        rate.sleep();
  }
  */

  ros::Time last_request = ros::Time::now();

  //TRY TO TAKE OFF To Height First in Offboard mode
  while (ros::ok() && !current_state.armed){
  	if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
	    ROS_INFO("Currently in %s", current_state.mode.c_str());
	    if (set_mode_client.call(set_mode) && set_mode.response.mode_sent){
	    	ROS_INFO("OFFBOARD IS ENABLED");
	    }
	    last_request = ros::Time::now();
	}else{
	    if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
	    	if (arming_client.call(arm_cmd) && arm_cmd.response.success){
		    ROS_INFO("The drone is armed...");
		}
		last_request = ros::Time::now();
	    }
	}
	local_pos_pub.publish(pose);
	ros::spinOnce();
	rate.sleep();
  }

  ROS_INFO("going to the Takeoff Location");
  for(int i = 0; ros::ok() && i < 10*20; ++i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }
    

  //Attempt to Set to Mission mode
  ROS_INFO("Now, Setting to Mission mode...");
  set_mode.request.custom_mode = "AUTO.MISSION";


  while (ros::ok() && current_state.mode != "AUTO.MISSION"){
      if (current_state.mode != "AUTO.MISSION" && (ros::Time::now() - last_request > ros::Duration(5.0))){
          ROS_INFO("Currently in %s", current_state.mode.c_str());
          if (set_mode_client.call(set_mode) && set_mode.response.mode_sent){
              ROS_INFO("Request to change to mission mode sent");
          }
          last_request = ros::Time::now();
      }else{
          if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
              if (arming_client.call(arm_cmd) && arm_cmd.response.success){
                  ROS_INFO("The drone is armed...");
              }
              last_request = ros::Time::now();
          }
      }
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
 }



  ROS_INFO("5 Second Break");
  sleep(5.0);
  ROS_INFO("Resumed. Missions in execution...");
  bool armed = static_cast<int>(current_state.armed);

  // Be in event loop
  while (ros::ok() && current_state.armed)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Mission accomplished!");

  return 0;
}
