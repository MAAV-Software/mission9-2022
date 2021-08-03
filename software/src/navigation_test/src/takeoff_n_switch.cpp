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
#include <mavros_msgs/CommandVtolTransition.h>


//Global Vars 
//ik it's a no no but oh well
bool g_is_home_gps_set = false;
sensor_msgs::NavSatFix home_gps;
mavros_msgs::State current_state;

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
  
  ros::Rate rate(20.0);


  ROS_INFO("Waiting for Home to be set...");
  while (ros::ok() && !g_is_home_gps_set)
  {
    ros::spinOnce();
    rate.sleep();
  }

  home_gps_sub.shutdown();

  
  //SET SOME SETPOINTS (so we can switch to OFFBOARD)
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 15;

  //send a few setpoints before starting
  for(int i = 100; ros::ok() && i > 0; --i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
  }

	
  arm_cmd.request.value = true;
  set_mode.request.custom_mode = "OFFBOARD";


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

  //ATTEMPT TO CHANGE TO VTOL MODE
  ros::ServiceClient arming_client_VTOL = nh.serviceClient<mavros_msgs::CommandVtolTransition>("/mavros/cmd/vtol_transition");
  mavros_msgs::CommandVtolTransition arm_cmd_vtol;
  arm_cmd_vtol.request.state = 4; //4 is for fized wing. 3 is for copter;
  while (current_state.armed && !arm_cmd_vtol.response.success && ros::ok()){
  	if (arming_client_VTOL.call(arm_cmd_vtol)){
		ROS_INFO("VTOL request --> State = %d", arm_cmd_vtol.request.state);
	}

	if (arm_cmd_vtol.response.success){
		ROS_INFO("In Fixed wing mode!");
		break;
	}else{
		ROS_INFO("Command Failed... Still in copter mode");
	}

	local_pos_pub.publish(pose);
	ros::spinOnce();
	rate.sleep();
  }

  pose.pose.position.x = 100;
  pose.pose.position.y = 100;
  pose.pose.position.z = 1.5;

  ROS_INFO("going to the next Location");
  for(int i = 0; ros::ok() && i < 10*20; ++i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }


 
    
  // Be in event loop
  while (ros::ok() && current_state.armed)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Mission accomplished!");

  return 0;
}
