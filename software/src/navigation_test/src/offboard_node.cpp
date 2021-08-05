// offboard_node.cpp --> the main driver

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/CommandVtolTransition.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/NavSatFix.h>

#define FLYING_ALTITUDE 2.0f


mavros_msgs::State current_state;
nav_msgs::Odometry current_pose;
geometry_msgs::PoseStamped current_local_waypoint;
geographic_msgs::GeoPoseStamped current_global_waypoint;


//Initial Position Callbacks
double latitude, longitude;
bool gps_recieved = false;
void get_global_pose_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
	latitude = msg->latitude;
	longitude = msg->longitude;
	gps_recieved = true;
}

double altitude;
bool alt_recieved = false;
void get_altitude_cb(const mavros_msgs::Altitude::ConstPtr& msg){
	altitude = msg->amsl;
	alt_recieved = true;
}

//Waypoint CallBacks
void global_waypt_cb(const geographic_msgs::GeoPoseStamped::ConstPtr& msg){
	current_global_waypoint = *msg;
}

void local_waypt_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	current_local_waypoint = *msg;
}

//State callback
void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

int main (int argc, char** argv){
	ros::init(argc, argv, "offboard_node");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 20, state_cb);
	ros::Subscriber next_loc_wp_sub = nh.subscribe("next_local_waypoint", 100, local_waypt_cb);
	ros::Subscriber next_global_wp_sub = nh.subscribe("next_global_waypoint", 100, global_waypt_cb);
	ros::Subscriber sub_global_pose = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, get_global_pose_cb);
	ros::Subscriber sub_altitude = nh.subscribe<mavros_msgs::Altitude>("mavros/altitude", 10, get_altitude_cb);

	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::Publisher global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);
		
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
	
	ros::Rate rate(20.0);

	// wait for FCU connection
	while(ros::ok() && current_state.connected){
        	ros::spinOnce();
        	rate.sleep();
        	ROS_INFO("connecting to FCT...");
   	}

	ROS_INFO("Waiting for Home to be set...");
	while (ros::ok() && (!gps_recieved || !alt_recieved)){
    		ros::spinOnce();
    		rate.sleep();
  	}


	//Set Home Position
	const double home_lat = latitude;
	const double home_lon = longitude;
	const double home_alt = altitude;
	ROS_INFO("GPS Home Recieved --> lat: %f, lon: %f, alt: %f", home_lat, home_lon, home_alt);

    	geometry_msgs::PoseStamped pose;
    	pose.pose.position.x = 0;
    	pose.pose.position.y = 0;
    	pose.pose.position.z = FLYING_ALTITUDE;

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
          		ROS_INFO("%s", current_state.mode.c_str());
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
	pose.pose.position.x = 0;
    	pose.pose.position.y = 0;
    	pose.pose.position.z = FLYING_ALTITUDE;

    	ROS_INFO("going to the first way point");
    	for(int i = 0; ros::ok() && i < 2*10*20; ++i){
    	  local_pos_pub.publish(pose);
    	  ros::spinOnce();
    	  rate.sleep();
	}
    	ROS_INFO("Reached Takeoff Altitude!");


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

	//Pull the next waypoints from topic
	while (ros::ok()){
    	  	//local_pos_pub.publish(current_local_waypoint);
		

		geographic_msgs::GeoPoseStamped wp = current_global_waypoint;
		wp.pose.position.altitude += home_alt;
		
		global_pos_pub.publish(wp);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
