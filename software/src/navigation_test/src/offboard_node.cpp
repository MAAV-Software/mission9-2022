// offboard_node.cpp --> the main driver

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/Waypoint.h>



#define FLYING_ALTITUDE 1.5f

/*
class FlightController{
	public:
		void setTakeoff(){
			takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
				
		}
	
	private:
		ros::ServiceClient takeoff_client;
		mavros_msgs::State current_state_g;
		nav_msgs::Odometry current_pose_g;
		geometry_msgs::Pose correction_vector_g;
		geometry_msgs::Point local_offset_pose_g;
		geometry_msgs::PoseStamped waypoint_g;

};*/


mavros_msgs::State current_state;
nav_msgs::Odometry current_pose;
geometry_msgs::PoseStamped waypoint;



void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

int main (int argc, char** argv){
	ros::init(argc, argv, "offboard_node");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 20, state_cb);
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	ros::ServiceClient wp_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
	ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
	
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

	//request the takeoff
	mavros_msgs::CommandTOL srv_takeoff;
	srv_takeoff.request.altitude = 1.5;
	if(takeoff_client.call(srv_takeoff)){
		sleep(3);
		ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
	}else{
		ROS_ERROR("Failed Takeoff");
		return -2;
	}
	sleep(2);

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
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLYING_ALTITUDE;

    ROS_INFO("going to the first way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("first way point finished!");



	mavros_msgs::WaypointPush wp_push_srv;
	mavros_msgs::Waypoint wp;


	wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    	wp.command        = mavros_msgs::CommandCode::NAV_TAKEOFF;
    	wp.is_current     = true;
    	wp.autocontinue   = true;
    	wp.x_lat          = 47.3978206;
    	wp.y_long         = 8.543987;
    	wp.z_alt          = 10;
    	wp_push_srv.request.waypoints.push_back(wp);
    
    	// WP 1
    	wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    	wp.command        = mavros_msgs::CommandCode::NAV_LOITER_TIME;
    	wp.is_current     = false;
    	wp.autocontinue   = true;
    	wp.x_lat          = 47.3962527;
    	wp.y_long         = 8.5467917;
    	wp.z_alt          = 20;
		wp.param1			= 10;
		wp.param3			= 2;
		wp.param4			= 1;
    	wp_push_srv.request.waypoints.push_back(wp);
    
    	// WP 2
    	wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    	wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
    	wp.is_current     = false;
    	wp.autocontinue   = true;
    	wp.x_lat          = 47.3977783;
    	wp.y_long         = 8.547906;
    	wp.z_alt          = 20;
    	wp_push_srv.request.waypoints.push_back(wp);

    	// WP 3
    	wp.frame          = mavros_msgs::Waypoint::FRAME_MISSION;
    	wp.command        = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
    	wp.is_current     = false;
    	wp.autocontinue   = true;
    	wp.x_lat          = 0;
    	wp.y_long         = 0;
    	wp.z_alt          = 0;
    	wp_push_srv.request.waypoints.push_back(wp);

	

	//mavros_msgs::SetMode offb_set_mode;
    	offb_set_mode.request.custom_mode = "AUTO.MISSION";

    	//mavros_msgs::CommandBool arm_cmd;
    	//arm_cmd.request.value = true;

	//ros::Time last_request = ros::Time::now();

	//Arm the drone
	/*while(ros::ok() && !current_state.armed){
        	if( current_state.mode != "AUTO.MISSION" && (ros::Time::now() - last_request > ros::Duration(5.0))){
          		ROS_INFO(current_state.mode.c_str());
            		if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                		ROS_INFO("AUTO.MISSION enabled");
            		}
            		last_request = ros::Time::now();
        	} else{
			if ( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
				if (arming_client.call(arm_cmd) && arm_cmd.response.success){
					ROS_INFO("VEHICLE IS ARMED");
				}
				last_request = ros::Time::now();
			}
		}
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}
	
	//Sending the waypoints
	if (wp_client.call(wp_push_srv)){
		ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
	}else{
		ROS_ERROR("Sending waypoints failed");
	}*/


	// ARM
    	//mavros_msgs::CommandBool arm_cmd;
    	//arm_cmd.request.value = true;
    	//if (arming_client.call(arm_cmd) &&
    	//    arm_cmd.response.success){
    	//    ROS_INFO("Vehicle armed");
    	//}

    	// Send WPs to Vehicle
    	if (wp_client.call(wp_push_srv)) {
    	    ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
    	    if (current_state.mode != "AUTO.MISSION") {
    	        if( set_mode_client.call(offb_set_mode)){
    	            ROS_INFO("AUTO.MISSION enabled");
    	        }
    	    }
    	}
	
	int c = 0;
	while (ros::ok()){
		//Do Something here
		//ROS_INFO("Doing mighty fine... [%d]", c);
		ros::spinOnce();
		rate.sleep();
		//++c;
	}

	return 0;


}
