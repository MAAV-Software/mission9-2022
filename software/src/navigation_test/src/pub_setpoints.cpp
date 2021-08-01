#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/CommandTOL.h"

struct Waypoint{
	double x, y, z;
};

std::vector<Waypoint> waypoints;
nav_msgs::Odometry current_pose_msg;

bool atWaypoint(Waypoint& wp){
	double max_error = 0.05; // This is in meters
	
	double diff_x = abs(wp.x - current_pose_msg.pose.pose.position.x);
	double diff_y = abs(wp.y - current_pose_msg.pose.pose.position.y);
	double diff_z = abs(wp.z - current_pose_msg.pose.pose.position.z);

	double dist = sqrt(diff_x*diff_x + diff_y*diff_y + diff_z* diff_z);

	if (dist < max_error){
		return true;
	}else{
		return false;
	}

}

void pose_cb(const nav_msgs::Odometry::ConstPtr& msg){
	current_pose_msg = *msg;
}

/*class Navigator {
	public:
		Navigator() {
		    InitWaypoints();
		}

		bool atWaypoint(){
        		Waypoint wp = _waypoints[current_waypoint_ind];
			
			double max_error = 0.05; // This is in meters

        		double diff_x = abs(wp.x - current_pose_msg.pose.pose.position.x);
        		double diff_y = abs(wp.y - current_pose_msg.pose.pose.position.y);
        		double diff_z = abs(wp.z - current_pose_msg.pose.pose.position.z);

        		double dist = sqrt(diff_x*diff_x + diff_y*diff_y + diff_z* diff_z);

        		if (dist < max_error){
				++curr_waypoint_ind;
                		return true;
        		}else{
                		return false;
        		}

		}
	private:
		void InitWaypoints() {
			_waypoints.push_back({0, 0, 2});
			_waypoints.push_back({0, 10, 2});
			_waypoints.push_back({10, 10, 2});
			
			curr_waypoint_ind = 0;
		}

		void pose_callback(nav_msgs::Odometry::ConstPtr& msg){
			curr_pose = *msg;
		}



		nav_msgs::Odometry curr_pose;
		vector<Waypoint> _waypoints;
		int curr_waypoint_ind;

};*/

int main(int argc, char** argv){
	ros::init(argc, argv, "waypoint_publisher");
	ros::NodeHandle nh;

	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);
	ros::Subscriber current_pos = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 20, pose_cb);
	//ros::ServiceClient take_off_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
	ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");


	ros::Rate loop_rate(20.0);
	ros::spinOnce();

	geometry_msgs::PoseStamped msg;
	
	//Takeoff
	
	

	waypoints = {{0, 0, 2}, {0, 10, 2}, {10, 10, 2}};
	waypoints.push_back(waypoints[0]);
	Waypoint current_waypoint = waypoints[0];
	int i = 0;
	int c= 0;
	while (i < waypoints.size()){
		if (atWaypoint(current_waypoint)){
			ROS_INFO("REACHED waypoint #[%d]", i);
			current_waypoint = waypoints[++i];
		}
		msg.header.seq = c;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = 1;

		msg.pose.position.x = current_waypoint.x;
		msg.pose.position.y = current_waypoint.y;
		msg.pose.position.z = current_waypoint.z;
	
		pose_pub.publish(msg);

		ros::spinOnce();
		++c;
		loop_rate.sleep();

	}

	ROS_INFO("FINISHED!!!");


	/*while (ros::ok()){
		msg.header.seq = c;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = 1;

		msg.pose.position.x = 0.0;
		msg.pose.position.y = 0.0;
		msg.pose.position.z = 1.5;

		msg.pose.orientation.x = 0;
		msg.pose.orientation.y = 0;
		msg.pose.orientation.z = 2;
		msg.pose.orientation.w = 1;
	
		pose_pub.publish(msg);
		ros::spinOnce();
		++c;
		loop_rate.sleep();
	}*/

	return 0;

}
