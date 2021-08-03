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

class Navigator {
	public:
		Navigator(ros::NodeHandle& nh)  {
			pose_pub = nh.advertise<geometry_msgs::PoseStamped>("next_waypoint", 100);
		}

		void pose_callback(const nav_msgs::Odometry::ConstPtr& msg){
			curr_pose = *msg;
		}

		void loadWaypoints(std::vector<Waypoint>& wps){
			this->_waypoints = wps;
			this->curr_waypoint_ind = 0;
		}

		void runMission(){			
			
			ros::Rate loop_rate(20.0);
			geometry_msgs::PoseStamped msg;

        		Waypoint curr_wp = _waypoints[curr_waypoint_ind];
			
			int c= 0;
			while (curr_waypoint_ind < _waypoints.size()){
				if (this->atWaypoint()){
					ROS_INFO("REACHED waypoint #[%d]", curr_waypoint_ind);
					curr_wp = _waypoints[++curr_waypoint_ind];
				}
				msg.header.seq = c;
				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = 1;

				msg.pose.position.x = curr_wp.x;
				msg.pose.position.y = curr_wp.y;
				msg.pose.position.z = curr_wp.z;
			
				pose_pub.publish(msg);

				ros::spinOnce();
				loop_rate.sleep();
				++c;
			}
		}

		
	private:
		bool atWaypoint(){
        		Waypoint wp = _waypoints[curr_waypoint_ind];
			
			double max_error = 1.0; // This is in meters

        		double diff_x = abs(wp.x - curr_pose.pose.pose.position.x);
        		double diff_y = abs(wp.y - curr_pose.pose.pose.position.y);
        		double diff_z = abs(wp.z - curr_pose.pose.pose.position.z);

        		double dist = sqrt(diff_x*diff_x + diff_y*diff_y + diff_z* diff_z);

        		if (dist < max_error){
                		return true;
        		}else{
                		return false;
        		}
		}
		

		ros::Publisher pose_pub;
	
		nav_msgs::Odometry curr_pose;
		std::vector<Waypoint> _waypoints;
		int curr_waypoint_ind;
		
};

int main(int argc, char** argv){
	ros::init(argc, argv, "waypoint_publisher");
	ros::NodeHandle nh;

	


	Navigator navigator(nh);
	ros::Subscriber current_pos = nh.subscribe("/mavros/global_position/local", 20, &Navigator::pose_callback, &navigator);
	
	//ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
 
	//Load the waypoints;	
	ROS_INFO("LOADING WAYPOINTS...");
	std::vector<Waypoint> waypoints = {{100, 100, 20}, {0, 100, 20}, {200, 0, 20}};
	waypoints.push_back(waypoints[0]);
	navigator.loadWaypoints(waypoints);
	ROS_INFO("LOADED WAYPOINTS...");


	//Run the waypoint mission
	ROS_INFO("MISSION IN PROGRESS...");
	navigator.runMission();	



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
