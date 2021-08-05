#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/CommandTOL.h"
#include "sensor_msgs/NavSatFix.h"
#include "mavros_msgs/Altitude.h"
#include <math.h>


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


struct Waypoint{ // x, y, z or lat, long, alt depending on what you put in
	double x, y, z;
};

class Navigator {
    public:
	Navigator(ros::NodeHandle& nh)  {
	    local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("next_local_waypoint", 100);
	    global_pose_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("next_global_waypoint", 100);	
	}

	void setGroundAlt(){
	    ros::Rate rate(20.0);

            while (ros::ok() && !got_alt){
	    	ROS_INFO("Wassup my guy");
		ros::spinOnce();
		rate.sleep();
	    }


	    this->grounded_alt = curr_alt;
	    ROS_INFO("Grounded alt = %f", this->grounded_alt);
	}
	
	void local_pose_callback(const nav_msgs::Odometry::ConstPtr& msg){
	    curr_local_pose = *msg;
	}


	void global_pose_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
	    curr_global_pose = *msg;
	}

	void altitude_callback(const mavros_msgs::Altitude::ConstPtr& msg){
	    this->curr_alt = msg->amsl;
	    this->got_alt = true;
	}

	//Populates local or global waypoints
	void loadWaypoints(std::vector<Waypoint>& wps, bool isGPS = false){
	    this->_waypoints = wps;
	    this->curr_waypoint_ind = 0;
	    this->isGPS = isGPS;
	}

	//Populates waypoints with lat/long/alt coords from a mission.plan file from QGC
	void loadWaypointsFromFile(const std::string filename){
	    this->isGPS = true;
	    this->curr_waypoint_ind = 0;
	    this->_waypoints = {};

	    std::ifstream file(filename);
	    std::stringstream ss;

	    ss << file.rdbuf();
	    file.close();

	    bpt::ptree mission_pt;
	    bpt::read_json(ss, mission_pt);

	    for (auto& mi : mission_pt.get_child("mission.items")){
		ROS_INFO("Reading Waypoint...");
		Waypoint wp;

		// Parameters
		std::vector<double> params;		
		for (auto& p : mi.second.get_child("params")){
		    try{
			params.push_back(p.second.get<double>(""));
		    }catch(...){ // Throws if there is a null in a param so I just changed it to 0
			params.push_back(0);
		    }
		}

		wp.x = params[4]; // Lat
		wp.y = params[5]; // Long
		wp.z = params[6]; // Alt
		this->_waypoints.push_back(wp);
    		}
	}

	//Loop that constantly pushes the next waypoint in the mission to 
	//a ros topic (either GPS or local)
	void runMission(){			
	    ros::Rate loop_rate(20.0);

	    Waypoint curr_wp = _waypoints[curr_waypoint_ind];

	    int c= 0;
	    while (curr_waypoint_ind < _waypoints.size()){
		if (this->atWaypoint()){
		    ROS_INFO("REACHED waypoint #[%d]", curr_waypoint_ind);
		    printCurrentGPSPosition();
		    curr_wp = _waypoints[++curr_waypoint_ind];
		}

		pushWaypoint(curr_wp, c);

		ros::spinOnce();
		loop_rate.sleep();
		++c;
	    }
	}


    private:
	void printCurrentGPSPosition(){
		double lat = curr_global_pose.latitude;
		double lon = curr_global_pose.longitude;
		double alt = this->curr_alt;
		ROS_INFO("Currently at position lat: %f, lon: %f, alt: %f", lat, lon, alt);
	}	
	//Actually pushes the message to correct topic
	void pushWaypoint(Waypoint& curr_wp, int c){
		if (!isGPS){ // if it is local waypoint

	    		geometry_msgs::PoseStamped msg;

			msg.header.seq = c;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = 1;

			msg.pose.position.x = curr_wp.x;
			msg.pose.position.y = curr_wp.y;
			msg.pose.position.z = curr_wp.z;

			local_pose_pub.publish(msg);
		}else{
		
	    		geographic_msgs::GeoPoseStamped msg;

			msg.header.seq = c;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = 1;

			msg.pose.position.latitude = curr_wp.x; //Lat
			msg.pose.position.longitude = curr_wp.y; //Lon
			msg.pose.position.altitude = curr_wp.z; //Alt

			global_pose_pub.publish(msg);
		}
	}


	bool atWaypoint(){
		if (isGPS){
			return atWaypointGlobal();
		}else{
			return atWaypointLocal();
		}
	}

	//Great Circle dist using Haversine formula then pythag
	bool atWaypointGlobal(){
		Waypoint wp = _waypoints[curr_waypoint_ind];
		
		double max_error = 3; //Maybe this should change depending on how sharp of a turn it has to do? idk
		
		//Haversine formula
		double lat1 = wp.x; double lon1 = wp.y;
		double lat2 = curr_global_pose.latitude; double lon2 = curr_global_pose.longitude;

		double dLat = (lat2 - lat1) * M_PI / 180.0;
		double dLon = (lon2 - lon1) * M_PI / 180.0;

		lat1 = (lat1) * M_PI / 180.0;
		lat2 = (lat2) * M_PI / 180.0;

		double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
		double rad = 6371 * 1000;
		double c = 2*asin(sqrt(a));
		double flat_dist = rad * c; // in M

		//Pythag
		double targetAlt = this->grounded_alt + wp.z;
		double dAlt = abs(targetAlt - this->curr_alt);
		
		double dist = sqrt(flat_dist*flat_dist + dAlt*dAlt);
		
		ROS_INFO("Distance to next: %f", dist);
		if (dist < max_error){
			return true;
		}else{
			return false;
		}
	}
	
	//Simple Pythag Distance
	bool atWaypointLocal(){
	    Waypoint wp = _waypoints[curr_waypoint_ind];

	    double max_error = 1.0; // This is in meters

	    double diff_x = abs(wp.x - curr_local_pose.pose.pose.position.x);
	    double diff_y = abs(wp.y - curr_local_pose.pose.pose.position.y);
	    double diff_z = abs(wp.z - curr_local_pose.pose.pose.position.z);

	    double dist = sqrt(diff_x*diff_x + diff_y*diff_y + diff_z* diff_z);

	    if (dist < max_error){
		return true;
	    }else{
		return false;
	    }
	}


	bool isGPS;
	bool got_alt = false;

	ros::Publisher local_pose_pub;
	ros::Publisher global_pose_pub;

	nav_msgs::Odometry curr_local_pose;
	double  grounded_alt;
	double curr_alt;
	sensor_msgs::NavSatFix curr_global_pose;	
	std::vector<Waypoint> _waypoints;
	int curr_waypoint_ind;
};

int main(int argc, char** argv){
	ros::init(argc, argv, "waypoint_publisher");
	ros::NodeHandle nh;

	Navigator navigator(nh);
	ros::Subscriber current_local_pos = nh.subscribe("/mavros/global_position/local", 20, &Navigator::local_pose_callback, &navigator);	
	ros::Subscriber current_global_pos = nh.subscribe("/mavros/global_position/global", 20, &Navigator::global_pose_callback, &navigator); 
	ros::Subscriber altitude_sub = nh.subscribe("/mavros/altitude", 20, &Navigator::altitude_callback, &navigator);
	ROS_INFO("Here");
	navigator.setGroundAlt();
	//DEFINE THE MISSION BELOW
	

	//For a local mission
	/*
	//Load the waypoints;	
	ROS_INFO("LOADING WAYPOINTS...");
	std::vector<Waypoint> waypoints = {{100, 100, 20}, {0, 100, 20}, {200, 0, 20}};
	waypoints.push_back(waypoints[0]);
	navigator.loadWaypoints(waypoints);
	ROS_INFO("LOADED WAYPOINTS...");


	//Run the waypoint mission
	ROS_INFO("MISSION IN PROGRESS...");
	navigator.runMission();	
	*/

	
	//For a GPS Mission
	ROS_INFO("LOADING THE WAYPOINTS...");
	navigator.loadWaypointsFromFile("/mission9-2021/software/src/navigation_test/missions/first_mission.plan");
	ROS_INFO("SUCCESS: LOADED THE WAYPOINTS!!!");

	ROS_INFO("MISSION IN PROGRESS...");
	navigator.runMission();

	ROS_INFO("FINISHED!!!");

	return 0;

}

