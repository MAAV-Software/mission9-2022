/**
 * Michigan Autonomous Aerial Vehicles 2023
 * file: ball_mover.cpp
 * brief: moves the guiding ball around
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <gazebo_msgs/SetModelState.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <tuple>
#include <cstdlib>
#include <math.h>
#include <cmath>
#include <string>
#include <cassert>
#include "ImageConverter.h"
#include <algorithm> 


using namespace cv;
using namespace std;



#define FLIGHT_ALTITUDE 1.5f


bool does_spin = true;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_node");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    double time_per_iteration = 1.0 / 20.0;
    double total_time = 0.0f;

    while (true){
        total_time += time_per_iteration;
        //Set the pose
        geometry_msgs::Pose start_pose;
        start_pose.position.x = 0.0;
        start_pose.position.y = sin(total_time);
        start_pose.position.z = 0.0;
        start_pose.orientation.x = 0;
        start_pose.orientation.y = 0;
        start_pose.orientation.z = 0;
        start_pose.orientation.w = 0;

        //Set the velocity
        geometry_msgs::Twist start_twist;
        start_twist.linear.x = 0.0;
        start_twist.linear.y = 0.0;
        start_twist.linear.z = 0.0;
        start_twist.angular.x = 0.0;
        start_twist.angular.y = 0.0;
        start_twist.angular.z = 0.0;

        //Setup ModelState
        gazebo_msgs::ModelState modelstate;
        modelstate.model_name = (std::string) "unit_sphere";
        modelstate.reference_frame = (std::string) "world";
        modelstate.pose = start_pose;
        modelstate.twist = start_twist;

        
        gazebo_msgs::SetModelState setmodelstate;
        setmodelstate.request.model_state = modelstate;
        client.call(setmodelstate);

        
        ros::spinOnce();
        rate.sleep();
    }
    

    return 0;
}
  