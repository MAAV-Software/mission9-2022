
/**
 * Michigan Autonomous Aerial Vehicles 2023
 * file: reset_mast_position.cpp
 * brief: resets the position of the mast
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
#include <tf/tf.h>


using namespace cv;
using namespace std;



#define FLIGHT_ALTITUDE 1.5f


bool does_spin = true;
tf2::Quaternion myQuaternion;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    double tagSize = 0.35;

    ros::init(argc, argv, "mast_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    double time_per_iteration = 1.0 / 20.0;
    double total_time = 0.0f;

    total_time += time_per_iteration;
    //Set the pose
    geometry_msgs::Pose start_pose;
    // 2.60854 -0.055846 0.104595 -8.6e-05 -0.000258 -1.57996
    myQuaternion.setRPY(-8.6e-05, -0.000258, -1.57996);
    myQuaternion = myQuaternion.normalize();
    start_pose.position.x = 2.60854;
    start_pose.position.y = -.055846;
    start_pose.position.z = 0.104595;
    start_pose.orientation.x = myQuaternion.getX();
    start_pose.orientation.y = myQuaternion.getY();
    start_pose.orientation.z = myQuaternion.getZ();
    start_pose.orientation.w = myQuaternion.getW();

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
    modelstate.model_name = (std::string) "mast";
    modelstate.reference_frame = (std::string) "world";
    modelstate.pose = start_pose;
    modelstate.twist = start_twist;

    
    gazebo_msgs::SetModelState setmodelstate;
    setmodelstate.request.model_state = modelstate;
    client.call(setmodelstate);

    
    ros::spinOnce();
    rate.sleep();
    
    

    return 0;
}
  