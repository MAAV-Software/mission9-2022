/**
 * Michigan Autonomous Aerial Vehicles 2023
 * file: mast_mover.cpp
 * brief: ______
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
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
// #include <Vector3.h>

using namespace cv;
using namespace std;
using namespace tf2;


#define FLIGHT_ALTITUDE 1.5f


bool does_spin = true;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

gazebo_msgs::ModelStates current_model_state;
size_t myIndex = 4;
tf2::Vector3 myVec;
void model_state_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
    current_model_state = *msg;
    // if(myIndex == -1)
    // {
    //     for(int i = 0; i < current_model_state.name.size(); i++)
    //     {
    //         if(current_model_state.name[0] == "unit_sphere")
    //         {
    //             myIndex = i;
    //             break;
    //         }
    //     }
    // }
    assert(current_model_state.name[4] == "unit_sphere");
    // cout << current_model_state.pose[4].position.x << " ";
    //  cout << current_model_state.pose[4].position.y << endl;
    myVec = Vector3(current_model_state.pose[4].position.x, current_model_state.pose[4].position.y, current_model_state.pose[4].position.z);

}

int main(int argc, char **argv)
{
    double tagSize = 0.35;

    ros::init(argc, argv, "mast_node");
    ros::NodeHandle nh;

    ros::Subscriber model_state_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, model_state_cb);
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    double time_per_iteration = 1.0 / 20.0;
    double total_time = 0.0f;

    while (true){
        total_time += time_per_iteration;
    
        //Set the pose
        geometry_msgs::Pose start_pose;
        start_pose.position.x = 2.60854;
        start_pose.position.y = -.055846;
        start_pose.position.z = 0.104595;
        start_pose.orientation.x = -0.2;
        start_pose.orientation.y = -0.5;
        start_pose.orientation.z = 1;
        start_pose.orientation.w = sin(total_time);

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
    }
    

    return 0;
}
  