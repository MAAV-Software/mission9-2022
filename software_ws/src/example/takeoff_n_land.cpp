/**
 * Michigan Autonomous Aerial Vehicles 2022
 * file: takeoff_n_land.cpp
 * brief: simple ros node to control px4 with local waypoints
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <tuple>
#include <cstdlib>
#include <math.h>
#include <string>
#include <cassert>
#include "ImageConverter.h"
#include <algorithm> 

#include <visp3/core/vpImage.h>

//Visp Stuff
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMomentAreaNormalized.h>
#include <visp3/core/vpMomentBasic.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentGravityCenterNormalized.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpXmlParserCamera.h>

//Other Visp Stuff
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureMomentAreaNormalized.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenterNormalized.h>
#include <visp3/visual_features/vpFeatureVanishingPoint.h>
//#include <visp3/vs/vpServo.h>
//#include <visp3/vs/vpServoDisplay.h>
#include <visp/vpServo.h>


using namespace cv;
using namespace std;



#define FLIGHT_ALTITUDE 1.5f

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
      ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    //MARK - Set up Visual Servoing
    vpServo task; // Visual servoing task
  
    // double lambda = 0.5;
    vpAdaptiveGain lambda = vpAdaptiveGain(1.5, 0.7, 30);
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(lambda);

    vpRxyzVector c1_rxyz_c2(0, 0, 0);
    vpRotationMatrix c1Rc2(c1_rxyz_c2);
    vpHomogeneousMatrix c1Mc2(vpTranslationVector(), c1Rc2); //Homo matrix from c1 to c2

    vpRotationMatrix c1Re{0, 1, 0, 0, 0, 1, 1, 0, 0};
    vpTranslationVector c1te(0, 0, 0); // TODO: Add translation vector 
    vpHomogeneousMatrix c1Me(c1te, c1Re);

    vpHomogeneousMatrix c2Me = c1Mc2.inverse() * c1Me;
    vpVelocityTwistMatrix cVe(c2Me);
    
    task.set_cVe(cVe); // TODO- See if this is actually needed
    

    vpMatrix eJe(6, 4, 0);

    eJe[0][0] = 1;
    eJe[1][1] = 1;
    eJe[2][2] = 1;
    eJe[5][3] = 1;

    //Desired distance to the target
    double Z_d = 1.;
    

    


    //END - Ended setting up visual servoing

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT...");
    }

    mavros_msgs::PositionTarget target;
    target.type_mask = target.IGNORE_AFX | target.IGNORE_AFY | target.IGNORE_AFZ | target.IGNORE_VX | target.IGNORE_VY | target.IGNORE_VZ | target.IGNORE_YAW_RATE;

    target.position.x = 0;
    target.position.y = 0;
    target.position.z = FLIGHT_ALTITUDE;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(target);
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
        local_pos_pub.publish(target);
        ros::spinOnce();
        rate.sleep();
    }


    //Start the image stuff
    ImageConverter ic;

    // go to the first waypoint
    target.position.x = 0;
    target.position.y = 0;
    target.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("going to the first way point");
    for(int i = 0; ros::ok() && true/*i < 200*/; ++i){
      local_pos_pub.publish(target);
      ros::spinOnce();
      rate.sleep();



    }
    ROS_INFO("first way point finished!");


    //mavros_msgs::PositionTarget target;
    target.type_mask = target.IGNORE_AFX | target.IGNORE_AFY | target.IGNORE_AFZ | target.IGNORE_PX | target.IGNORE_PY | target.IGNORE_PZ | target.IGNORE_YAW_RATE;


    

    while (true){
        cout << "X err --> "<< ic.get_x_offset() << "| Y err --> " << ic.get_y_offset() << endl;
        int x_err = ic.get_x_offset();
        int y_err = ic.get_y_offset();

        double K_P = 0.01;

        target.velocity.x = 0;//
        // target.velocity.y = -x_err * K_P; 
        x_err < 0 ? target.velocity.y = max(-x_err * K_P, -0.1) : target.velocity.y = min(-x_err * K_P, 0.1);
        // target.velocity.z = -1*y_err * K_P;
        y_err < 0 ? target.velocity.z = max(-y_err * K_P, -0.1) : target.velocity.z = min(-y_err * K_P, 0.1);
        target.coordinate_frame = 1;
        cout << "X velocity: " << x_err * K_P << endl;

        local_pos_pub.publish(target);
        ros::spinOnce();
        rate.sleep();
    }
    
    // while (true){
    //     target.velocity.x = 0;
    //     target.velocity.y = 1;
    //     target.velocity.z = 0;
    //     target.coordinate_frame = 1;

    //     ROS_INFO("going to the first way point");
    //     for(int i = 0; ros::ok() && i < 200; ++i){
    //     local_pos_pub.publish(target);
    //     ros::spinOnce();
    //     rate.sleep();
    //     }
    //     ROS_INFO(" way point finished!");

    //     target.velocity.x = 0;
    //     target.velocity.y = -1;
    //     target.velocity.z = 0;
    //     target.coordinate_frame = 1;

    //     ROS_INFO("going to the first way point");
    //     for(int i = 0; ros::ok() && i < 200; ++i){
    //     local_pos_pub.publish(target);
    //     ros::spinOnce();
    //     rate.sleep();
    //     }
    //     ROS_INFO("first way point finished!");
    // }
    

    return 0;
}
