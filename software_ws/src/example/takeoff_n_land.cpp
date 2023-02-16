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

#include "Utilities.h"


using namespace cv;
using namespace std;



#define FLIGHT_ALTITUDE 1.5f

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    Utilities U;
    double tagSize = 0.35;

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

    // //MARK - Set up Visual Servoing
    vpServo task; // Visual servoing taskZdf
    vpCameraParameters cam(565.6, 565.6, 320.5, 240.5);
  
    double lambda = 0.05;
    //vpAdaptiveGain lambda = vpAdaptiveGain(1.5, 0.7, 30);
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
    double Z_d = 1.5;

    // // This effectively takes the four points in camera frame and "pushes" them back by a distance Z_d (desired distance)
    // CREATES: vec_P_d which are the desired 4 corners of the mast in the camera frame --> (u1, v1), (u2, v2), (u3, v3), (u4, v4)
    double X[4] = {tagSize / 2., tagSize / 2., -tagSize / 2., -tagSize / 2.};
    double Y[4] = {tagSize / 2., -tagSize / 2., -tagSize / 2., tagSize / 2.};
    std::vector<vpPoint> vec_P, vec_P_d;

    for (int i = 0; i < 4; i++) {
        vpPoint P_d(X[i], Y[i], 0); 
        vpHomogeneousMatrix cdMo(0, 0, Z_d, 0, 0, 0);
        P_d.track(cdMo); //fj 
        vec_P_d.push_back(P_d);
    }

    vpMomentObject m_obj(3), m_obj_d(3);    //m_obj is some polygon object that is initialized of some vector of points of the mast
    vpMomentDatabase mdb, mdb_d;            // A database of moments that is necessary because some moments are dependent on each other. 
    vpMomentBasic mb_d;                     // Here only to get the desired area m00
    vpMomentGravityCenter mg, mg_d;         //Moment for the gravity center
    vpMomentCentered mc, mc_d;              //Moment Centered --> Depends on gravity center moment ^
    vpMomentAreaNormalized man(0, Z_d), man_d(0, Z_d); // Declare normalized area updated below with m00
    vpMomentGravityCenterNormalized mgn, mgn_d;        // Declare normalized gravity center
    
    
    // Desired moments
    m_obj_d.setType(vpMomentObject::DENSE_POLYGON); // Consider the AprilTag as a polygon type
    m_obj_d.fromVector(vec_P_d);                    // Initialize the object with the points coordinates
  
    mb_d.linkTo(mdb_d);       // Add basic moments to database
    mg_d.linkTo(mdb_d);       // Add gravity center to database
    mc_d.linkTo(mdb_d);       // Add centered moments to database
    man_d.linkTo(mdb_d);      // Add area normalized to database
    mgn_d.linkTo(mdb_d);      // Add gravity center normalized to database
    mdb_d.updateAll(m_obj_d); // All of the moments must be updated, not just man_d
    mg_d.compute();           // Compute gravity center moment
    mc_d.compute();           // Compute centered moments AFTER gravity center

    double area = 2;
    if (m_obj_d.getType() == vpMomentObject::DISCRETE)
        area = mb_d.get(2, 0) + mb_d.get(0, 2);
    else
        area = mb_d.get(0, 0);
    man_d.setDesiredArea(area); // Desired area was init at 0 (unknow at contruction),
                                // need to be updated here
    man_d.compute();            // Compute area normalized moment AFTER centered moment
    mgn_d.compute();            // Compute gravity center normalized moment AFTER area normalized
                                // moment
    
    //Define the image plane
    double A = 0.0;
    double B = 0.0;
    double C = 1.0 / Z_d;

    // Construct area normalized features
    std::cout << 1 << std::endl;
    vpFeatureMomentGravityCenterNormalized s_mgn(mdb, A, B, C), s_mgn_d(mdb_d, A, B, C);
    vpFeatureMomentAreaNormalized s_man(mdb, A, B, C), s_man_d(mdb_d, A, B, C);
    vpFeatureVanishingPoint s_vp, s_vp_d;
    
    //Compute two of the interaction matricies
    task.addFeature(s_mgn, s_mgn_d);
    task.addFeature(s_man, s_man_d);
    task.addFeature(s_vp, s_vp_d, vpFeatureVanishingPoint::selectAtanOneOverRho());

    // Update desired gravity center normalized feature
    s_mgn_d.update(A, B, C);
    s_mgn_d.compute_interaction();
    // Update desired area normalized feature
    s_man_d.update(A, B, C);
    s_man_d.compute_interaction();

    // Update desired vanishing point feature for the horizontal line
    s_vp_d.setAtanOneOverRho(0);
    s_vp_d.setAlpha(0);
    // //END - Ended setting up visual servoing

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

    //change to offboard mode and arm
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

    //                                                          MARK THIS HOVERS IN PLACE
    // go to the first waypoint
    target.position.x = 0;
    target.position.y = 0;
    target.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("going to the first way point");
    for(int i = 0; ros::ok() && i < 200; ++i){
      local_pos_pub.publish(target);
      ros::spinOnce();
      rate.sleep();
    }
    //                                                         END MARK THIS HOVERS IN PLACE


    // ROS_INFO("first way point finished!");


    //mavros_msgs::PositionTarget target;
    //

    //DEBUG: Testing visual servoing
    int num_errs = 0;
    while (true){
        // try{

        
        //Get some points
        vector<cv::Point> cv_points = ic.get_filtered_corner_points(); //TL TR BL BR
        vector<vpImagePoint> visp_mast_points = U.cvPointsToVPImagePoints(cv_points);

        if (cv_points[0].x == -1){
            
            
            std::cout << "Can't see crap, waiting a bit" << std::endl;
            // target.type_mask = target.IGNORE_AFX | target.IGNORE_AFY | target.IGNORE_AFZ | target.IGNORE_PX | target.IGNORE_PY | target.IGNORE_PZ | target.IGNORE_YAW_RATE;
            // target.velocity.x = 0;
            // target.velocity.y = 0;
            // target.velocity.z = 0; 
            
            TODO: local_pos_pub.publish(target);
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        cout << "visp_points: " << endl;
        for (auto point : visp_mast_points){
            std::cout << "(" << point.get_i() << ", " << point.get_j() << ") ";
        }
        std::cout << endl;
        cout << "cv_points: " << endl;
        for (auto point : cv_points){
            std::cout << "(" << point.x << ", " << point.y << ") ";
        }
        cout << endl;

       

        // Update current points used to compute the moments
        cout << 1 << endl;
        vec_P.clear();
        for (size_t i = 0; i < visp_mast_points.size(); i++) { // size = 4
            double x = 0, y = 0;
            vpPixelMeterConversion::convertPoint(cam, visp_mast_points[i], x, y);
            vpPoint P;
            P.set_x(x);
            P.set_y(y);
            vec_P.push_back(P);
            cout << P.get_x() << " " << P.get_y() << endl;
        }
        //make sure the elements are clockwise
        std::swap(vec_P[2], vec_P[3]);
        // Update Current moments
        cout << 2 << endl;


        m_obj.setType(vpMomentObject::DENSE_POLYGON); // Consider the MAST as a polygon
        m_obj.fromVector(vec_P);                      // Initialize the object with the points coordinates

        mg.linkTo(mdb);           // Add gravity center to database
        mc.linkTo(mdb);           // Add centered moments to database
        man.linkTo(mdb);          // Add area normalized to database
        mgn.linkTo(mdb);          // Add gravity center normalized to database
        mdb.updateAll(m_obj);     // All of the moments must be updated, not just an_d
        mg.compute();             // Compute gravity center moment
        mc.compute();             // Compute centered moments AFTER gravity center
        man.setDesiredArea(area); // Desired area was init at 0 (unknow at contruction),
                                    // need to be updated here
        man.compute();            // Compute area normalized moment AFTER centered moment
        mgn.compute();            // Compute gravity center normalized moment AFTER area normalized
                                    // moment
        cout << 2.5 << endl;

        s_mgn.update(A, B, C);
        s_mgn.compute_interaction();
        s_man.update(A, B, C);
        s_man.compute_interaction();
                cout << 3 << endl;





        //Top two points for first line, bottom two for the second line TL TR BL BR
        vpFeatureBuilder::create(s_vp, cam, visp_mast_points[0], visp_mast_points[1],
                                    visp_mast_points[2], visp_mast_points[3],
                                    vpFeatureVanishingPoint::selectAtanOneOverRho());

        //Print out the current features:
        std::cout << "CURRENT FEATURES:" << std::endl;
        s_mgn.print();
        s_man.print();
        cout << "atan(1/p): "<<s_vp.getAtanOneOverRho()<<endl;
        cout << endl;

        std::cout << "DESIERD FEATURES" << endl;
        s_mgn_d.print();
        s_man_d.print();
        cout << "desired atan(1/p): "<<s_vp_d.getAtanOneOverRho()<<endl;
        cout << endl;        


        task.set_cVe(cVe);
        task.set_eJe(eJe);
  
        // Compute the control law.
        vpColVector ve = task.computeControlLaw();
        std::cout << "------start------" << std::endl;
        std::cout << ve << std::endl;
        std::cout << "-------end-------" << std::endl;

    
        //Command the robo
        // target.type_mask = target.IGNORE_AFX | target.IGNORE_AFY | target.IGNORE_AFZ | target.IGNORE_PX | target.IGNORE_PY | target.IGNORE_PZ | target.IGNORE_YAW_RATE;
        // target.coordinate_frame = 1;
        // target.velocity.x = 0;
        // target.velocity.y = ve[2];
        // target.velocity.z = 0;
        local_pos_pub.publish(target);
        ros::spinOnce();
        rate.sleep();
    }


    // target.type_mask = target.IGNORE_AFX | target.IGNORE_AFY | target.IGNORE_AFZ | target.IGNORE_PX | target.IGNORE_PY | target.IGNORE_PZ | target.IGNORE_YAW_RATE;
    // while (true){
    //     cout << "X err --> "<< ic.get_x_offset() << "| Y err --> " << ic.get_y_offset() << endl;
    //     int x_err = ic.get_x_offset();
    //     int y_err = ic.get_y_offset();

    //     double K_P = 0.01;

    //     target.velocity.x = 0;//
    //     //target.velocity.y = -x_err * K_P; 
    //     target.velocity.y = x_err < 0 ?  max(-x_err * K_P, -0.05) : min(-x_err * K_P, 0.05);
    //     //target.velocity.z = -1*y_err * K_P;
    //     target.velocity.z = y_err < 0 ?  max(-y_err * K_P, -0.05) : min(-y_err * K_P, 0.05);
    //     target.coordinate_frame = 1;
    //     cout << "Y velocity: " << x_err * K_P << endl;

    //     local_pos_pub.publish(target);
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    

    return 0;
}
  