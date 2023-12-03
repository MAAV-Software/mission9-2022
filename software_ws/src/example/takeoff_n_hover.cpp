#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

#define FLIGHT_ALTITUDE 1.5f
#define HOVER_DURATION 30.0
#define HOVER_X 0.0  // Adjust these values according to your desired hover position
#define HOVER_Y 0.0

// PID constants (adjust according to your needs)
#define KP 1.0
#define KI 0.0
#define KD 0.0

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}

double compute_pid(double setpoint, double current, double& prev_error, double integral) {
    double error = setpoint - current;
    integral += error;
    double derivative = error - prev_error;
    double output = KP * error + KI * integral + KD * derivative;
    prev_error = error;
    return output;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "takeoff_hover_land");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
            "mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
            "mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>(
            "mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
            "mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
            "mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>(
            "mavros/cmd/land");

    ros::Rate rate(20.0);

    // Wait for FCU connection
    while (ros::ok() && current_state.connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Connecting to FCU...");
    }

    mavros_msgs::PositionTarget target;
    target.type_mask = target.IGNORE_AFX | target.IGNORE_AFY | target.IGNORE_AFZ | target.IGNORE_VX | target.IGNORE_VY | target.IGNORE_VZ | target.IGNORE_YAW_RATE;

    // Send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(target);
        ros::spinOnce();
        rate.sleep();
    }

    // Change to offboard mode and arm
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok() && !current_state.armed) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(target);
        ros::spinOnce();
        rate.sleep();
    }

    // Start the image stuff
    // (You can add your image processing code here if needed)

    // Go to the first waypoint (hover)
    ros::Time start_time = ros::Time::now();
    double integral_z = 0.0;
    double prev_error_z = 0.0;

    while (ros::ok() && (ros::Time::now() - start_time).toSec() < HOVER_DURATION) {
        // Compute PID for z-axis
        double target_z = FLIGHT_ALTITUDE;
        double current_z = current_pose.pose.position.z;
        double output_z = compute_pid(target_z, current_z, prev_error_z, integral_z);

        // Update position target
        target.position.x = HOVER_X;
        target.position.y = HOVER_Y;
        target.position.z = FLIGHT_ALTITUDE + output_z;

        local_pos_pub.publish(target);
        ros::spinOnce();
        rate.sleep();
    }

    // Land
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    if (land_client.call(land_cmd) && land_cmd.response.success) {
        ROS_INFO("Landing...");
    }

    return 0;
}
