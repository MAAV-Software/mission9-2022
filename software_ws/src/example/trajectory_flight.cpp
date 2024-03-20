#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <vector>
#include <sstream>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#define FLIGHT_ALTITUDE 1.5f
#define HOVER_DURATION 5.0
#define HOVER_X 0.0  
#define HOVER_Y 0.0

// PID constants 
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

std::vector<geometry_msgs::PoseStamped> read_waypoints(const std::string& file_path) {
    std::ifstream file(file_path);
    std::vector<geometry_msgs::PoseStamped> waypoints;

    if (file.is_open()) {
        std::string line;
        std::getline(file, line);
        std::getline(file, line);
        // WGS84 ellipsoid parameters
        const double a = 6378137.0;  // semi-major axis in meters
        const double f = 1.0 / 298.257223563;  // flattening

        GeographicLib::Geocentric earth(a, f);
        GeographicLib::LocalCartesian proj(0, 0, 0, earth);

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            geometry_msgs::PoseStamped waypoint;

            double lat, lon, alt_ft;
            iss >> lat >> lon >> alt_ft;

            double x, y, z;
            //proj.Forward(lat, lon, alt_ft * 0.3048, x, y, z);

            waypoint.pose.position.x = lat;
            waypoint.pose.position.y = lon;
            waypoint.pose.position.z = alt_ft;

            ROS_INFO("Reading waypoints from txt file: x=%f, y=%f, z=%f", lat, lon, alt_ft);
            waypoints.push_back(waypoint);
        }
        file.close();
    } else {
        ROS_ERROR("Unable to open waypoints file");
    }

    return waypoints;
}




// x = R * cos(lat) * cos(lon)

// y = R * cos(lat) * sin(lon)

// z = R *sin(lat)



int main(int argc, char **argv) {
    ros::init(argc, argv, "fly_through_waypoints");
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

    // Hover for a specified duration
    // ros::Time hover_start_time = ros::Time::now();
    // double integral_z = 0.0;
    // double prev_error_z = 0.0;

    // while (ros::ok() && (ros::Time::now() - hover_start_time).toSec() < HOVER_DURATION) {
    //     // Compute PID for z-axis
    //     double target_z = FLIGHT_ALTITUDE;
    //     double current_z = current_pose.pose.position.z;
    //     double output_z = compute_pid(target_z, current_z, prev_error_z, integral_z);

    //     // Update position target
    //     target.position.x = HOVER_X;
    //     target.position.y = HOVER_Y;
    //     target.position.z = FLIGHT_ALTITUDE + output_z;

    //     local_pos_pub.publish(target);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // Read waypoints from file
    std::vector<geometry_msgs::PoseStamped> waypoints = read_waypoints("way_points.txt");

    // Fly to waypoints
    for (const auto& waypoint : waypoints) {
        ros::Time waypoint_start_time = ros::Time::now();
        double lat = waypoint.pose.position.x;
        double lon = waypoint.pose.position.y;
        double alt_ft = waypoint.pose.position.z;
        ROS_INFO("Flying to the following waypoint: x=%f, y=%f, z=%f", lat, lon, alt_ft);

        while (ros::ok() && (ros::Time::now() - waypoint_start_time).toSec() < 50.0) {
            target.position.x = waypoint.pose.position.x;
            target.position.y = waypoint.pose.position.y;
            target.position.z = waypoint.pose.position.z;

            local_pos_pub.publish(target);
            ros::spinOnce();
            rate.sleep();
        }
    }

    // Return to takeoff position
    ros::Time return_start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - return_start_time).toSec() < 50.0) {
        target.position.x = HOVER_X;
        target.position.y = HOVER_Y;
        target.position.z = FLIGHT_ALTITUDE;

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
