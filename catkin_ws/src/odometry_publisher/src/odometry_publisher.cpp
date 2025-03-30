#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h> // Serial communication for Jetson-Arduino
#include <cmath>

// Global Variables
double x = 0.0, y = 0.0, th = 0.0; // Position & orientation
double prev_right_distance = 0.0, prev_left_distance = 0.0; // Store previous encoder readings

const double WHEEL_BASE = 0.54; // Distance between wheels (meters)
serial::Serial ser; // Serial communication object

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    // Open Serial Port
    try {
        ser.setPort("/dev/ttyACM0"); // Adjust as needed
        ser.setBaudrate(115200);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(timeout);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR("Unable to open serial port!");
        return -1;
    }

    ros::Time last_time = ros::Time::now();
    ros::Rate r(10.0); // 10 Hz loop

    while (ros::ok()) {
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;

        // Read total displacement from encoders
        double right_distance = 0.0, left_distance = 0.0;
        if (ser.available()) {
            std::string serial_data = ser.readline();
            if (sscanf(serial_data.c_str(), "Right Displacement:%lf,Left Displacement:%lf", 
                       &right_distance, &left_distance) == 2) {
                // Successfully read total displacement values
            }
        }

        // 🔹 Compute per-loop displacement (ΔD) by subtracting previous readings
        double delta_s_right = right_distance - prev_right_distance;
        double delta_s_left = left_distance - prev_left_distance;

        // Update previous displacement values for next iteration
        prev_right_distance = right_distance;
        prev_left_distance = left_distance;

        // 🔹 Compute forward displacement and rotation
        double delta_s = (delta_s_right + delta_s_left) / 2.0;
        double delta_th = (delta_s_right - delta_s_left) / WHEEL_BASE;

        // 🔹 Update robot position
        x += delta_s * cos(th + delta_th / 2.0);
        y += delta_s * sin(th + delta_th / 2.0);
        th += delta_th;

        // Normalize theta to [-pi, pi]
        th = atan2(sin(th), cos(th));

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        // 🔹 Compute linear and angular velocity
        double vx = delta_s / dt;
        double vth = delta_th / dt;

        // Publish odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;  // Correct velocity calculation
        odom.twist.twist.angular.z = vth;
        odom_pub.publish(odom);

        r.sleep();
    }
}

