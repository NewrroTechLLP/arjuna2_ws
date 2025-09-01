/*
 * ROS Version: ROS 2 - Jazzy
 * Website: https://automaticaddison.com
 * Publishes odometry information for use with robot_pose_ekf package.
 * This odometry information is based on wheel encoder tick counts.
 *
 * Subscribe: ROS node that subscribes to the following topics:
 * right_ticks : Tick counts from the right motor encoder (std_msgs/Int64)
 * left_ticks : Tick counts from the left motor encoder (std_msgs/Int64)
 * initial_2d : The initial position and orientation of the robot.
 * (geometry_msgs/PoseStamped)
 *
 * Publish: This node will publish to the following topics:
 * odom_data_euler : Position and velocity estimate (nav_msgs/Odometry)
 * odom_data_quat : Position and velocity estimate in quaternion format
 * (nav_msgs/Odometry)
 *
 * Modified from Practical Robotics in C++ book by Lloyd Brombach
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <memory>
#include <chrono>

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.14159265358979323846;

// scrubby
const double TICKS_PER_REVOLUTION = 4095;
const double WHEEL_RADIUS = 0.04;
const double WHEEL_BASE = 0.28;
const double TICKS_PER_METER = 20475;

class EKF_Odom_Pub : public rclcpp::Node
{
public:
    EKF_Odom_Pub() : Node("ekf_odom_pub")
    {
        // Publishers
        odom_data_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_euler", 100);
        odom_data_pub_quat_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_quat", 100);

        // Subscribers with optional TransportHints, which is TCP_NODELAY by default in ROS 2
        subForRightCounts_ = this->create_subscription<std_msgs::msg::Int64>(
            "right_ticks",
            100,
            std::bind(&EKF_Odom_Pub::calc_right, this, std::placeholders::_1)
        );
        subForLeftCounts_ = this->create_subscription<std_msgs::msg::Int64>(
            "left_ticks",
            100,
            std::bind(&EKF_Odom_Pub::calc_left, this, std::placeholders::_1)
        );
        subInitialPose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "initial_2d",
            1,
            std::bind(&EKF_Odom_Pub::set_initial_2d, this, std::placeholders::_1)
        );

        // Timer for the main loop (updates odometry at 30 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // 1000ms / 30Hz
            std::bind(&EKF_Odom_Pub::update_and_publish, this)
        );

        // Initialize odometry messages
        odomNew.header.frame_id = "odom";
        odomNew.pose.pose.position.z = 0;
        odomNew.pose.pose.orientation.x = 0;
        odomNew.pose.pose.orientation.y = 0;
        odomNew.twist.twist.linear.x = 0;
        odomNew.twist.twist.linear.y = 0;
        odomNew.twist.twist.linear.z = 0;
        odomNew.twist.twist.angular.x = 0;
        odomNew.twist.twist.angular.y = 0;
        odomNew.twist.twist.angular.z = 0;
        odomOld.pose.pose.position.x = initialX;
        odomOld.pose.pose.position.y = initialY;
        odomOld.pose.pose.orientation.z = initialTheta;
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_quat_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subForRightCounts_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subForLeftCounts_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subInitialPose_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry odomNew;
    nav_msgs::msg::Odometry odomOld;
    double distanceLeft = 0;
    double distanceRight = 0;
    bool initialPoseRecieved = false;
    
    // To handle tick overflows
    int lastCountL = 0;
    int lastCountR = 0;

    void set_initial_2d(const geometry_msgs::msg::PoseStamped::SharedPtr rvizClick)
    {
        odomOld.pose.pose.position.x = rvizClick->pose.position.x;
        odomOld.pose.pose.position.y = rvizClick->pose.position.y;
        odomOld.pose.pose.orientation.z = rvizClick->pose.orientation.z;
        initialPoseRecieved = true;
    }

    void calc_left(const std_msgs::msg::Int64::SharedPtr leftCount)
    {
        if (leftCount->data != 0 && lastCountL != 0) {
            int leftTicks = leftCount->data - lastCountL;
            if (leftTicks > 10000) {
                leftTicks = 0 - (65535 - leftTicks);
            } else if (leftTicks < -10000) {
                leftTicks = 65535 - leftTicks;
            }
            distanceLeft = leftTicks / TICKS_PER_METER;
        }
        lastCountL = leftCount->data;
    }

    void calc_right(const std_msgs::msg::Int64::SharedPtr rightCount)
    {
        if (rightCount->data != 0 && lastCountR != 0) {
            int rightTicks = rightCount->data - lastCountR;
            if (rightTicks > 10000) {
                distanceRight = (0 - (65535 - rightTicks)) / TICKS_PER_METER;
            } else if (rightTicks < -10000) {
                rightTicks = 65535 - rightTicks;
            }
            distanceRight = rightTicks / TICKS_PER_METER;
        }
        lastCountR = rightCount->data;
    }

    void publish_quat()
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

        nav_msgs::msg::Odometry quatOdom;
        quatOdom.header.stamp = odomNew.header.stamp;
        quatOdom.header.frame_id = "odom";
        quatOdom.child_frame_id = "base_link";
        quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
        quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
        quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
        quatOdom.pose.pose.orientation.x = q.x();
        quatOdom.pose.pose.orientation.y = q.y();
        quatOdom.pose.pose.orientation.z = q.z();
        quatOdom.pose.pose.orientation.w = q.w();
        quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
        quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
        quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
        quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
        quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
        quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;

        for (int i = 0; i < 36; i++) {
            if (i == 0 || i == 7 || i == 14) {
                quatOdom.pose.covariance[i] = .01;
            } else if (i == 21 || i == 28 || i == 35) {
                quatOdom.pose.covariance[i] += 0.1;
            } else {
                quatOdom.pose.covariance[i] = 0;
            }
        }

        odom_data_pub_quat_->publish(quatOdom);
    }

    void update_odom()
    {
        // Calculate the average distance
        double cycleDistance = (distanceRight + distanceLeft) / 2.0;
        
        // Calculate the number of radians the robot has turned since the last cycle
        double cycleAngle = asin((distanceRight - distanceLeft) / WHEEL_BASE);

        // Average angle during the last cycle
        double avgAngle = cycleAngle / 2.0 + odomOld.pose.pose.orientation.z;

        if (avgAngle > PI) {
            avgAngle -= 2 * PI;
        } else if (avgAngle < -PI) {
            avgAngle += 2 * PI;
        }

        // Calculate the new pose (x, y, and theta)
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle) * cycleDistance;
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle) * cycleDistance;
        odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

        if (std::isnan(odomNew.pose.pose.position.x) || std::isnan(odomNew.pose.pose.position.y) || std::isnan(odomNew.pose.pose.position.z)) {
            odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
            odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
            odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
        }

        // Make sure theta stays in the correct range
        if (odomNew.pose.pose.orientation.z > PI) {
            odomNew.pose.pose.orientation.z -= 2 * PI;
        } else if (odomNew.pose.pose.orientation.z < -PI) {
            odomNew.pose.pose.orientation.z += 2 * PI;
        }

        // Compute the velocity
        odomNew.header.stamp = this->get_clock()->now();
        double delta_t = (rclcpp::Time(odomNew.header.stamp).seconds() - rclcpp::Time(odomOld.header.stamp).seconds());
        if (delta_t > 0.0) {
            odomNew.twist.twist.linear.x = cycleDistance / delta_t;
            odomNew.twist.twist.angular.z = cycleAngle / delta_t;
        } else {
            odomNew.twist.twist.linear.x = 0.0;
            odomNew.twist.twist.angular.z = 0.0;
        }

        // Save the pose data for the next cycle
        odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
        odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
        odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
        odomOld.header.stamp = odomNew.header.stamp;

        odom_data_pub_->publish(odomNew);
    }
    
    void update_and_publish()
    {
        if (initialPoseRecieved) {
            update_odom();
            publish_quat();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKF_Odom_Pub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}