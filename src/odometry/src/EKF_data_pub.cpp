/*
 * Automatic Addison - Modified for ROS 2
 * Date: Converted for ROS 2
 * ROS Version: ROS 2
 * Website: https://automaticaddison.com
 * Publishes odometry information for use with robot_pose_ekf package.
 *   This odometry information is based on wheel encoder tick counts.
 * Subscribe: ROS node that subscribes to the following topics:
 *  right_ticks : Tick counts from the right motor encoder (std_msgs/Int64)
 *  left_ticks : Tick counts from the left motor encoder  (std_msgs/Int64)
 *  initial_2d : The initial position and orientation of the robot.
 *               (geometry_msgs/PoseStamped)
 *
 * Publish: This node will publish to the following topics:
 *  odom_data_euler : Position and velocity estimate. The orientation.z 
 *                    variable is an Euler angle representing the yaw angle.
 *                    (nav_msgs/Odometry)
 *  odom_data_quat : Position and velocity estimate. The orientation is 
 *                   in quaternion format.
 *                   (nav_msgs/Odometry)
 */
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include <memory>

class EKFDataPublisher : public rclcpp::Node
{
public:
    EKFDataPublisher() : Node("ekf_odom_pub")
    {
        // Declare parameters
        this->declare_parameter("ticks_per_revolution", 4095.0);
        this->declare_parameter("wheel_radius", 0.04);
        this->declare_parameter("wheel_base", 0.28);
        this->declare_parameter("ticks_per_meter", 20475.0);
        this->declare_parameter("initial_x", 0.0);
        this->declare_parameter("initial_y", 0.0);
        this->declare_parameter("initial_theta", 0.00000000001);
        this->declare_parameter("publish_rate", 30.0);

        // Get parameters
        ticks_per_revolution_ = this->get_parameter("ticks_per_revolution").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        ticks_per_meter_ = this->get_parameter("ticks_per_meter").as_double();
        initial_x_ = this->get_parameter("initial_x").as_double();
        initial_y_ = this->get_parameter("initial_y").as_double();
        initial_theta_ = this->get_parameter("initial_theta").as_double();
        
        // Create publishers
        odom_data_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_euler", 100);
        odom_data_pub_quat_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_quat", 100);
        
        // Create subscribers
        right_ticks_sub_ = this->create_subscription<std_msgs::msg::Int64>(
            "right_ticks", 100,
            std::bind(&EKFDataPublisher::calc_right, this, std::placeholders::_1));
            
        left_ticks_sub_ = this->create_subscription<std_msgs::msg::Int64>(
            "left_ticks", 100,
            std::bind(&EKFDataPublisher::calc_left, this, std::placeholders::_1));
            
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "initial_2d", 1,
            std::bind(&EKFDataPublisher::set_initial_2d, this, std::placeholders::_1));
        
        // Initialize odometry messages
        initialize_odometry();
        
        // Create timer for main loop
        double rate_hz = this->get_parameter("publish_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz)),
            std::bind(&EKFDataPublisher::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "EKF Odometry Publisher Node Started");
    }

private:
    // Constants
    static constexpr double PI = 3.141592653589793;
    
    // Parameters
    double ticks_per_revolution_;
    double wheel_radius_;
    double wheel_base_;
    double ticks_per_meter_;
    double initial_x_;
    double initial_y_;
    double initial_theta_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_quat_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr right_ticks_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr left_ticks_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Odometry data
    nav_msgs::msg::Odometry odom_new_;
    nav_msgs::msg::Odometry odom_old_;
    
    // Distance tracking
    double distance_left_ = 0.0;
    double distance_right_ = 0.0;
    
    // State
    bool initial_pose_received_ = false;
    int last_count_left_ = 0;
    int last_count_right_ = 0;
    bool first_left_ = true;
    bool first_right_ = true;

    void initialize_odometry()
    {
        // Set the data fields of the odometry message
        odom_new_.header.frame_id = "odom";
        odom_new_.child_frame_id = "base_link";
        odom_new_.pose.pose.position.z = 0.0;
        odom_new_.pose.pose.orientation.x = 0.0;
        odom_new_.pose.pose.orientation.y = 0.0;
        odom_new_.twist.twist.linear.x = 0.0;
        odom_new_.twist.twist.linear.y = 0.0;
        odom_new_.twist.twist.linear.z = 0.0;
        odom_new_.twist.twist.angular.x = 0.0;
        odom_new_.twist.twist.angular.y = 0.0;
        odom_new_.twist.twist.angular.z = 0.0;
        
        odom_old_.pose.pose.position.x = initial_x_;
        odom_old_.pose.pose.position.y = initial_y_;
        odom_old_.pose.pose.orientation.z = initial_theta_;
        odom_old_.header.stamp = this->get_clock()->now();
    }

    void set_initial_2d(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        odom_old_.pose.pose.position.x = msg->pose.position.x;
        odom_old_.pose.pose.position.y = msg->pose.position.y;
        odom_old_.pose.pose.orientation.z = msg->pose.orientation.z;
        initial_pose_received_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Initial pose set: x=%.2f, y=%.2f, theta=%.2f", 
                   msg->pose.position.x, msg->pose.position.y, msg->pose.orientation.z);
    }

    void calc_left(const std_msgs::msg::Int64::SharedPtr msg)
    {
        if (first_left_) {
            last_count_left_ = msg->data;
            first_left_ = false;
            return;
        }
        
        if (msg->data != 0) {
            int left_ticks = static_cast<int>(msg->data - last_count_left_);
            
            if (left_ticks > 10000) {
                left_ticks = 0 - (65535 - left_ticks);
            }
            else if (left_ticks < -10000) {
                left_ticks = 65535 - left_ticks;
            }
            
            distance_left_ = static_cast<double>(left_ticks) / ticks_per_meter_;
        }
        last_count_left_ = msg->data;
    }

    void calc_right(const std_msgs::msg::Int64::SharedPtr msg)
    {
        if (first_right_) {
            last_count_right_ = msg->data;
            first_right_ = false;
            return;
        }
        
        if (msg->data != 0) {
            int right_ticks = static_cast<int>(msg->data - last_count_right_);
            
            if (right_ticks > 10000) {
                right_ticks = 0 - (65535 - right_ticks);
            }
            else if (right_ticks < -10000) {
                right_ticks = 65535 - right_ticks;
            }
            
            distance_right_ = static_cast<double>(right_ticks) / ticks_per_meter_;
        }
        last_count_right_ = msg->data;
    }

    void publish_quat()
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_new_.pose.pose.orientation.z);

        nav_msgs::msg::Odometry quat_odom;
        quat_odom.header.stamp = odom_new_.header.stamp;
        quat_odom.header.frame_id = "odom";
        quat_odom.child_frame_id = "base_link";
        quat_odom.pose.pose.position.x = odom_new_.pose.pose.position.x;
        quat_odom.pose.pose.position.y = odom_new_.pose.pose.position.y;
        quat_odom.pose.pose.position.z = odom_new_.pose.pose.position.z;
        quat_odom.pose.pose.orientation.x = q.x();
        quat_odom.pose.pose.orientation.y = q.y();
        quat_odom.pose.pose.orientation.z = q.z();
        quat_odom.pose.pose.orientation.w = q.w();
        quat_odom.twist.twist.linear.x = odom_new_.twist.twist.linear.x;
        quat_odom.twist.twist.linear.y = odom_new_.twist.twist.linear.y;
        quat_odom.twist.twist.linear.z = odom_new_.twist.twist.linear.z;
        quat_odom.twist.twist.angular.x = odom_new_.twist.twist.angular.x;
        quat_odom.twist.twist.angular.y = odom_new_.twist.twist.angular.y;
        quat_odom.twist.twist.angular.z = odom_new_.twist.twist.angular.z;

        // Set covariance
        for (int i = 0; i < 36; i++) {
            if (i == 0 || i == 7 || i == 14) {
                quat_odom.pose.covariance[i] = 0.01;
            }
            else if (i == 21 || i == 28 || i == 35) {
                quat_odom.pose.covariance[i] = 0.1;
            }
            else {
                quat_odom.pose.covariance[i] = 0.0;
            }
        }

        odom_data_pub_quat_->publish(quat_odom);
    }

    void update_odom()
    {
        // Calculate the average distance
        double cycle_distance = (distance_right_ + distance_left_) / 2.0;

        // Calculate the number of radians the robot has turned since the last cycle
        double cycle_angle = asin((distance_right_ - distance_left_) / wheel_base_);

        // Average angle during the last cycle
        double avg_angle = cycle_angle / 2.0 + odom_old_.pose.pose.orientation.z;

        if (avg_angle > PI) {
            avg_angle -= 2 * PI;
        }
        else if (avg_angle < -PI) {
            avg_angle += 2 * PI;
        }

        // Calculate the new pose (x, y, and theta)
        odom_new_.pose.pose.position.x = odom_old_.pose.pose.position.x + cos(avg_angle) * cycle_distance;
        odom_new_.pose.pose.position.y = odom_old_.pose.pose.position.y + sin(avg_angle) * cycle_distance;
        odom_new_.pose.pose.orientation.z = cycle_angle + odom_old_.pose.pose.orientation.z;

        // Prevent lockup from a single bad cycle
        if (std::isnan(odom_new_.pose.pose.position.x) || std::isnan(odom_new_.pose.pose.position.y) ||
            std::isnan(odom_new_.pose.pose.orientation.z)) {
            odom_new_.pose.pose.position.x = odom_old_.pose.pose.position.x;
            odom_new_.pose.pose.position.y = odom_old_.pose.pose.position.y;
            odom_new_.pose.pose.orientation.z = odom_old_.pose.pose.orientation.z;
        }

        // Make sure theta stays in the correct range
        if (odom_new_.pose.pose.orientation.z > PI) {
            odom_new_.pose.pose.orientation.z -= 2 * PI;
        }
        else if (odom_new_.pose.pose.orientation.z < -PI) {
            odom_new_.pose.pose.orientation.z += 2 * PI;
        }

        // Compute the velocity
        odom_new_.header.stamp = this->get_clock()->now();
        rclcpp::Time new_time(odom_new_.header.stamp);
        rclcpp::Time old_time(odom_old_.header.stamp);
        double dt = (new_time - old_time).seconds();
        
        if (dt > 0) {
            odom_new_.twist.twist.linear.x = cycle_distance / dt;
            odom_new_.twist.twist.angular.z = cycle_angle / dt;
        }

        // Save the pose data for the next cycle
        odom_old_.pose.pose.position.x = odom_new_.pose.pose.position.x;
        odom_old_.pose.pose.position.y = odom_new_.pose.pose.position.y;
        odom_old_.pose.pose.orientation.z = odom_new_.pose.pose.orientation.z;
        odom_old_.header.stamp = odom_new_.header.stamp;

        // Publish the odometry message
        odom_data_pub_->publish(odom_new_);
    }

    void timer_callback()
    {
        if (initial_pose_received_) {
            update_odom();
            publish_quat();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKFDataPublisher>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
