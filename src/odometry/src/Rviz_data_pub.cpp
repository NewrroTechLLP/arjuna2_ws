#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <memory>

class RvizDataPublisher : public rclcpp::Node
{
public:
    RvizDataPublisher() : Node("rviz_click_to_2d")
    {
        // Create publishers
        goal_2d_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_2d", 10);
        initial_2d_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("initial_2d", 10);
        
        // Create subscribers
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "move_base_simple/goal", 10,
            std::bind(&RvizDataPublisher::handle_goal, this, std::placeholders::_1));
            
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10,
            std::bind(&RvizDataPublisher::handle_initial_pose, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "RViz Click to 2D Publisher Node Started");
        RCLCPP_INFO(this->get_logger(), "Publishers: goal_2d, initial_2d");
        RCLCPP_INFO(this->get_logger(), "Subscribers: move_base_simple/goal, initialpose");
    }

private:
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_2d_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr initial_2d_pub_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

    void handle_goal(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
    {
        geometry_msgs::msg::PoseStamped rpy_goal;
        rpy_goal.header.frame_id = "map";
        rpy_goal.header.stamp = goal->header.stamp;
        rpy_goal.pose.position.x = goal->pose.position.x;
        rpy_goal.pose.position.y = goal->pose.position.y;
        rpy_goal.pose.position.z = 0.0;
        
        // Convert quaternion to RPY
        tf2::Quaternion q(
            goal->pose.orientation.x,
            goal->pose.orientation.y, 
            goal->pose.orientation.z,
            goal->pose.orientation.w
        );
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        rpy_goal.pose.orientation.x = 0.0;
        rpy_goal.pose.orientation.y = 0.0;
        rpy_goal.pose.orientation.z = yaw;
        rpy_goal.pose.orientation.w = 0.0;
        
        goal_2d_pub_->publish(rpy_goal);
        
        RCLCPP_INFO(this->get_logger(), "Published goal_2d: x=%.2f, y=%.2f, yaw=%.2f", 
                   rpy_goal.pose.position.x, rpy_goal.pose.position.y, yaw);
    }

    void handle_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
    {
        geometry_msgs::msg::PoseStamped rpy_pose;
        rpy_pose.header.frame_id = "map";
        rpy_pose.header.stamp = pose->header.stamp;
        rpy_pose.pose.position.x = pose->pose.pose.position.x;
        rpy_pose.pose.position.y = pose->pose.pose.position.y;
        rpy_pose.pose.position.z = 0.0;
        
        // Convert quaternion to RPY
        tf2::Quaternion q(
            pose->pose.pose.orientation.x,
            pose->pose.pose.orientation.y,
            pose->pose.pose.orientation.z,
            pose->pose.pose.orientation.w
        );
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        rpy_pose.pose.orientation.x = 0.0;
        rpy_pose.pose.orientation.y = 0.0;
        rpy_pose.pose.orientation.z = yaw;
        rpy_pose.pose.orientation.w = 0.0;
        
        initial_2d_pub_->publish(rpy_pose);
        
        RCLCPP_INFO(this->get_logger(), "Published initial_2d: x=%.2f, y=%.2f, yaw=%.2f", 
                   rpy_pose.pose.position.x, rpy_pose.pose.position.y, yaw);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RvizDataPublisher>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
