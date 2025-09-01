/*
 * ROS Version: ROS 2 - Jazzy
 * Converts Rviz 2D Nav Goal and initialpose topics to simpler 2D poses.
 * This is useful for a simple navigation stack that doesn't use the full
 * 3D pose and covariance data.
 *
 * Subscribe:
 * /move_base_simple/goal (geometry_msgs/PoseStamped)
 * /initialpose (geometry_msgs/PoseWithCovarianceStamped)
 *
 * Publish:
 * /goal_2d (geometry_msgs/PoseStamped)
 * /initial_2d (geometry_msgs/PoseStamped)
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>
#include <memory>

class PoseConverter : public rclcpp::Node
{
public:
    PoseConverter() : Node("rviz_click_to_2d")
    {
        // Publishers
        goal_2d_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_2d", 10);
        initial_2d_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("initial_2d", 10);

        // Subscribers
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal",
            10,
            std::bind(&PoseConverter::handle_goal, this, std::placeholders::_1)
        );

        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            10,
            std::bind(&PoseConverter::handle_initial_pose, this, std::placeholders::_1)
        );
    }

private:
    void handle_goal(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
    {
        geometry_msgs::msg::PoseStamped rpyGoal;
        rpyGoal.header.frame_id = "map";
        rpyGoal.header.stamp = goal->header.stamp;
        rpyGoal.pose.position.x = goal->pose.position.x;
        rpyGoal.pose.position.y = goal->pose.position.y;
        rpyGoal.pose.position.z = 0;

        tf2::Quaternion q;
        q.setW(goal->pose.orientation.w);
        q.setZ(goal->pose.orientation.z);
        q.setX(goal->pose.orientation.x);
        q.setY(goal->pose.orientation.y);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        rpyGoal.pose.orientation.x = 0;
        rpyGoal.pose.orientation.y = 0;
        rpyGoal.pose.orientation.z = yaw;
        rpyGoal.pose.orientation.w = 0;

        goal_2d_pub_->publish(rpyGoal);
    }

    void handle_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
    {
        geometry_msgs::msg::PoseStamped rpyPose;
        rpyPose.header.frame_id = "map";
        rpyPose.header.stamp = pose->header.stamp;
        rpyPose.pose.position.x = pose->pose.pose.position.x;
        rpyPose.pose.position.y = pose->pose.pose.position.y;
        rpyPose.pose.position.z = 0;

        tf2::Quaternion q;
        q.setW(pose->pose.pose.orientation.w);
        q.setZ(pose->pose.pose.orientation.z);
        q.setX(pose->pose.pose.orientation.x);
        q.setY(pose->pose.pose.orientation.y);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        rpyPose.pose.orientation.x = 0;
        rpyPose.pose.orientation.y = 0;
        rpyPose.pose.orientation.z = yaw;
        rpyPose.pose.orientation.w = 0;

        initial_2d_pub_->publish(rpyPose);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_2d_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr initial_2d_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}