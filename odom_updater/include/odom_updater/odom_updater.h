#pragma once

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
// #include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <string>
#include<sstream>

class OdomBroadcaster : public rclcpp::Node
{
public:
    OdomBroadcaster(std::string node_name) : Node(node_name)
    {
        
        // Initialize the transform broadcaster
        tf_broadcaster =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // std::ostringstream stream;
        // stream << "/robot1/odom";

        // std::string topic_name = stream.str();

        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/robot1/odom", 10,
        std::bind(&OdomBroadcaster::odom_callback, this, std::placeholders::_1));

        // Create a timer
        // timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / 1.0)),
        //                                   std::bind(&OdomBroadcaster::timer_callback, this));
    }

private:
    // attributes
    rclcpp::TimerBase::SharedPtr timer;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    
    // methods
    // void timer_callback();
    void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
};