#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/transform_broadcaster.h"

// timer
class TargetReacher : public rclcpp::Node
{
public:
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {

        m_bot_controller = bot_controller;
        auto aruco_target_x = this->declare_parameter<double>("aruco_target.x");
        auto aruco_target_y = this->declare_parameter<double>("aruco_target.y");
        this->declare_parameter<std::string>("final_destination.frame_id");
        this->declare_parameter<double>("final_destination.aruco_0.x");
        this->declare_parameter<double>("final_destination.aruco_0.y");
        this->declare_parameter<double>("final_destination.aruco_1.x");
        this->declare_parameter<double>("final_destination.aruco_1.y");
        this->declare_parameter<double>("final_destination.aruco_2.x");
        this->declare_parameter<double>("final_destination.aruco_2.y");
        this->declare_parameter<double>("final_destination.aruco_3.x");
        this->declare_parameter<double>("final_destination.aruco_3.y");

        m_bot_controller->set_goal(aruco_target_x, aruco_target_y);


        // subscribes to topic /goal_reached
        goal_reached_subscriber = this->create_subscription<std_msgs::msg::Bool>("/goal_reached", 10, std::bind(&TargetReacher::goal_reached_callback, this, std::placeholders::_1));
        
        // publishes to topic /robot1/odom
        spin_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

        // subscriber to topic /aruco_marker
        aruco_subscriber = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 10, std::bind(&TargetReacher::aruco_callback, this, std::placeholders::_1));

    }

    void spin();
    void goal_reached_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

private:
    // attributes
    std::shared_ptr<BotController> m_bot_controller;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_subscriber;
    
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr spin_publisher;

};



// TODO
// listen to output of set_goal (subscribe to goal_reached/data)
// spin at location (publish to cmd_vel/angular_velocity and cmd_vel/linear_velocity)
// find marker (if marker found read id (id is a vector use vector.begin()) and call set_goal())
// subscribe to goal rreached
        // if goal reached == true:
        //     spin();
        // find marker()
        // if marker found:
        //      subscribe to aruco_markers
        //         get the marker id (vector)
        //         vector.begin()
        //         if id = 0
        //         goto aruco_0 for frame_id
        //         if id = 1
        //         goto aruco_1 for frame_id
        //         if id = 2
        //         goto aruco_2 for frame_id
        //         if id = 3
        //         goto aruco_3 for frame_id
        // make a new tf2 say final_dest
        // final_dest = origin + aruco_id (the final_det wrt world)
        // set+goal (final_dest.x, final_dest.y)
