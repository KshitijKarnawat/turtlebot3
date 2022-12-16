#include <rclcpp/rclcpp.hpp>
#include "target_reacher/target_reacher.h"

void TargetReacher::spin(){

    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0.2;
    spin_publisher->publish(msg);
}

void TargetReacher::goal_reached_callback(const std_msgs::msg::Bool::SharedPtr msg){
   if(msg->data){
        spin();
   }
}

void TargetReacher::aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){
    
    auto final_x{0};
    auto final_y{0};

    if(msg->marker_ids.at(0) == 0){
        // get parameters
        rclcpp::Parameter aruco_0_x = this->get_parameter("final_destination.aruco_0.x");
        rclcpp::Parameter aruco_0_y = this->get_parameter("final_destination.aruco_0.y");
        
        final_x = aruco_0_x.as_double();
        final_y = aruco_0_y.as_double();
    }

    if(msg->marker_ids.at(0) == 1){
        // get parameters
        rclcpp::Parameter aruco_1_x = this->get_parameter("final_destination.aruco_1.x");
        rclcpp::Parameter aruco_1_y = this->get_parameter("final_destination.aruco_1.y");
        
        final_x = aruco_1_x.as_double();
        final_y = aruco_1_y.as_double();
    }

    if(msg->marker_ids.at(0) == 2){
        // get parameters
        rclcpp::Parameter aruco_2_x = this->get_parameter("final_destination.aruco_2.x");
        rclcpp::Parameter aruco_2_y = this->get_parameter("final_destination.aruco_2.y");
        
        final_x = aruco_2_x.as_double();
        final_y = aruco_2_y.as_double();
    }

    if(msg->marker_ids.at(0) == 3){
        // get parameters
        rclcpp::Parameter aruco_3_x = this->get_parameter("final_destination.aruco_3.x");
        rclcpp::Parameter aruco_3_y = this->get_parameter("final_destination.aruco_3.y");
        
        final_x = aruco_3_x.as_double();
        final_y = aruco_3_y.as_double();
    }
    m_bot_controller->set_goal(final_x, final_y);
}
