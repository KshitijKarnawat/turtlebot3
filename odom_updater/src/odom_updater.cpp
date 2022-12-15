#include <string>
#include "odom_updater.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>

// void OdomBroadcaster::timer_callback()
// {
//     odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
// }

void OdomBroadcaster::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    geometry_msgs::msg::TransformStamped t;
    

    std::string base_footprint = "/robot1/base_footprint";

    /*******************************************
     *  broadcaster: "/robot1/odom" -> "/robot1/base_footprint"
     *******************************************/
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "/robot1/odom";
    t.child_frame_id = base_footprint;

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    // Send the transformation
    tf_broadcaster->sendTransform(t);


}

