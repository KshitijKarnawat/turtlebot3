#include <string>
#include "odom_updater.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

void OdomBroadcaster::timer_callback()
{
    broadcast_odom();
}

void OdomBroadcaster::broadcast_odom()
{
    geometry_msgs::msg::TransformStamped t;

    std::string base_footprint = "/robot1/base_footprint";

    /*******************************************
     *  broadcaster: "/robot1/odom" -> "/robot1/base_footprint"
     *******************************************/
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "/robot1/odom";
    t.child_frame_id = base_footprint;

    t.transform.translation.x = 0;
    t.transform.translation.y = 0;
    t.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster->sendTransform(t);


}

