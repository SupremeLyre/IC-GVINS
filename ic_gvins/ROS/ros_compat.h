#ifndef IC_GVINS_ROS_COMPAT_H
#define IC_GVINS_ROS_COMPAT_H

#if ROS_AVAILABLE == 1
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#elif ROS_AVAILABLE == 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#else
#error "Build the ROS interface with ROS_AVAILABLE=1 or ROS_AVAILABLE=2"
#endif

// Message fields and the fusion algorithms are shared by both ROS interfaces.
namespace gvins_ros {

#if ROS_AVAILABLE == 1
namespace sensor = sensor_msgs;
namespace nav = nav_msgs;
namespace geometry = geometry_msgs;
using ImuConstPtr = sensor_msgs::ImuConstPtr;
using NavSatFixConstPtr = sensor_msgs::NavSatFixConstPtr;
using ImageConstPtr = sensor_msgs::ImageConstPtr;
using CompressedImageConstPtr = sensor_msgs::CompressedImageConstPtr;
using Time = ros::Time;

inline double stampToSeconds(const ros::Time &stamp) {
    return stamp.toSec();
}
#else
namespace sensor = sensor_msgs::msg;
namespace nav = nav_msgs::msg;
namespace geometry = geometry_msgs::msg;
using ImuConstPtr = sensor_msgs::msg::Imu::ConstSharedPtr;
using NavSatFixConstPtr = sensor_msgs::msg::NavSatFix::ConstSharedPtr;
using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;
using CompressedImageConstPtr = sensor_msgs::msg::CompressedImage::ConstSharedPtr;
using Time = rclcpp::Time;

inline double stampToSeconds(const builtin_interfaces::msg::Time &stamp) {
    return rclcpp::Time(stamp).seconds();
}
#endif

template <typename Publisher, typename Message>
void publish(Publisher &publisher, const Message &message) {
#if ROS_AVAILABLE == 1
    publisher.publish(message);
#else
    publisher->publish(message);
#endif
}

} // namespace gvins_ros

#endif // IC_GVINS_ROS_COMPAT_H
