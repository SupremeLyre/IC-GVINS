/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DRAWER_RVIZ_H
#define DRAWER_RVIZ_H

#include "tracking/drawer.h"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_ros/transform_broadcaster.h>

using std::string;
using std::vector;

class DrawerRviz : public Drawer {

public:
    explicit DrawerRviz(const rclcpp::Node::SharedPtr &node);

    void run() override;

    void setFinished() override;

    // 地图
    void addNewFixedMappoint(Vector3d point) override;

    void updateMap(const Eigen::Matrix4d &pose) override;

    // 跟踪图像
    void updateFrame(Frame::Ptr frame, const Mat &undistorted_image = Mat()) override;
    void updateTrackedMapPoints(vector<cv::Point2f> map, vector<cv::Point2f> matched,
                                vector<MapPointType> mappoint_type) override;
    void updateTrackedRefPoints(vector<cv::Point2f> ref, vector<cv::Point2f> cur) override;

private:
    void publishTrackingImage();

    void publishMapPoints();

    void publishOdometry();

private:
    // 多线程
    std::condition_variable update_sem_;
    std::mutex update_mutex_;
    std::mutex map_mutex_;
    std::mutex image_mutex_;

    // 标志
    std::atomic<bool> isfinished_;
    std::atomic<bool> isframerdy_;
    std::atomic<bool> ismaprdy_;

    // 跟踪
    Mat raw_image_;
    vector<Vector3d> fixed_mappoints_;

    Pose pose_;
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    nav_msgs::msg::Path path_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr track_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr fixed_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr current_points_pub_;

    string frame_id_;
    string body_frame_id_;
};

#endif // DRAWER_RVIZ_H
