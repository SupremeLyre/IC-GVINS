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

#include "ROS/drawer_rviz.h"

#include "common/logging.h"
#include "common/rotation.h"

#if ROS_AVAILABLE == 1
DrawerRviz::DrawerRviz(ros::NodeHandle &node)
#else
DrawerRviz::DrawerRviz(const rclcpp::Node::SharedPtr &node)
#endif
    : isfinished_(false)
    , isframerdy_(false)
    , ismaprdy_(false)
#if ROS_AVAILABLE == 2
    , node_(node)
#endif
{

    frame_id_ = "world";
    body_frame_id_ = "gvins_body";
#if ROS_AVAILABLE == 1
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    pose_pub_ = node.advertise<nav_msgs::Odometry>("pose", 2);
    path_pub_ = node.advertise<nav_msgs::Path>("path", 2);
    track_image_pub_ = node.advertise<sensor_msgs::Image>("tracking", 2);
    fixed_points_pub_ = node.advertise<sensor_msgs::PointCloud>("fixed", 2);
    current_points_pub_ = node.advertise<sensor_msgs::PointCloud>("current", 2);
#else
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    pose_pub_           = node_->create_publisher<nav_msgs::msg::Odometry>("pose", rclcpp::QoS(2));
    path_pub_           = node_->create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(2));
    track_image_pub_    = node_->create_publisher<sensor_msgs::msg::Image>("tracking", rclcpp::QoS(2));
    fixed_points_pub_   = node_->create_publisher<sensor_msgs::msg::PointCloud>("fixed", rclcpp::QoS(2));
    current_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>("current", rclcpp::QoS(2));
#endif
}

gvins_ros::Time DrawerRviz::now() const {
#if ROS_AVAILABLE == 1
    return ros::Time::now();
#else
    return node_->now();
#endif
}

void DrawerRviz::setFinished() {
    isfinished_ = true;
    update_sem_.notify_one();
}

void DrawerRviz::run() {

    while (!isfinished_) {
        // 等待绘图更新信号
        std::unique_lock<std::mutex> lock(update_mutex_);
        update_sem_.wait(lock);

        // 发布跟踪的图像
        if (isframerdy_) {
            publishTrackingImage();

            isframerdy_ = false;
        }

        // 发布轨迹和地图点
        if (ismaprdy_) {
            publishOdometry();

            publishMapPoints();

            ismaprdy_ = false;
        }
    }
}

void DrawerRviz::updateFrame(Frame::Ptr frame, const Mat &undistorted_image) {
    std::unique_lock<std::mutex> lock(image_mutex_);

    if (!undistorted_image.empty()) {
        undistorted_image.copyTo(raw_image_);
    } else {
        frame->image().copyTo(raw_image_);
    }

    isframerdy_ = true;
    update_sem_.notify_one();
}

void DrawerRviz::updateTrackedMapPoints(vector<cv::Point2f> map, vector<cv::Point2f> matched,
                                        vector<MapPointType> mappoint_type) {
    std::unique_lock<std::mutex> lock(image_mutex_);
    pts2d_map_     = std::move(map);
    pts2d_matched_ = std::move(matched);
    mappoint_type_ = std::move(mappoint_type);
}

void DrawerRviz::updateTrackedRefPoints(vector<cv::Point2f> ref, vector<cv::Point2f> cur) {
    std::unique_lock<std::mutex> lock(image_mutex_);
    pts2d_ref_ = std::move(ref);
    pts2d_cur_ = std::move(cur);
}

void DrawerRviz::publishTrackingImage() {
    std::unique_lock<std::mutex> lock(image_mutex_);

    Mat drawed;
    drawTrackingImage(raw_image_, drawed);

    gvins_ros::sensor::Image image;

    image.header.stamp    = now();
    image.header.frame_id = frame_id_;
    image.encoding        = sensor_msgs::image_encodings::BGR8;
    image.height          = drawed.rows;
    image.width           = drawed.cols;

    size_t size = image.height * image.width * 3;
    image.step  = image.width * 3;
    image.data.resize(size);
    memcpy(image.data.data(), drawed.data, size);

    gvins_ros::publish(track_image_pub_, image);
}

void DrawerRviz::publishMapPoints() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    auto stamp = now();

    // 发布窗口内的路标点
    gvins_ros::sensor::PointCloud current_pointcloud;

    current_pointcloud.header.stamp    = stamp;
    current_pointcloud.header.frame_id = frame_id_;

    // 获取当前点云
    for (const auto &local : map_->landmarks()) {
        if (local.second && !local.second->isOutlier()) {
            gvins_ros::geometry::Point32 point;
            point.x = static_cast<float>(local.second->pos().x());
            point.y = static_cast<float>(local.second->pos().y());
            point.z = static_cast<float>(local.second->pos().z());

            current_pointcloud.points.push_back(point);
        }
    }
    gvins_ros::publish(current_points_pub_, current_pointcloud);

    // 发布新的地图点
    gvins_ros::sensor::PointCloud fixed_pointcloud;

    fixed_pointcloud.header.stamp    = stamp;
    fixed_pointcloud.header.frame_id = frame_id_;

    for (const auto &pts : fixed_mappoints_) {
        gvins_ros::geometry::Point32 point;
        point.x = static_cast<float>(pts.x());
        point.y = static_cast<float>(pts.y());
        point.z = static_cast<float>(pts.z());

        fixed_pointcloud.points.push_back(point);
    }
    gvins_ros::publish(fixed_points_pub_, fixed_pointcloud);

    fixed_mappoints_.clear();
}

void DrawerRviz::publishOdometry() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    gvins_ros::nav::Odometry odometry;

    auto quaternion = Rotation::matrix2quaternion(pose_.R);
    auto stamp      = now();

    // Odometry
    odometry.header.stamp            = stamp;
    odometry.header.frame_id         = frame_id_;
    odometry.child_frame_id          = body_frame_id_;
    odometry.pose.pose.position.x    = pose_.t.x();
    odometry.pose.pose.position.y    = pose_.t.y();
    odometry.pose.pose.position.z    = pose_.t.z();
    odometry.pose.pose.orientation.x = quaternion.x();
    odometry.pose.pose.orientation.y = quaternion.y();
    odometry.pose.pose.orientation.z = quaternion.z();
    odometry.pose.pose.orientation.w = quaternion.w();
    gvins_ros::publish(pose_pub_, odometry);

    gvins_ros::geometry::TransformStamped transform;
    transform.header.stamp            = stamp;
    transform.header.frame_id         = frame_id_;
    transform.child_frame_id          = body_frame_id_;
    transform.transform.translation.x = pose_.t.x();
    transform.transform.translation.y = pose_.t.y();
    transform.transform.translation.z = pose_.t.z();
    transform.transform.rotation      = odometry.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);

    // Path
    gvins_ros::geometry::PoseStamped pose_stamped;
    pose_stamped.header.stamp    = stamp;
    pose_stamped.header.frame_id = frame_id_;
    pose_stamped.pose            = odometry.pose.pose;

    path_.header.stamp    = stamp;
    path_.header.frame_id = frame_id_;
    path_.poses.push_back(pose_stamped);
    gvins_ros::publish(path_pub_, path_);
}

void DrawerRviz::addNewFixedMappoint(Vector3d point) {
    std::unique_lock<std::mutex> lock(map_mutex_);

    fixed_mappoints_.push_back(point);
}

void DrawerRviz::updateMap(const Eigen::Matrix4d &pose) {
    std::unique_lock<std::mutex> lock(map_mutex_);

    pose_.R = pose.block<3, 3>(0, 0);
    pose_.t = pose.block<3, 1>(0, 3);

    ismaprdy_ = true;
    update_sem_.notify_one();
}
