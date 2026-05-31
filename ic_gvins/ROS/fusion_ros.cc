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

#include "fusion_ros.h"
#include "drawer_rviz.h"

#include "ic_gvins/common/angle.h"
#include "ic_gvins/common/gpstime.h"
#include "ic_gvins/common/imu_frame.h"
#include "ic_gvins/common/logging.h"
#include "ic_gvins/misc.h"
#include "ic_gvins/tracking/frame.h"

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <functional>
#include <memory>
#include <thread>

std::atomic<bool> isfinished{false};

void sigintHandler(int sig);
void checkStateThread(std::shared_ptr<FusionROS> fusion);

void FusionROS::setFinished() {
    if (gvins_ && gvins_->isRunning()) {
        gvins_->setFinished();
    }
}

void FusionROS::run() {
    auto node = std::make_shared<rclcpp::Node>("gvins_node");

    // message topic
    string imu_topic, gnss_topic, image_topic, livox_topic;
    node->declare_parameter<string>("imu_topic", "/imu0");
    node->declare_parameter<string>("gnss_topic", "/gnss0");
    node->declare_parameter<string>("image_topic", "/cam0");
    node->get_parameter("imu_topic", imu_topic);
    node->get_parameter("gnss_topic", gnss_topic);
    node->get_parameter("image_topic", image_topic);

    // GVINS parameter
    string configfile;
    node->declare_parameter<string>("configfile", "gvins.yaml");
    node->get_parameter("configfile", configfile);

    // Load configurations
    YAML::Node config;
    std::vector<double> vecdata;
    try {
        config = YAML::LoadFile(configfile);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return;
    }
    auto outputpath        = config["outputpath"].as<string>();
    auto is_make_outputdir = config["is_make_outputdir"].as<bool>();

    // Create the output directory
    boost::system::error_code filesystem_error;
    if (!boost::filesystem::is_directory(outputpath, filesystem_error)) {
        filesystem_error.clear();
        boost::filesystem::create_directories(outputpath, filesystem_error);
    }
    if (filesystem_error || !boost::filesystem::is_directory(outputpath)) {
        std::cout << "Failed to open outputpath" << std::endl;
        return;
    }

    if (is_make_outputdir) {
        absl::CivilSecond cs = absl::ToCivilSecond(absl::Now(), absl::LocalTimeZone());
        absl::StrAppendFormat(&outputpath, "/T%04d%02d%02d%02d%02d%02d", cs.year(), cs.month(), cs.day(), cs.hour(),
                              cs.minute(), cs.second());
        filesystem_error.clear();
        boost::filesystem::create_directories(outputpath, filesystem_error);
        if (filesystem_error || !boost::filesystem::is_directory(outputpath)) {
            std::cout << "Failed to open outputpath" << std::endl;
            return;
        }
    }

    // GNSS outage configurations
    isusegnssoutage_ = config["isusegnssoutage"].as<bool>();
    gnssoutagetime_  = config["gnssoutagetime"].as<double>();
    gnssthreshold_   = config["gnssthreshold"].as<double>();

    YAML::Node imu_config = config["imu"];
    if (config["imu_orientation"]) {
        imu_orientation_ = config["imu_orientation"].as<std::string>();
        LOGI << "IMU orientation: " << imu_orientation_;
    } else if (imu_config && imu_config["imu_orientation"]) {
        imu_orientation_ = imu_config["imu_orientation"].as<std::string>();
        LOGI << "IMU orientation: " << imu_orientation_;
    } else {
        LOGI << "IMU orientation: Default (FRD)";
    }

    // Glog output path
    FLAGS_log_dir = outputpath;

    // The GVINS object
    Drawer::Ptr drawer = std::make_shared<DrawerRviz>(node);
    gvins_             = std::make_shared<GVINS>(configfile, outputpath, drawer);

    // check is initialized
    if (!gvins_->isRunning()) {
        LOGE << "Fusion ROS terminate";
        return;
    }

    // subscribe message
    auto imu_qos = rclcpp::SensorDataQoS();
    imu_qos.keep_last(200);
    auto gnss_qos = rclcpp::SensorDataQoS();
    gnss_qos.keep_last(1);
    auto image_qos = rclcpp::SensorDataQoS();
    image_qos.keep_last(20);

    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, imu_qos, std::bind(&FusionROS::imuCallback, this, std::placeholders::_1));
    auto gnss_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
        gnss_topic, gnss_qos, std::bind(&FusionROS::gnssCallback, this, std::placeholders::_1));
    auto image_sub = node->create_subscription<sensor_msgs::msg::Image>(
        image_topic, image_qos, std::bind(&FusionROS::imageCallback, this, std::placeholders::_1));

    LOGI << "Waiting ROS message...";

    // enter message loopback
    rclcpp::spin(node);
}

void FusionROS::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imumsg) {
    imu_pre_ = imu_;

    // Time convertion
    double unixsecond = rclcpp::Time(imumsg->header.stamp).seconds();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    imu_.time = weeksec;
    // delta time
    imu_.dt = imu_.time - imu_pre_.time;

    // IMU measurements, converted to Front-Right-Down.
    const Matrix3d r_frd_raw = ImuFrame::rawToFrdRotation(imu_orientation_);
    const Vector3d angular_velocity(imumsg->angular_velocity.x, imumsg->angular_velocity.y,
                                    imumsg->angular_velocity.z);
    const Vector3d linear_acceleration(imumsg->linear_acceleration.x, imumsg->linear_acceleration.y,
                                       imumsg->linear_acceleration.z);
    imu_.dtheta = r_frd_raw * angular_velocity * imu_.dt;
    imu_.dvel   = r_frd_raw * linear_acceleration * imu_.dt;

    // Not ready
    if (imu_pre_.time == 0) {
        return;
    }

    imu_buffer_.push(imu_);
    while (!imu_buffer_.empty()) {
        auto imu = imu_buffer_.front();

        // Add new IMU to GVINS
        if (gvins_->addNewImu(imu)) {
            imu_buffer_.pop();
        } else {
            // Thread lock failed, try next time
            break;
        }
    }
}

void FusionROS::gnssCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr gnssmsg) {
    // Time convertion
    double unixsecond = rclcpp::Time(gnssmsg->header.stamp).seconds();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    gnss_.time = weeksec;

    gnss_.blh[0] = gnssmsg->latitude * D2R;
    gnss_.blh[1] = gnssmsg->longitude * D2R;
    gnss_.blh[2] = gnssmsg->altitude;
    gnss_.std[0] = sqrt(gnssmsg->position_covariance[4]); // N
    gnss_.std[1] = sqrt(gnssmsg->position_covariance[0]); // E
    gnss_.std[2] = sqrt(gnssmsg->position_covariance[8]); // D

    gnss_.isyawvalid = false;

    // Exception
    if ((gnss_.std[0] == 0) || (gnss_.std[1] == 0) || (gnss_.std[2] == 0)) {
        return;
    }

    // Remove bad GNSS
    bool isoutage = false;
    if ((gnss_.std[0] < gnssthreshold_) && (gnss_.std[1] < gnssthreshold_) && (gnss_.std[2] < gnssthreshold_)) {

        if (isusegnssoutage_ && (weeksec >= gnssoutagetime_)) {
            isoutage = true;
        }

        // add new GNSS to GVINS
        if (!isoutage) {
            gvins_->addNewGnss(gnss_);
        }
    }
}

void FusionROS::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr imagemsg) {
    Mat image;

    // Copy image data
    if (imagemsg->encoding == sensor_msgs::image_encodings::MONO8) {
        image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC1);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width);
    } else if (imagemsg->encoding == sensor_msgs::image_encodings::BGR8) {
        image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC3);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width * 3);
    }

    // Time convertion
    double unixsecond = rclcpp::Time(imagemsg->header.stamp).seconds();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    // Add new Image to GVINS
    frame_ = Frame::createFrame(weeksec, image);

    frame_buffer_.push(frame_);
    while (!frame_buffer_.empty()) {
        auto frame = frame_buffer_.front();
        if (gvins_->addNewFrame(frame)) {
            frame_buffer_.pop();
        } else {
            break;
        }
    }

    LOG_EVERY_N(INFO, 20) << "Raw data time " << Logging::doubleData(imu_.time) << ", "
                          << Logging::doubleData(gnss_.time) << ", " << Logging::doubleData(frame_->stamp());
}

void sigintHandler(int sig) {
    std::cout << "Terminate by Ctrl+C " << sig << std::endl;
    isfinished = true;
}

void checkStateThread(std::shared_ptr<FusionROS> fusion) {
    std::cout << "Check thread is started..." << std::endl;

    auto fusion_ptr = std::move(fusion);
    while (!isfinished) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Exit the GVINS thread
    fusion_ptr->setFinished();

    std::cout << "GVINS has been shutdown ..." << std::endl;

    // Shutdown ROS
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    std::cout << "ROS node has been shutdown ..." << std::endl;
}

int main(int argc, char *argv[]) {
    // Glog initialization
    Logging::initialization(argv, true, true);

    // ROS node
    rclcpp::init(argc, argv);

    // Register signal handler
    std::signal(SIGINT, sigintHandler);

    auto fusion = std::make_shared<FusionROS>();

    // Check thread
    std::thread check_thread(checkStateThread, fusion);

    std::cout << "Fusion process is started..." << std::endl;

    // Enter message loop
    fusion->run();

    isfinished = true;
    if (check_thread.joinable()) {
        check_thread.join();
    }

    return 0;
}
