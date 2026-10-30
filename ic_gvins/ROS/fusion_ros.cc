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
#include <opencv2/imgcodecs.hpp>

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
#if ROS_AVAILABLE == 1
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
#else
    auto node = std::make_shared<rclcpp::Node>("gvins_node");
#endif

    // GVINS parameter
    string configfile;
#if ROS_AVAILABLE == 1
    private_node.param<string>("configfile", configfile, "gvins.yaml");
#else
    node->declare_parameter<string>("configfile", "gvins.yaml");
    node->get_parameter("configfile", configfile);
#endif

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

    // message topic
    string imu_topic   = "/imu0";
    string gnss_topic  = "/gnss0";
    string image_topic = "/cam0";
    if (config["ros"]) {
        if (config["ros"]["imu_topic"]) {
            imu_topic = config["ros"]["imu_topic"].as<string>();
        }
        if (config["ros"]["gnss_topic"]) {
            gnss_topic = config["ros"]["gnss_topic"].as<string>();
        }
        if (config["ros"]["image_topic"]) {
            image_topic = config["ros"]["image_topic"].as<string>();
        }
    }
#if ROS_AVAILABLE == 1
    private_node.param<string>("imu_topic", imu_topic, imu_topic);
    private_node.param<string>("gnss_topic", gnss_topic, gnss_topic);
    private_node.param<string>("image_topic", image_topic, image_topic);
#else
    node->declare_parameter<string>("imu_topic", imu_topic);
    node->declare_parameter<string>("gnss_topic", gnss_topic);
    node->declare_parameter<string>("image_topic", image_topic);
    node->get_parameter("imu_topic", imu_topic);
    node->get_parameter("gnss_topic", gnss_topic);
    node->get_parameter("image_topic", image_topic);
#endif

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
    if (config["ros"] && config["ros"]["use_compressed_image"]) {
        use_compressed_image_ = config["ros"]["use_compressed_image"].as<bool>();
    }

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
#if ROS_AVAILABLE == 1
    auto imu_sub = node.subscribe<sensor_msgs::Imu>(imu_topic, 200, &FusionROS::imuCallback, this);
    auto gnss_sub = node.subscribe<sensor_msgs::NavSatFix>(gnss_topic, 1, &FusionROS::gnssCallback, this);
    ros::Subscriber image_sub;
    if (use_compressed_image_) {
        image_sub = node.subscribe<sensor_msgs::CompressedImage>(
            image_topic, 20, &FusionROS::compressedImageCallback, this);
    } else {
        image_sub = node.subscribe<sensor_msgs::Image>(image_topic, 20, &FusionROS::imageCallback, this);
    }
#else
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
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub;
    if (use_compressed_image_) {
        compressed_image_sub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
            image_topic, image_qos, std::bind(&FusionROS::compressedImageCallback, this, std::placeholders::_1));
    } else {
        image_sub = node->create_subscription<sensor_msgs::msg::Image>(
            image_topic, image_qos, std::bind(&FusionROS::imageCallback, this, std::placeholders::_1));
    }
#endif

    LOGI << "Waiting ROS message...";

    // enter message loopback
#if ROS_AVAILABLE == 1
    ros::spin();
#else
    rclcpp::spin(node);
#endif
}

void FusionROS::imuCallback(const gvins_ros::ImuConstPtr &imumsg) {
    imu_pre_ = imu_;

    // Time convertion
    double unixsecond = gvins_ros::stampToSeconds(imumsg->header.stamp);
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    imu_.time     = weeksec;
    imu_.gps_week = week;
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

void FusionROS::gnssCallback(const gvins_ros::NavSatFixConstPtr &gnssmsg) {
    // Time convertion
    double unixsecond = gvins_ros::stampToSeconds(gnssmsg->header.stamp);
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

namespace {

bool addImageFrameToGvins(std::shared_ptr<GVINS> &gvins, std::queue<Frame::Ptr> &frame_buffer, Frame::Ptr &last_frame,
                          const Mat &image, double unixsecond) {
    if (image.empty()) {
        LOGE << "Empty image message";
        return false;
    }

    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    last_frame = Frame::createFrame(weeksec, image);

    frame_buffer.push(last_frame);
    while (!frame_buffer.empty()) {
        auto frame = frame_buffer.front();
        if (gvins->addNewFrame(frame)) {
            frame_buffer.pop();
        } else {
            break;
        }
    }
    return true;
}

} // namespace

void FusionROS::imageCallback(const gvins_ros::ImageConstPtr &imagemsg) {
    Mat image;

    // Copy image data
    if (imagemsg->encoding == sensor_msgs::image_encodings::MONO8) {
        image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC1);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width);
    } else if ((imagemsg->encoding == sensor_msgs::image_encodings::BGR8) ||
               (imagemsg->encoding == sensor_msgs::image_encodings::RGB8)) {
        image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC3);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width * 3);
        if (imagemsg->encoding == sensor_msgs::image_encodings::RGB8) {
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        }
    } else {
        LOGE << "Unsupported image encoding " << imagemsg->encoding;
        return;
    }

    double unixsecond = gvins_ros::stampToSeconds(imagemsg->header.stamp);
    if (!addImageFrameToGvins(gvins_, frame_buffer_, frame_, image, unixsecond)) {
        return;
    }

    LOG_EVERY_N(INFO, 20) << "Raw data time " << Logging::doubleData(imu_.time) << ", "
                          << Logging::doubleData(gnss_.time) << ", " << Logging::doubleData(frame_->stamp());
}

void FusionROS::compressedImageCallback(const gvins_ros::CompressedImageConstPtr &imagemsg) {
    Mat encoded(1, static_cast<int>(imagemsg->data.size()), CV_8UC1, const_cast<uint8_t *>(imagemsg->data.data()));
    Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);

    double unixsecond = gvins_ros::stampToSeconds(imagemsg->header.stamp);
    if (!addImageFrameToGvins(gvins_, frame_buffer_, frame_, image, unixsecond)) {
        return;
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
#if ROS_AVAILABLE == 1
    ros::shutdown();
#else
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
#endif

    std::cout << "ROS node has been shutdown ..." << std::endl;
}

int main(int argc, char *argv[]) {
    // Glog initialization
    Logging::initialization(argv, true, true);

    // ROS node
#if ROS_AVAILABLE == 1
    ros::init(argc, argv, "gvins_node", ros::init_options::NoSigintHandler);
#else
    rclcpp::init(argc, argv);
#endif

    // Register signal handler
    std::signal(SIGINT, sigintHandler);
    std::signal(SIGTERM, sigintHandler);

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
