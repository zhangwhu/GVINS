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
#include "ic_gvins/common/logging.h"
#include "ic_gvins/misc.h"
#include "ic_gvins/tracking/frame.h"

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <sensor_msgs/image_encodings.h>

#include <atomic>
#include <csignal>
#include <memory>

std::atomic<bool> isfinished{false};

void sigintHandler(int sig);
void checkStateThread(std::shared_ptr<FusionROS> fusion);

void FusionROS::setFinished() {
    if (gvins_ && gvins_->isRunning()) {
        gvins_->setFinished();
    }
}

void FusionROS::run() {
    ros::NodeHandle nh;                 //全局命名空间
    ros::NodeHandle pnh("~");

    // message topic
    string imu_topic, gnss_topic, image_topic, livox_topic;
    pnh.param<string>("imu_topic", imu_topic, "/imu0");             
    pnh.param<string>("gnss_topic", gnss_topic, "/gnss0");
    pnh.param<string>("image_topic", image_topic, "/image0");

    // GVINS parameter
    string configfile;
    pnh.param<string>("configfile", configfile, "gvins.yaml");

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
    if (!boost::filesystem::is_directory(outputpath)) {
        boost::filesystem::create_directory(outputpath);
    }
    if (!boost::filesystem::is_directory(outputpath)) {
        std::cout << "Failed to open outputpath" << std::endl;
        return;
    }

    if (is_make_outputdir) {
        absl::CivilSecond cs = absl::ToCivilSecond(absl::Now(), absl::LocalTimeZone());
        absl::StrAppendFormat(&outputpath, "/T%04d%02d%02d%02d%02d%02d", cs.year(), cs.month(), cs.day(), cs.hour(),
                              cs.minute(), cs.second());
        boost::filesystem::create_directory(outputpath);
    }

    // GNSS outage configurations
    isusegnssoutage_ = config["isusegnssoutage"].as<bool>();
    gnssoutagetime_  = config["gnssoutagetime"].as<double>();
    gnssthreshold_   = config["gnssthreshold"].as<double>();

    // Glog output path
    FLAGS_log_dir = outputpath;

    // The GVINS object
    Drawer::Ptr drawer = std::make_shared<DrawerRviz>(nh);
    gvins_             = std::make_shared<GVINS>(configfile, outputpath, drawer);           

    // check is initialized
    if (!gvins_->isRunning()) {             
        LOGE << "Fusion ROS terminate";
        return;
    }

    // subscribe message
    ros::Subscriber imu_sub   = nh.subscribe<sensor_msgs::Imu>(imu_topic, 100, &FusionROS::imuCallback, this);  
    ros::Subscriber gnss_sub  = nh.subscribe<gnss_comm::GnssMeasMsg>(gnss_topic, 1, &FusionROS::gnssCallback, this);
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>(image_topic, 10, &FusionROS::imageCallback, this);

    LOGI << "Waiting ROS message...";

    // enter message loopback
    ros::spin();
}

void FusionROS::imuCallback(const sensor_msgs::ImuConstPtr &imumsg) {
    imu_pre_ = imu_;

    // Time convertion
    double unixsecond = imumsg->header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);
    
    imu_.time = weeksec;
    // delta time
    imu_.dt = imu_.time - imu_pre_.time;

    // IMU measurements, Front-Right-Down (FRD) rad-m/s^2
    imu_.dtheta[0] = imumsg->angular_velocity.x * imu_.dt;
    imu_.dtheta[1] = imumsg->angular_velocity.y * imu_.dt;
    imu_.dtheta[2] = imumsg->angular_velocity.z * imu_.dt;
    imu_.dvel[0]   = imumsg->linear_acceleration.x * imu_.dt;
    imu_.dvel[1]   = imumsg->linear_acceleration.y * imu_.dt;
    imu_.dvel[2]   = imumsg->linear_acceleration.z * imu_.dt;

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

void FusionROS::gnssCallback(const gnss_comm::GnssMeasMsgConstPtr &gnssmsg) { 
    // Time convertion
    double unixsecond = gnssmsg->header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);
    
    // Add new GNSS to GVINS
    obss_.time  = weeksec; 
    obss_.node  = 0;
    obss_.state = GOBSS::Null;    
    obss_.meas  = msg2meas(gnssmsg);

    obs_buffer_.push(obss_);
    while (!obs_buffer_.empty()) {
        auto obss_ = obs_buffer_.front();
        if (gvins_->addNewObss(obss_)) {                         
            obs_buffer_.pop();
        } else {
            break;
        }
    }
}

void FusionROS::imageCallback(const sensor_msgs::ImageConstPtr &imagemsg) {
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
    double unixsecond = imagemsg->header.stamp.toSec();
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
                          << Logging::doubleData(frame_->stamp()) << ",  " << frame_buffer_.size() << ",  "
                          << Logging::doubleData(obss_.time) << ",  " << obs_buffer_.size();
}

void sigintHandler(int sig) {
    std::cout << "Terminate by Ctrl+C " << sig << std::endl;
    isfinished = true;
}

void checkStateThread(std::shared_ptr<FusionROS> fusion) {
    std::cout << "Check thread is started..." << std::endl;

    auto fusion_ptr = std::move(fusion);
    while (!isfinished) {
        sleep(1);
    }

    // Exit the GVINS thread
    fusion_ptr->setFinished();

    std::cout << "GVINS has been shutdown ..." << std::endl;

    // Shutdown ROS
    ros::shutdown();

    std::cout << "ROS node has been shutdown ..." << std::endl;
}

int main(int argc, char *argv[]) {
    // Glog initialization
    Logging::initialization(argv, true, true);              

    // ROS node
    ros::init(argc, argv, "gvins_node", ros::init_options::NoSigintHandler);

    // Register signal handler
    std::signal(SIGINT, sigintHandler);       

    auto fusion = std::make_shared<FusionROS>();

    // Check thread
    std::thread check_thread(checkStateThread, fusion);                        

    std::cout << "Fusion process is started..." << std::endl;

    // Enter message loop
    fusion->run();

    // Release check thread
    check_thread.join();

    return 0;
}
