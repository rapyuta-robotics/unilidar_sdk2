/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#pragma once

#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "tf2_ros/static_transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// SDK
#include "unitree_lidar_sdk_pcl.h"


using std::placeholders::_1;
using std::placeholders::_2;

class UnitreeLidarSDKNode : public rclcpp::Node
{
public:
    explicit UnitreeLidarSDKNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~UnitreeLidarSDKNode() {};

    void timer_callback();

protected:
    // ROS
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laser_scan_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // Unitree Lidar Reader
    UnitreeLidarReader *lsdk_;

    // Config params
    int work_mode_;
    int initialize_type_;
    int local_port_;
    std::string local_ip_;
    int lidar_port_;
    std::string lidar_ip_;
    std::string serial_port_;
    int baudrate_;
    
    int cloud_scan_num_;
    bool use_system_timestamp_;
    double range_min_;
    double range_max_;
    double range_offset_;
    double range_scale_;

    std::string cloud_frame_;
    std::string cloud_topic_;

    std::string imu_frame_;
    std::string imu_topic_;

    std::string laserscan_frame_;
    std::string laserscan_topic_;

    rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> &params);
    void onReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void applyRangeCorrection(pcl::PointCloud<PointType>::Ptr cloud);
};

///////////////////////////////////////////////////////////////////

UnitreeLidarSDKNode::UnitreeLidarSDKNode(const rclcpp::NodeOptions &options)
    : Node("unitree_lidar_ros2_node", options)
{
    // load config parameters
    declare_parameter<int>("initialize_type", 2);
    declare_parameter<int>("work_mode", 0);
    declare_parameter<bool>("use_system_timestamp", true);
    declare_parameter<double>("range_min", 0);
    declare_parameter<double>("range_max", 50);
    declare_parameter<double>("range_offset", 0.0);
    declare_parameter<double>("range_scale", 1.0);
    declare_parameter<int>("cloud_scan_num", 18);

    declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    declare_parameter<int>("baudrate", 4000000);

    declare_parameter<int>("lidar_port", 6101);
    declare_parameter<std::string>("lidar_ip", "192.168.1.2");
    declare_parameter<int>("local_port", 6201);
    declare_parameter<std::string>("local_ip", "192.168.1.62");

    declare_parameter<std::string>("cloud_frame", "unilidar_lidar");
    declare_parameter<std::string>("cloud_topic", "unilidar/cloud");

    declare_parameter<std::string>("imu_frame", "unilidar_imu");
    declare_parameter<std::string>("imu_topic", "unilidar/imu");

    declare_parameter<std::string>("laserscan_frame", "unilidar_lidar");
    declare_parameter<std::string>("laserscan_topic", "unilidar/laserscan");

    work_mode_ = get_parameter("work_mode").as_int();
    initialize_type_ = get_parameter("initialize_type").as_int();

    serial_port_ = get_parameter("serial_port").as_string();
    baudrate_ = get_parameter("baudrate").as_int();

    lidar_port_ = get_parameter("lidar_port").as_int();
    lidar_ip_ = get_parameter("lidar_ip").as_string();
    local_port_ = get_parameter("local_port").as_int();
    local_ip_ = get_parameter("local_ip").as_string();

    cloud_scan_num_ = get_parameter("cloud_scan_num").as_int();
    use_system_timestamp_ = get_parameter("use_system_timestamp").as_bool();
    range_max_ = get_parameter("range_max").as_double();
    range_min_ = get_parameter("range_min").as_double();
    range_offset_ = get_parameter("range_offset").as_double();
    range_scale_ = get_parameter("range_scale").as_double();

    cloud_frame_ = get_parameter("cloud_frame").as_string();
    cloud_topic_ = get_parameter("cloud_topic").as_string();
    
    imu_frame_ = get_parameter("imu_frame").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();

    laserscan_frame_ = get_parameter("laserscan_frame").as_string();
    laserscan_topic_ = get_parameter("laserscan_topic").as_string();

    // Initialize UnitreeLidarReader
    lsdk_ = createUnitreeLidarReader();

    std::cout << "initialize_type_ = " << initialize_type_ << std::endl;

    if (initialize_type_ == 1)
    {
        lsdk_->initializeSerial(serial_port_, baudrate_,
                                cloud_scan_num_, use_system_timestamp_, range_min_, range_max_);
    }
    else if (initialize_type_ == 2)
    {
        lsdk_->initializeUDP(lidar_port_, lidar_ip_, local_port_, local_ip_,
                             cloud_scan_num_, use_system_timestamp_, range_min_, range_max_);
    }
    else
    {
        std::cout << "initialize_type is not right! exit now ...\n";
        exit(0);
    }

    lsdk_->setLidarWorkMode(work_mode_);

    // ROS2
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 10);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    pub_laser_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(laserscan_topic_, 10);

    // TF tree: cloud_frame -> imu_frame (datasheet extrinsic, parallel axes)
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = cloud_frame_;
    transformStamped.child_frame_id = imu_frame_;
    transformStamped.transform.translation.x = -0.007698;
    transformStamped.transform.translation.y = -0.014655;
    transformStamped.transform.translation.z = 0.00667;
    transformStamped.transform.rotation.w = 1;
    static_broadcaster_->sendTransform(transformStamped);

    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&UnitreeLidarSDKNode::onSetParameters, this, _1));
    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/reset", std::bind(&UnitreeLidarSDKNode::onReset, this, _1, _2));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&UnitreeLidarSDKNode::timer_callback, this));
}

rcl_interfaces::msg::SetParametersResult UnitreeLidarSDKNode::onSetParameters(
    const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : params)
    {
        if (param.get_name() == "range_offset")
            range_offset_ = param.as_double();
        else if (param.get_name() == "range_scale")
            range_scale_ = param.as_double();
    }
    return result;
}

void UnitreeLidarSDKNode::onReset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Reset requested, shutting down");
    response->success = true;
    shutdown_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), []() { rclcpp::shutdown(); });
}

// Map r to r * range_scale + range_offset along each ray; drop points at or behind the origin
void UnitreeLidarSDKNode::applyRangeCorrection(pcl::PointCloud<PointType>::Ptr cloud)
{
    size_t n = 0;
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        PointType p = cloud->points[i];
        const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        if (r <= 0.0 || r * range_scale_ + range_offset_ <= 0.0)
            continue;
        const float s = range_scale_ + range_offset_ / r;
        p.x *= s;
        p.y *= s;
        p.z *= s;
        cloud->points[n++] = p;
    }
    cloud->points.resize(n);
    cloud->width = n;
    cloud->height = 1;
}

void UnitreeLidarSDKNode::timer_callback()
{
    int result = lsdk_->runParse();
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    if (result == LIDAR_IMU_DATA_PACKET_TYPE)
    {
        LidarImuData imu;
        if (lsdk_->getImuData(imu))
        {
            // publish imu message
            sensor_msgs::msg::Imu imuMsg;
            imuMsg.header.frame_id = imu_frame_;
            imuMsg.header.stamp = this->now();

            imuMsg.orientation.w = imu.quaternion[0];
            imuMsg.orientation.x = imu.quaternion[1];
            imuMsg.orientation.y = imu.quaternion[2];
            imuMsg.orientation.z = imu.quaternion[3];

            imuMsg.angular_velocity.x = imu.angular_velocity[0];
            imuMsg.angular_velocity.y = imu.angular_velocity[1];
            imuMsg.angular_velocity.z = imu.angular_velocity[2];

            imuMsg.linear_acceleration.x = imu.linear_acceleration[0];
            imuMsg.linear_acceleration.y = imu.linear_acceleration[1];
            imuMsg.linear_acceleration.z = imu.linear_acceleration[2];

            pub_imu_->publish(imuMsg);
        }
    }
    else if (result == LIDAR_POINT_DATA_PACKET_TYPE)
    {
        PointCloudUnitree cloud;
        if (lsdk_->getPointCloud(cloud))
        {
            transformUnitreeCloudToPCL(cloud, cloudOut);
            if (range_offset_ != 0.0 || range_scale_ != 1.0)
                applyRangeCorrection(cloudOut);

            rclcpp::Time timestamp(
                static_cast<int32_t>(cloud.stamp),
                static_cast<uint32_t>((cloud.stamp - static_cast<int32_t>(cloud.stamp)) * 1e9));

            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloudOut, cloud_msg);
            cloud_msg.header.frame_id = cloud_frame_;
            cloud_msg.header.stamp = timestamp;

            pub_cloud_->publish(cloud_msg);
        }
    }
    else if (result == LIDAR_2D_POINT_DATA_PACKET_TYPE)
    {
        const Lidar2DPointDataPacket &packet = lsdk_->getLidar2DPointDataPacket();
        const Lidar2DPointData &data = packet.data;

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = this->now();
        scan.header.frame_id = laserscan_frame_;
        scan.angle_min = data.angle_min;
        scan.angle_max = data.angle_min + data.angle_increment * data.point_num;
        scan.angle_increment = data.angle_increment;
        scan.time_increment = data.time_increment;
        scan.range_min = 0.0;
        scan.range_max = 100.0;
        scan.ranges.resize(data.point_num);
        scan.intensities.resize(data.point_num);

        for (unsigned int i = 0; i < data.point_num; ++i)
        {
            scan.ranges[i] = data.ranges[i] * data.param.range_scale;
            if ((range_offset_ != 0.0 || range_scale_ != 1.0) && scan.ranges[i] > 0.0f)
                scan.ranges[i] = std::max(0.0, scan.ranges[i] * range_scale_ + range_offset_);
            scan.intensities[i] = data.intensities[i];
        }

        pub_laser_scan_->publish(scan);
    }
}
