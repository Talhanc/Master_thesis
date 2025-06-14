#ifndef ESKF_ROS_HPP
#define ESKF_ROS_HPP

#include <chrono>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <stonefish_ros2/msg/dvl.hpp>
#include "eskf/eskf.hpp"
#include "eskf/typedefs.hpp"
#include "spdlog/spdlog.h"

class ESKFNode : public rclcpp::Node {
   public:
    explicit ESKFNode();

   private:
    
    // @brief Callback function for the imu topic
    // @param msg: Imu message containing the imu data
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // @brief Callback function for the dvl topic
    // @param msg: TwistWithCovarianceStamped message containing the dvl data
    void dvl_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

    // @brief Publish the odometry message
    void publish_odom();

    // @brief Set the subscriber and publisher for the node
    void set_subscribers_and_publisher();

    // @brief Set the parameters for the eskf
    void set_parameters();

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_pub_;

    std::chrono::milliseconds time_step;

    rclcpp::TimerBase::SharedPtr odom_pub_timer_;

    nominal_state nom_state_;

    error_state error_state_;

    imu_measurement imu_meas_;

    dvl_measurement dvl_meas_;

    eskf_params eskf_params_;

    std::unique_ptr<ESKF> eskf_;

    bool first_imu_msg_received_ = false;

    Eigen::Matrix3d R_imu_eskf_;


    // With this:
    rclcpp::Time last_imu_time_;

    struct TimingStats {
        double mean;
        double min;
        double max;
        int count;
    };
    TimingStats imu_timing_stats_;
    TimingStats dvl_timing_stats_;
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point end_time_;
    std::chrono::high_resolution_clock::time_point last_odom_pub_time_;
    std::chrono::high_resolution_clock::time_point last_dvl_time_;
    std::chrono::high_resolution_clock::time_point last_eskf_update_time_;
    std::chrono::high_resolution_clock::time_point last_eskf_dvl_update_time_;
    std::chrono::high_resolution_clock::time_point last_eskf_imu_update_time_;
    std::chrono::high_resolution_clock::time_point last_eskf_publish_time_;
    std::chrono::high_resolution_clock::time_point last_eskf_reset_time_;
    std::chrono::high_resolution_clock::time_point last_eskf_init_time_;
    std::chrono::high_resolution_clock::time_point last_eskf_dvl_reset_time_;
    std::chrono::high_resolution_clock::time_point last_eskf_imu_reset_time_;
    std::chrono::high_resolution_clock::time_point last_eskf_publish_odom_time_;            
};

#endif  // ESKF_ROS_HPP
