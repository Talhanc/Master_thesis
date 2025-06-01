#include "eskf/eskf_ros.hpp"
#include <spdlog/spdlog.h>
#include "eskf/eskf_utils.hpp"
#include "eskf/typedefs.hpp"

ESKFNode::ESKFNode() : Node("eskf_node") {
    time_step = std::chrono::milliseconds(1);
    odom_pub_timer_ = this->create_wall_timer(
        time_step, std::bind(&ESKFNode::publish_odom, this));

    set_subscribers_and_publisher();

    set_parameters();

    spdlog::info("ESKF Node Initialized");
}

void ESKFNode::set_subscribers_and_publisher() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    this->declare_parameter<std::string>("imu_topic");
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos_sensor_data,
        std::bind(&ESKFNode::imu_callback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("dvl_topic");
    std::string dvl_topic = this->get_parameter("dvl_topic").as_string();
    dvl_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        dvl_topic, qos_sensor_data,
        std::bind(&ESKFNode::dvl_callback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("odom_topic");
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic, qos_sensor_data);

    nis_pub_ = create_publisher<std_msgs::msg::Float64>("dvl/nis", 10);
}

void ESKFNode::set_parameters() {
    std::vector<double> R_imu_correction;
    this->declare_parameter<std::vector<double>>("imu_frame");
    R_imu_correction = get_parameter("imu_frame").as_double_array();
    R_imu_eskf_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        R_imu_correction.data());

    std::vector<double> diag_Q_std;
    this->declare_parameter<std::vector<double>>("diag_Q_std");

    diag_Q_std = this->get_parameter("diag_Q_std").as_double_array();

    Eigen::Matrix12d Q;
    Q.setZero();
    Q.diagonal() << sq(diag_Q_std[0]), sq(diag_Q_std[1]), sq(diag_Q_std[2]),
        sq(diag_Q_std[3]), sq(diag_Q_std[4]), sq(diag_Q_std[5]),
        sq(diag_Q_std[6]), sq(diag_Q_std[7]), sq(diag_Q_std[8]),
        sq(diag_Q_std[9]), sq(diag_Q_std[10]), sq(diag_Q_std[11]);
    eskf_params_.Q = Q;

    eskf_ = std::make_unique<ESKF>(eskf_params_);

    std::vector<double> diag_p_init =
        this->declare_parameter<std::vector<double>>("diag_p_init");
    Eigen::Matrix18d P = createDiagonalMatrix<18>(diag_p_init);

    error_state_.covariance = P;
}

void ESKFNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    rclcpp::Time current_time = msg->header.stamp;

    if (!first_imu_msg_received_) {
        last_imu_time_ = current_time;
        first_imu_msg_received_ = true;
        imu_timing_stats_ = {0.0, std::numeric_limits<double>::max(), 0.0, 0}; // mean, min, max, count
        return;
    }

    double dt = (current_time - last_imu_time_).nanoseconds() * 1e-9;
    last_imu_time_ = current_time;

    Eigen::Vector3d raw_accel(msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z);

    imu_meas_.accel = R_imu_eskf_ * raw_accel;

    Eigen::Vector3d raw_gyro(msg->angular_velocity.x, msg->angular_velocity.y,
                             msg->angular_velocity.z);

    imu_meas_.gyro = R_imu_eskf_ * raw_gyro;

    // Time the IMU update
    auto start_time = std::chrono::high_resolution_clock::now();
    std::tie(nom_state_, error_state_) = eskf_->imu_update(imu_meas_, dt);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    // Compute execution time in milliseconds
    double execution_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    // Update statistics
    imu_timing_stats_.count++;
    imu_timing_stats_.mean = imu_timing_stats_.mean + 
                            (execution_time_ms - imu_timing_stats_.mean) / imu_timing_stats_.count;
    imu_timing_stats_.min = std::min(imu_timing_stats_.min, execution_time_ms);
    imu_timing_stats_.max = std::max(imu_timing_stats_.max, execution_time_ms);
    
    // Log timing stats every 100 IMU updates
    if (imu_timing_stats_.count % 100 == 0) {
        spdlog::info("IMU update timing (ms) - Mean: {:.3f}, Min: {:.3f}, Max: {:.3f}, Count: {}",
                    imu_timing_stats_.mean, imu_timing_stats_.min, 
                    imu_timing_stats_.max, imu_timing_stats_.count);
    }
}

void ESKFNode::dvl_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    // Initialize timing stats on first call
    if (!dvl_timing_stats_.count) {
        dvl_timing_stats_ = {0.0, std::numeric_limits<double>::max(), 0.0, 0}; // mean, min, max, count
    }
    
    dvl_meas_.vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;

    dvl_meas_.cov << msg->twist.covariance[0], msg->twist.covariance[1], msg->twist.covariance[2],
        msg->twist.covariance[6], msg->twist.covariance[7], msg->twist.covariance[8],
        msg->twist.covariance[12], msg->twist.covariance[13], msg->twist.covariance[14];

    // Time the DVL update
    auto start_time = std::chrono::high_resolution_clock::now();
    std::tie(nom_state_, error_state_) = eskf_->dvl_update(dvl_meas_);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    // Compute execution time in milliseconds
    double execution_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    // Update statistics
    dvl_timing_stats_.count++;
    dvl_timing_stats_.mean = dvl_timing_stats_.mean + 
                           (execution_time_ms - dvl_timing_stats_.mean) / dvl_timing_stats_.count;
    dvl_timing_stats_.min = std::min(dvl_timing_stats_.min, execution_time_ms);
    dvl_timing_stats_.max = std::max(dvl_timing_stats_.max, execution_time_ms);
    
    // Log timing stats every 10 DVL updates (since DVL typically comes at a slower rate than IMU)
    if (dvl_timing_stats_.count % 10 == 0) {
        spdlog::info("DVL update timing (ms) - Mean: {:.3f}, Min: {:.3f}, Max: {:.3f}, Count: {}",
                    dvl_timing_stats_.mean, dvl_timing_stats_.min, 
                    dvl_timing_stats_.max, dvl_timing_stats_.count);
    }

    std_msgs::msg::Float64 nis_msg;
    nis_msg.data = eskf_->NIS_;
    nis_pub_->publish(nis_msg);
}

void ESKFNode::publish_odom() {
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.pose.pose.position.x = nom_state_.pos.x();
    odom_msg.pose.pose.position.y = nom_state_.pos.y();
    odom_msg.pose.pose.position.z = nom_state_.pos.z();

    odom_msg.pose.pose.orientation.w = nom_state_.quat.w();
    odom_msg.pose.pose.orientation.x = nom_state_.quat.x();
    odom_msg.pose.pose.orientation.y = nom_state_.quat.y();
    odom_msg.pose.pose.orientation.z = nom_state_.quat.z();

    odom_msg.twist.twist.linear.x = nom_state_.vel.x();
    odom_msg.twist.twist.linear.y = nom_state_.vel.y();
    odom_msg.twist.twist.linear.z = nom_state_.vel.z();

    // Add bias values to the angular velocity field of twist
    odom_msg.twist.twist.angular.x = nom_state_.accel_bias.x();
    odom_msg.twist.twist.angular.y = nom_state_.accel_bias.y();
    odom_msg.twist.twist.angular.z = nom_state_.accel_bias.z();

    // If you also want to include gyro bias, you could add it to the covariance
    // matrix or publish a separate topic for biases

    odom_msg.header.stamp = this->now();
    odom_pub_->publish(odom_msg);
}
