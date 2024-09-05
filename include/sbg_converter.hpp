#ifndef SBG_CONVERTER_HPP_
#define SBG_CONVERTER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <geographic_msgs/msg/geo_pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <sbg_driver/msg/sbg_ekf_nav.hpp>
#include <sbg_driver/msg/sbg_ekf_quat.hpp>
#include <sbg_driver/msg/sbg_gps_hdt.hpp>
#include <sbg_driver/msg/sbg_gps_pos.hpp>
#include <sbg_driver/msg/sbg_imu_data.hpp>
#include <sbg_driver/msg/sbg_mag.hpp>

#include "custom_message_wrapper.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Core>
#include "rclcpp_components/register_node_macro.hpp"

class SbgConverter : public rclcpp::Node
{
public:
  SbgConverter(const rclcpp::NodeOptions& options);

private:
  void publish_ekf_geo_pose();
  void ekf_nav_cb(const sbg_driver::msg::SbgEkfNav::ConstSharedPtr msg);
  void ekf_quat_cb(const sbg_driver::msg::SbgEkfQuat::ConstSharedPtr msg);
  void ekf_euler_cb(const sbg_driver::msg::SbgEkfEuler::ConstSharedPtr msg);
  void gps_pos_cb(const sbg_driver::msg::SbgGpsPos::ConstSharedPtr msg);
  void gps_hdt_cb(const sbg_driver::msg::SbgGpsHdt::ConstSharedPtr msg);
  void imu_cb(
      const sbg_driver::msg::SbgEkfQuat::ConstSharedPtr& quat,
      const sbg_driver::msg::SbgImuData::ConstSharedPtr& msg);
  void utc_time_cb(const sbg_driver::msg::SbgUtcTime::ConstSharedPtr msg);
  void imu_data_cb(const sbg_driver::msg::SbgImuData::ConstSharedPtr msg);
  void mag_cb(const sbg_driver::msg::SbgMag::ConstSharedPtr msg);

  /*!
    * Process a ROS Velocity standard message.
    */
  void processRosVelMessage();

  /*!
    * Process a ROS IMU standard message.
    */
  void processRosImuMessage();

  /*!
    * Process a ROS odometry standard message.
    */
  void processRosOdoMessage();

  rclcpp::Publisher<geographic_msgs::msg::GeoPoseWithCovarianceStamped>::SharedPtr
      gps_pos_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_navsat_publisher_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPoseWithCovarianceStamped>::SharedPtr
      ekf_pos_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr ekf_navsat_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr utc_reference_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ekf_pos_ecef_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr raw_pos_ecef_pub_;
  rclcpp::Subscription<sbg_driver::msg::SbgEkfNav>::SharedPtr ekf_nav_subscription_;
  rclcpp::Subscription<sbg_driver::msg::SbgEkfQuat>::SharedPtr ekf_quat_subscription_;
  rclcpp::Subscription<sbg_driver::msg::SbgEkfEuler>::SharedPtr ekf_euler_subscription_;
  rclcpp::Subscription<sbg_driver::msg::SbgGpsPos>::SharedPtr gps_pos_subscription_;
  rclcpp::Subscription<sbg_driver::msg::SbgGpsHdt>::SharedPtr gps_hdt_subscription_;
  rclcpp::Subscription<sbg_driver::msg::SbgUtcTime>::SharedPtr utc_time_subscription;
  rclcpp::Subscription<sbg_driver::msg::SbgImuData>::SharedPtr imu_data_subscription;
  rclcpp::Subscription<sbg_driver::msg::SbgMag>::SharedPtr imu_mag_sub_;
  // message_filters::Subscriber<sbg_driver::msg::SbgMag> imu_mag_sub_;
  message_filters::Subscriber<sbg_driver::msg::SbgImuData> imu_sub_;
  message_filters::Subscriber<sbg_driver::msg::SbgEkfQuat> sbg_ekf_quat_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<
      sbg_driver::msg::SbgEkfQuat, sbg_driver::msg::SbgImuData>>
      sync_;
  std::shared_ptr<rclcpp::TimerBase> publish_timer_;
  sbg_driver::msg::SbgGpsPos::ConstSharedPtr gps_pos_;
  sbg_driver::msg::SbgGpsHdt::ConstSharedPtr gps_hdt_;
  std::string sbg_namespace_, ros_namespace_;
  std::string imu_pub_topic_;
  std::string frame_id_;
  sbg::CustomMessageWrapper message_wrapper_;
  sbg_driver::msg::SbgImuData::ConstSharedPtr sbg_imu_message_;
  sbg_driver::msg::SbgEkfQuat::ConstSharedPtr sbg_ekf_quat_message_;
  sbg_driver::msg::SbgEkfNav::ConstSharedPtr sbg_ekf_nav_message_;
  sbg_driver::msg::SbgEkfEuler::ConstSharedPtr sbg_ekf_euler_message_;
  bool enu_enable_, odom_publish_tf_, odom_invert_tf_;
  std::string odom_frame_id_, odom_base_frame_id_, odom_init_frame_id_;
};

#endif   // SBG_CONVERTER_HPP_
