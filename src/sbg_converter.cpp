#include "sbg_converter.hpp"

SbgConverter::SbgConverter(const rclcpp::NodeOptions &options)
    : Node("sbg_converter", options) {
  this->declare_parameter("sbg_namespace", "sbg");
  this->declare_parameter("imu_pub_topic", "/imu/data");

  this->get_parameter("sbg_namespace", sbg_namespace_);
  this->get_parameter("imu_pub_topic", imu_pub_topic_);

  sbg_namespace_ = sbg_namespace_ + "/";

  gps_pos_publisher_ = this->create_publisher<
      geographic_msgs::msg::GeoPoseWithCovarianceStamped>(
      sbg_namespace_ + "raw_geo_pose_ned", 10);
  gps_navsat_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      sbg_namespace_ + "raw_navsat_ned", 10);
  ekf_pos_publisher_ = this->create_publisher<
      geographic_msgs::msg::GeoPoseWithCovarianceStamped>(
      sbg_namespace_ + "ekf_geo_pose_ned", 10);
  ekf_navsat_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      sbg_namespace_ + "ekf_navsat_ned", 10);
  imu_publisher_ =
      this->create_publisher<sensor_msgs::msg::Imu>(imu_pub_topic_, 10);

  ekf_nav_subscription_ = this->create_subscription<sbg_driver::msg::SbgEkfNav>(
      sbg_namespace_ + "ekf_nav", 10,
      std::bind(&SbgConverter::ekf_nav_cb, this, std::placeholders::_1));
  gps_pos_subscription_ = this->create_subscription<sbg_driver::msg::SbgGpsPos>(
      sbg_namespace_ + "gps_pos", 10,
      std::bind(&SbgConverter::gps_pos_cb, this, std::placeholders::_1));
  gps_hdt_subscription_ = this->create_subscription<sbg_driver::msg::SbgGpsHdt>(
      sbg_namespace_ + "gps_hdt", 10,
      std::bind(&SbgConverter::gps_hdt_cb, this, std::placeholders::_1));

  using namespace message_filters;
  using namespace std::placeholders;
  rclcpp::QoS qos(10);
  auto rmw_qos_profile = qos.get_rmw_qos_profile();

  imu_mag_sub_.subscribe(this, sbg_namespace_ + "mag", rmw_qos_profile);
  imu_sub_.subscribe(this, sbg_namespace_ + "imu_data", rmw_qos_profile);
  sbg_ekf_quat_sub_.subscribe(this, sbg_namespace_ + "ekf_quat",
                              rmw_qos_profile);

  sync_ = std::make_shared<message_filters::TimeSynchronizer<
      sbg_driver::msg::SbgEkfQuat, sbg_driver::msg::SbgImuData>>(
      sbg_ekf_quat_sub_, imu_sub_, 10);
  sync_->registerCallback(std::bind(&SbgConverter::imu_cb, this, _1, _2));

  publish_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&SbgConverter::publish_geo_pose, this));
}

void SbgConverter::publish_geo_pose() {
  if (!ekf_quat_ || !ekf_nav_ || !gps_pos_) {
    return;
  }

  auto geo_pose_msg = geographic_msgs::msg::GeoPoseWithCovarianceStamped();
  geo_pose_msg.header = ekf_nav_->header;
  geo_pose_msg.header.frame_id = "asv4/sbg/imu";
  geo_pose_msg.pose.pose.position.latitude = gps_pos_->latitude;
  geo_pose_msg.pose.pose.position.longitude = gps_pos_->longitude;
  geo_pose_msg.pose.pose.position.altitude = gps_pos_->altitude;

  if (gps_hdt_) {
    tf2::Quaternion q;
    q.setRPY(gps_hdt_->true_heading * M_PI / 180, gps_hdt_->pitch * M_PI / 180,
             0);
    geo_pose_msg.pose.pose.orientation = tf2::toMsg(q);
    geo_pose_msg.pose.covariance[21] = 0;
    geo_pose_msg.pose.covariance[28] = gps_hdt_->pitch_acc;
    geo_pose_msg.pose.covariance[35] = gps_hdt_->true_heading_acc;
  }

  geo_pose_msg.pose.covariance[0] = gps_pos_->position_accuracy.x;
  geo_pose_msg.pose.covariance[7] = gps_pos_->position_accuracy.y;
  geo_pose_msg.pose.covariance[14] = gps_pos_->position_accuracy.z;
  gps_pos_publisher_->publish(geo_pose_msg);

  auto navsat_msg = sensor_msgs::msg::NavSatFix();
  navsat_msg.header = ekf_nav_->header;
  navsat_msg.header.frame_id = "sbg_raw_ned";
  navsat_msg.latitude = gps_pos_->latitude;
  navsat_msg.longitude = gps_pos_->longitude;
  navsat_msg.altitude = gps_pos_->altitude;
  navsat_msg.position_covariance[0] = gps_pos_->position_accuracy.x;
  navsat_msg.position_covariance[4] = gps_pos_->position_accuracy.y;
  navsat_msg.position_covariance[8] = gps_pos_->position_accuracy.z;
  navsat_msg.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  gps_navsat_publisher_->publish(navsat_msg);

  auto ekf_geo_pose_msg = geographic_msgs::msg::GeoPoseWithCovarianceStamped();
  ekf_geo_pose_msg.header = ekf_nav_->header;
  ekf_geo_pose_msg.header.frame_id = "sbg_ekf_ned";
  ekf_geo_pose_msg.pose.pose.position.latitude = ekf_nav_->latitude;
  ekf_geo_pose_msg.pose.pose.position.longitude = ekf_nav_->longitude;
  ekf_geo_pose_msg.pose.pose.position.altitude = ekf_nav_->altitude;
  ekf_geo_pose_msg.pose.pose.orientation = ekf_quat_->quaternion;
  ekf_geo_pose_msg.pose.covariance[0] = ekf_nav_->position_accuracy.x;
  ekf_geo_pose_msg.pose.covariance[7] = ekf_nav_->position_accuracy.y;
  ekf_geo_pose_msg.pose.covariance[14] = ekf_nav_->position_accuracy.z;
  ekf_geo_pose_msg.pose.covariance[21] = ekf_quat_->accuracy.x;
  ekf_geo_pose_msg.pose.covariance[28] = ekf_quat_->accuracy.y;
  ekf_geo_pose_msg.pose.covariance[35] = ekf_quat_->accuracy.z;
  ekf_pos_publisher_->publish(ekf_geo_pose_msg);

  auto ekf_navsat_msg = sensor_msgs::msg::NavSatFix();
  ekf_navsat_msg.header = ekf_nav_->header;
  ekf_navsat_msg.header.frame_id = "sbg_ekf_ned";
  ekf_navsat_msg.latitude = ekf_nav_->latitude;
  ekf_navsat_msg.longitude = ekf_nav_->longitude;
  ekf_navsat_msg.altitude = ekf_nav_->altitude;
  ekf_navsat_msg.position_covariance[0] = ekf_nav_->position_accuracy.x;
  ekf_navsat_msg.position_covariance[4] = ekf_nav_->position_accuracy.y;
  ekf_navsat_msg.position_covariance[8] = ekf_nav_->position_accuracy.z;
  ekf_navsat_msg.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  ekf_navsat_publisher_->publish(ekf_navsat_msg);
}

void SbgConverter::ekf_nav_cb(const sbg_driver::msg::SbgEkfNav::SharedPtr msg) {
  ekf_nav_ = msg;
}

void SbgConverter::gps_pos_cb(const sbg_driver::msg::SbgGpsPos::SharedPtr msg) {
  gps_pos_ = msg;
}

void SbgConverter::gps_hdt_cb(const sbg_driver::msg::SbgGpsHdt::SharedPtr msg) {
  gps_hdt_ = msg;
}

void SbgConverter::imu_cb(
    const sbg_driver::msg::SbgEkfQuat::ConstSharedPtr &quat,
    const sbg_driver::msg::SbgImuData::ConstSharedPtr &msg) {
  publish_geo_pose();
}

RCLCPP_COMPONENTS_REGISTER_NODE(SbgConverter)