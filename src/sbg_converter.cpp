#include "sbg_converter.hpp"

SbgConverter::SbgConverter(const rclcpp::NodeOptions &options)
    : Node("sbg_converter", options) {
  this->declare_parameter("sbg_namespace", "/sbg");
  this->declare_parameter("ros_namespace", "/sbg");
  this->declare_parameter("output.use_enu", false);
  this->declare_parameter("output.frame_id", "");
  this->declare_parameter("publish_tf", true);
  this->declare_parameter("odometry.publishTf", true);
  this->declare_parameter("odometry.invertTf", true);
  this->declare_parameter("odometry.odomFrameId", "odom_ned");
  this->declare_parameter("odometry.baseFrameId", "base_link_ned");
  this->declare_parameter("odometry.initFrameId", "map");

  this->get_parameter("sbg_namespace", sbg_namespace_);
  this->get_parameter("ros_namespace", ros_namespace_);
  this->get_parameter("output.use_enu", enu_enable_);
  this->get_parameter("output.frame_id", frame_id_);
  this->get_parameter("odometry.publishTf", odom_publish_tf_);
  this->get_parameter("odometry.invertTf", odom_invert_tf_);
  this->get_parameter("odometry.odomFrameId", odom_frame_id_);
  this->get_parameter("odometry.baseFrameId", odom_base_frame_id_);
  this->get_parameter("odometry.initFrameId", odom_init_frame_id_);
  message_wrapper_.setOdomPublishTf(odom_publish_tf_);
  message_wrapper_.setOdomInvertTf(odom_invert_tf_);
  message_wrapper_.setOdomBaseFrameId(odom_base_frame_id_);
  message_wrapper_.setOdomFrameId(odom_frame_id_);
  message_wrapper_.setOdomInitFrameId(odom_init_frame_id_);

  std::string time_reference;
  this->get_parameter_or<std::string>("output.time_reference", time_reference, "ros");
  if (time_reference == "ros")
  {
    message_wrapper_.setTimeReference(sbg::TimeReference::ROS);
  }
  else if (time_reference == "ins_unix")
  {
    message_wrapper_.setTimeReference(sbg::TimeReference::INS_UNIX);
  }
  else
  {
    rclcpp::exceptions::throw_from_rcl_error(RMW_RET_ERROR, "unknown time reference: " + time_reference);
  }

  sbg_namespace_ = sbg_namespace_ + "/";
  ros_namespace_ = ros_namespace_ + "/";

  if (frame_id_.empty())
  {
    frame_id_ = std::string("imu") + (enu_enable_ ? "" : "_ned");
  }
  message_wrapper_.setFrameId(frame_id_);

  gps_navsat_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      ros_namespace_ + "raw_navsat" + (enu_enable_? "" : "_ned"), 10);
  ekf_pos_publisher_ = this->create_publisher<
      geographic_msgs::msg::GeoPoseWithCovarianceStamped>(
      ros_namespace_ + "ekf_geo_pose" + (enu_enable_? "" : "_ned"), 10);
  raw_pos_publisher_ = this->create_publisher<
      geographic_msgs::msg::GeoPoseWithCovarianceStamped>(
      ros_namespace_ + "raw_geo_pose" + (enu_enable_? "" : "_ned"), 10);
  ekf_navsat_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      ros_namespace_ + "ekf_navsat" + (enu_enable_? "" : "_ned"), 10);
  imu_pub_ =
      this->create_publisher<sensor_msgs::msg::Imu>(
        ros_namespace_ + "imu_data" + (enu_enable_ ? "" : "_ned"), 10);
  utc_reference_pub_ = this->create_publisher<sensor_msgs::msg::TimeReference>(
      ros_namespace_ + "time_reference", 10);
  temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>(
    ros_namespace_ + "temp", 10);
  mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
    ros_namespace_ + "mag", 10);
  velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    ros_namespace_ + "velocity" + (enu_enable_ ? "" : "_ned"), 10);
  odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    ros_namespace_ + "odometry" + (enu_enable_ ? "" : "_ned"), 10);
  ekf_pos_ecef_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    ros_namespace_ + "ekf_pos_ecef", 10);
  raw_pos_ecef_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    ros_namespace_ + "raw_pos_ecef", 10);


  ekf_nav_subscription_ = this->create_subscription<sbg_driver::msg::SbgEkfNav>(
      sbg_namespace_ + "ekf_nav", 10,
      std::bind(&SbgConverter::ekf_nav_cb, this, std::placeholders::_1));
  ekf_quat_subscription_ = this->create_subscription<sbg_driver::msg::SbgEkfQuat>(
      sbg_namespace_ + "ekf_quat", 10,
      std::bind(&SbgConverter::ekf_quat_cb, this, std::placeholders::_1));
  ekf_euler_subscription_ = this->create_subscription<sbg_driver::msg::SbgEkfEuler>(
      sbg_namespace_ + "ekf_euler", 10,
      std::bind(&SbgConverter::ekf_euler_cb, this, std::placeholders::_1));
  gps_pos_subscription_ = this->create_subscription<sbg_driver::msg::SbgGpsPos>(
      sbg_namespace_ + "gps_pos", 10,
      std::bind(&SbgConverter::gps_pos_cb, this, std::placeholders::_1));
  gps_hdt_subscription_ = this->create_subscription<sbg_driver::msg::SbgGpsHdt>(
      sbg_namespace_ + "gps_hdt", 10,
      std::bind(&SbgConverter::gps_hdt_cb, this, std::placeholders::_1));
  utc_time_subscription = this->create_subscription<sbg_driver::msg::SbgUtcTime>(
      sbg_namespace_ + "utc_time", 10,
      std::bind(&SbgConverter::utc_time_cb, this, std::placeholders::_1));
  imu_data_subscription = this->create_subscription<sbg_driver::msg::SbgImuData>(
      sbg_namespace_ + "imu_data", 10,
      std::bind(&SbgConverter::imu_data_cb, this, std::placeholders::_1));
  imu_mag_sub_ = this->create_subscription<sbg_driver::msg::SbgMag>(
      sbg_namespace_ + "mag", 10,
      std::bind(&SbgConverter::mag_cb, this, std::placeholders::_1));

  using namespace message_filters;
  using namespace std::placeholders;
  rclcpp::QoS qos(10);
  auto rmw_qos_profile = qos.get_rmw_qos_profile();

  // imu_mag_sub_.subscribe(this, sbg_namespace_ + "mag", rmw_qos_profile);
  imu_sub_.subscribe(this, sbg_namespace_ + "imu_data", rmw_qos_profile);
  sbg_ekf_quat_sub_.subscribe(this, sbg_namespace_ + "ekf_quat", rmw_qos_profile);
}

void SbgConverter::publish_ekf_geo_pose()
{
  if (gps_pos_)
  {
    auto navsat_msg = message_wrapper_.createRosNavSatFixMessage(gps_pos_);
    gps_navsat_publisher_->publish(navsat_msg);

    if (gps_pos_->status.status == 0)
    {
      auto raw_geo_pose_msg = geographic_msgs::msg::GeoPoseWithCovarianceStamped();
      raw_geo_pose_msg.header = navsat_msg.header;
      raw_geo_pose_msg.pose.pose.position.latitude = navsat_msg.latitude;
      raw_geo_pose_msg.pose.pose.position.longitude = navsat_msg.longitude;
      raw_geo_pose_msg.pose.pose.position.altitude = navsat_msg.altitude;
      raw_geo_pose_msg.pose.covariance[0] = pow(gps_pos_->position_accuracy.x, 2);
      raw_geo_pose_msg.pose.covariance[7] = pow(gps_pos_->position_accuracy.y, 2);
      raw_geo_pose_msg.pose.covariance[14] = pow(gps_pos_->position_accuracy.z, 2);
      raw_pos_publisher_->publish(raw_geo_pose_msg);
    }

    if (sbg_ekf_nav_message_ && sbg_ekf_nav_message_->status.position_valid)
    {
      auto ekf_navsat_msg =
          message_wrapper_.createRosNavSatFixMessage(sbg_ekf_nav_message_, gps_pos_);
      ekf_navsat_publisher_->publish(ekf_navsat_msg);
      if (sbg_ekf_nav_message_->status.heading_valid)
      {
        if (sbg_ekf_quat_message_)
        {
          auto ekf_geo_pose_msg = geographic_msgs::msg::GeoPoseWithCovarianceStamped();
          ekf_geo_pose_msg.header = navsat_msg.header;
          ekf_geo_pose_msg.pose.pose.position.latitude = ekf_navsat_msg.latitude;
          ekf_geo_pose_msg.pose.pose.position.longitude = ekf_navsat_msg.longitude;
          ekf_geo_pose_msg.pose.pose.position.altitude = ekf_navsat_msg.altitude;
          ekf_geo_pose_msg.pose.covariance[0] =
              pow(sbg_ekf_nav_message_->position_accuracy.x, 2);
          ekf_geo_pose_msg.pose.covariance[7] =
              pow(sbg_ekf_nav_message_->position_accuracy.y, 2);
          ekf_geo_pose_msg.pose.covariance[14] =
              pow(sbg_ekf_nav_message_->position_accuracy.z, 2);

          ekf_geo_pose_msg.pose.pose.orientation = sbg_ekf_quat_message_->quaternion;
          ekf_geo_pose_msg.pose.covariance[21] =
              pow(sbg_ekf_quat_message_->accuracy.x, 2);
          ekf_geo_pose_msg.pose.covariance[28] =
              pow(sbg_ekf_quat_message_->accuracy.y, 2);
          ekf_geo_pose_msg.pose.covariance[35] =
              pow(sbg_ekf_quat_message_->accuracy.z, 2);
          ekf_pos_publisher_->publish(ekf_geo_pose_msg);
        }
      }
    }
  }
}

void SbgConverter::ekf_nav_cb(const sbg_driver::msg::SbgEkfNav::ConstSharedPtr msg)
{
  sbg_ekf_nav_message_ = msg;
  publish_ekf_geo_pose();
  processRosOdoMessage();
  processRosVelMessage();
}

void SbgConverter::ekf_euler_cb(const sbg_driver::msg::SbgEkfEuler::ConstSharedPtr msg) {
  sbg_ekf_euler_message_ = msg;
    // processRosVelMessage();
    // processRosOdoMessage();
}

void SbgConverter::ekf_quat_cb(const sbg_driver::msg::SbgEkfQuat::ConstSharedPtr msg) {
  sbg_ekf_quat_message_ = msg;
  processRosImuMessage();
  // processRosVelMessage();
}

void SbgConverter::gps_pos_cb(const sbg_driver::msg::SbgGpsPos::ConstSharedPtr msg) {
  gps_pos_ = msg;
}

void SbgConverter::gps_hdt_cb(const sbg_driver::msg::SbgGpsHdt::ConstSharedPtr msg) {
  gps_hdt_ = msg;
}

void SbgConverter::mag_cb(const sbg_driver::msg::SbgMag::ConstSharedPtr msg) {
  mag_pub_->publish(message_wrapper_.createRosMagneticMessage(msg));
}

void SbgConverter::imu_cb(
    const sbg_driver::msg::SbgEkfQuat::ConstSharedPtr &quat,
    const sbg_driver::msg::SbgImuData::ConstSharedPtr &msg) {
  publish_ekf_geo_pose();
}

void SbgConverter::utc_time_cb(sbg_driver::msg::SbgUtcTime::ConstSharedPtr msg) {
  if (msg->clock_status.clock_utc_status != SBG_ECOM_UTC_STATUS_INVALID)
  {
    utc_reference_pub_->publish(message_wrapper_.createRosUtcTimeReferenceMessage(msg));
  }
}

void SbgConverter::imu_data_cb(sbg_driver::msg::SbgImuData::ConstSharedPtr msg) {
  if (temp_pub_)
  {
    temp_pub_->publish(message_wrapper_.createRosTemperatureMessage(msg));
  }
  sbg_imu_message_ = msg;
  processRosImuMessage();
  // processRosVelMessage();
  // processRosOdoMessage();
}


void SbgConverter::processRosVelMessage()
{
  if (velocity_pub_ && sbg_ekf_quat_message_ && sbg_ekf_nav_message_ && sbg_imu_message_)
  {
    velocity_pub_->publish(message_wrapper_.createRosTwistStampedMessage(
      sbg_ekf_quat_message_, sbg_ekf_nav_message_, sbg_imu_message_));
  }
}

void SbgConverter::processRosImuMessage()
{
  if (imu_pub_ && sbg_imu_message_ && sbg_ekf_quat_message_)
  {
    if (sbg_imu_message_->time_stamp == sbg_ekf_quat_message_->time_stamp)
    {
      imu_pub_->publish(message_wrapper_.createRosImuMessage(sbg_imu_message_, sbg_ekf_quat_message_));
    }
  }
}

void SbgConverter::processRosOdoMessage()
{
  if (odometry_pub_ && sbg_ekf_nav_message_ && sbg_imu_message_ && sbg_ekf_euler_message_ && sbg_ekf_quat_message_)
  {
    if (sbg_ekf_nav_message_->status.position_valid)
    {
      if (sbg_imu_message_->time_stamp == sbg_ekf_nav_message_->time_stamp)
      {
        /*
         * Odometry message can be generated from quaternion or euler angles.
         * Quaternion is prefered if they are available.
         */
        if (sbg_imu_message_->time_stamp == sbg_ekf_quat_message_->time_stamp)
        {
          odometry_pub_->publish(message_wrapper_.createRosOdoMessage(sbg_imu_message_, sbg_ekf_nav_message_, sbg_ekf_quat_message_, sbg_ekf_euler_message_));
        }
      }
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(SbgConverter)
