// File header
#include "custom_message_wrapper.h"

// ROS headers
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// Project headers
#include <sbg_vector3.h>
#include <sbg_ros_helpers.h>

// STL headers
#include <type_traits>

using sbg::CustomMessageWrapper;

/*!
 * Class to wrap the SBG logs into ROS messages.
 */
//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

CustomMessageWrapper::CustomMessageWrapper():
Node("custom_tf_broadcaster")
{
  first_valid_utc_ = false;
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

//---------------------------------------------------------------------//
//- Internal methods                                                  -//
//---------------------------------------------------------------------//

const std_msgs::msg::Header CustomMessageWrapper::createRosHeader(uint32_t device_timestamp) const
{
  std_msgs::msg::Header header;

  header.frame_id = frame_id_;

  if (first_valid_utc_ && (time_reference_ == TimeReference::INS_UNIX))
  {
    header.stamp = convertInsTimeToUnix(device_timestamp);
  }
  else
  {
    header.stamp = rclcpp::Clock().now();
  }

  return header;
}

const rclcpp::Time CustomMessageWrapper::convertInsTimeToUnix(uint32_t device_timestamp) const
{
  //
  // Convert the UTC time to epoch from the last received message.
  // Add the SBG timestamp difference (timestamp is in microsecond).
  //
  rclcpp::Time utc_to_epoch;
  uint32_t  device_timestamp_diff;
  uint64_t  nanoseconds;

  utc_to_epoch          = convertUtcTimeToUnix(last_sbg_utc_);
  device_timestamp_diff = device_timestamp - last_sbg_utc_->time_stamp;
  nanoseconds = utc_to_epoch.nanoseconds() + static_cast<uint64_t>(device_timestamp_diff) * 1000;
  utc_to_epoch = rclcpp::Time(nanoseconds);
  return utc_to_epoch;
}

const rclcpp::Time CustomMessageWrapper::convertUtcTimeToUnix(std::shared_ptr<const sbg_driver::msg::SbgUtcTime> ref_sbg_utc_msg) const
{
  rclcpp::Time utc_to_epoch;
  uint32_t  days;
  uint64_t  nanoseconds;

  //
  // Convert the UTC time to Epoch(Unix) time, which is the elapsed seconds since 1 Jan 1970.
  //
  days        = 0;
  nanoseconds = 0;

  for (uint16_t yearIndex = 1970; yearIndex < ref_sbg_utc_msg->year; yearIndex++)
  {
    days += sbg::helpers::getNumberOfDaysInYear(yearIndex);
  }

  for (uint8_t monthIndex = 1; monthIndex < ref_sbg_utc_msg->month; monthIndex++)
  {
    days += sbg::helpers::getNumberOfDaysInMonth(ref_sbg_utc_msg->year, monthIndex);
  }

  days += ref_sbg_utc_msg->day - 1;
  nanoseconds = days * 24;
  nanoseconds = (nanoseconds + ref_sbg_utc_msg->hour) * 60;
  nanoseconds = (nanoseconds + ref_sbg_utc_msg->min) * 60;
  nanoseconds = nanoseconds + ref_sbg_utc_msg->sec;
  nanoseconds = nanoseconds * 1000000000 + ref_sbg_utc_msg->nanosec;

  utc_to_epoch = rclcpp::Time(nanoseconds);

  return utc_to_epoch;
}

const sbg_driver::msg::SbgEkfStatus CustomMessageWrapper::createEkfStatusMessage(uint32_t ekf_status) const
{
  sbg_driver::msg::SbgEkfStatus ekf_status_message;

  ekf_status_message.solution_mode      = sbgEComLogEkfGetSolutionMode(ekf_status);
  ekf_status_message.attitude_valid     = (ekf_status & SBG_ECOM_SOL_ATTITUDE_VALID) != 0;
  ekf_status_message.heading_valid      = (ekf_status & SBG_ECOM_SOL_HEADING_VALID) != 0;
  ekf_status_message.velocity_valid     = (ekf_status & SBG_ECOM_SOL_VELOCITY_VALID) != 0;
  ekf_status_message.position_valid     = (ekf_status & SBG_ECOM_SOL_POSITION_VALID) != 0;

  ekf_status_message.vert_ref_used      = (ekf_status & SBG_ECOM_SOL_VERT_REF_USED) != 0;
  ekf_status_message.mag_ref_used       = (ekf_status & SBG_ECOM_SOL_MAG_REF_USED) != 0;

  ekf_status_message.gps1_vel_used      = (ekf_status & SBG_ECOM_SOL_GPS1_VEL_USED) != 0;
  ekf_status_message.gps1_pos_used      = (ekf_status & SBG_ECOM_SOL_GPS1_POS_USED) != 0;
  ekf_status_message.gps1_hdt_used      = (ekf_status & SBG_ECOM_SOL_GPS1_HDT_USED) != 0;

  ekf_status_message.gps2_vel_used      = (ekf_status & SBG_ECOM_SOL_GPS2_VEL_USED) != 0;
  ekf_status_message.gps2_pos_used      = (ekf_status & SBG_ECOM_SOL_GPS2_POS_USED) != 0;
  ekf_status_message.gps2_hdt_used      = (ekf_status & SBG_ECOM_SOL_GPS2_HDT_USED) != 0;

  ekf_status_message.odo_used           = (ekf_status & SBG_ECOM_SOL_ODO_USED) != 0;

  ekf_status_message.dvl_bt_used        = (ekf_status & SBG_ECOM_SOL_DVL_BT_USED) != 0;
  ekf_status_message.dvl_wt_used        = (ekf_status & SBG_ECOM_SOL_DVL_WT_USED) != 0;

  ekf_status_message.user_pos_used      = (ekf_status & SBG_ECOM_SOL_USER_POS_USED) != 0;
  ekf_status_message.user_vel_used      = (ekf_status & SBG_ECOM_SOL_USER_VEL_USED) != 0;
  ekf_status_message.user_heading_used  = (ekf_status & SBG_ECOM_SOL_USER_HEADING_USED) != 0;

  ekf_status_message.usbl_used          = (ekf_status & SBG_ECOM_SOL_USBL_USED) != 0;

  ekf_status_message.air_data_used      = (ekf_status & SBG_ECOM_SOL_AIR_DATA_USED) != 0;

  ekf_status_message.zupt_used          = (ekf_status & SBG_ECOM_SOL_ZUPT_USED) != 0;

  ekf_status_message.align_used         = (ekf_status & SBG_ECOM_SOL_ALIGN_VALID) != 0;

  ekf_status_message.depth_used         = (ekf_status & SBG_ECOM_SOL_DEPTH_USED) != 0;

  return ekf_status_message;
}

//---------------------------------------------------------------------//
//- Parameters                                                        -//
//---------------------------------------------------------------------//

void CustomMessageWrapper::setTimeReference(TimeReference time_reference)
{
  time_reference_ = time_reference;
}

void CustomMessageWrapper::setFrameId(const std::string &frame_id)
{
  frame_id_ = frame_id;
}

void CustomMessageWrapper::setUseEnu(bool enu)
{
  use_enu_ = enu;
}

void CustomMessageWrapper::setOdomEnable(bool odom_enable)
{
  odom_enable_ = odom_enable;
}

void CustomMessageWrapper::setOdomPublishTf(bool publish_tf)
{
  odom_publish_tf_ = publish_tf;
}

void CustomMessageWrapper::setOdomInvertTf(bool invert_tf)
{
  odom_invert_tf_ = invert_tf;
}

void CustomMessageWrapper::setOdomFrameId(const std::string &ref_frame_id)
{
  odom_frame_id_ = ref_frame_id;
}

void CustomMessageWrapper::setOdomBaseFrameId(const std::string &ref_frame_id)
{
  odom_base_frame_id_ = ref_frame_id;
}

void CustomMessageWrapper::setOdomInitFrameId(const std::string &ref_frame_id)
{
  odom_init_frame_id_ = ref_frame_id;
}

//---------------------------------------------------------------------//
//- Operations                                                        -//
//---------------------------------------------------------------------//

const sensor_msgs::msg::Imu CustomMessageWrapper::createRosImuMessage(sbg_driver::msg::SbgImuData::ConstSharedPtr ref_sbg_imu_msg, sbg_driver::msg::SbgEkfQuat::ConstSharedPtr ref_sbg_quat_msg) const
{
  sensor_msgs::msg::Imu imu_ros_message;

  imu_ros_message.header = createRosHeader(ref_sbg_imu_msg->time_stamp);

  imu_ros_message.orientation               = ref_sbg_quat_msg->quaternion;
  imu_ros_message.angular_velocity          = ref_sbg_imu_msg->delta_angle;
  imu_ros_message.linear_acceleration       = ref_sbg_imu_msg->delta_vel;

  imu_ros_message.orientation_covariance[0] = pow(ref_sbg_quat_msg->accuracy.x, 2);
  imu_ros_message.orientation_covariance[4] = pow(ref_sbg_quat_msg->accuracy.y, 2);
  imu_ros_message.orientation_covariance[8] = pow(ref_sbg_quat_msg->accuracy.z, 2);

  //
  // Angular velocity and linear acceleration covariances are not provided.
  //
  for (size_t i = 0; i < 9; i++)
  {
    imu_ros_message.angular_velocity_covariance[i]    = 0.0;
    imu_ros_message.linear_acceleration_covariance[i] = 0.0;
  }

  return imu_ros_message;
}

void CustomMessageWrapper::fillTransform(const std::string &ref_parent_frame_id, const std::string &ref_child_frame_id, const geometry_msgs::msg::Pose &ref_pose, geometry_msgs::msg::TransformStamped &refTransformStamped)
{
  refTransformStamped.header.stamp = rclcpp::Clock().now();
  refTransformStamped.header.frame_id = ref_parent_frame_id;
  refTransformStamped.child_frame_id = ref_child_frame_id;

  refTransformStamped.transform.translation.x = ref_pose.position.x;
  refTransformStamped.transform.translation.y = ref_pose.position.y;
  refTransformStamped.transform.translation.z = ref_pose.position.z;
  refTransformStamped.transform.rotation.x = ref_pose.orientation.x;
  refTransformStamped.transform.rotation.y = ref_pose.orientation.y;
  refTransformStamped.transform.rotation.z = ref_pose.orientation.z;
  refTransformStamped.transform.rotation.w = ref_pose.orientation.w;
}

const nav_msgs::msg::Odometry CustomMessageWrapper::createRosOdoMessage(
  sbg_driver::msg::SbgImuData::ConstSharedPtr ref_sbg_imu_msg, sbg_driver::msg::SbgEkfNav::ConstSharedPtr ref_ekf_nav_msg, sbg_driver::msg::SbgEkfQuat::ConstSharedPtr ref_ekf_quat_msg, sbg_driver::msg::SbgEkfEuler::ConstSharedPtr ref_ekf_euler_msg)
{
  tf2::Quaternion orientation(ref_ekf_quat_msg->quaternion.x, ref_ekf_quat_msg->quaternion.y, ref_ekf_quat_msg->quaternion.z, ref_ekf_quat_msg->quaternion.w);

  return createRosOdoMessage(ref_sbg_imu_msg, ref_ekf_nav_msg, orientation, ref_ekf_euler_msg);
}

const nav_msgs::msg::Odometry CustomMessageWrapper::createRosOdoMessage(
  sbg_driver::msg::SbgImuData::ConstSharedPtr ref_sbg_imu_msg,
  sbg_driver::msg::SbgEkfNav::ConstSharedPtr ref_ekf_nav_msg,
  sbg_driver::msg::SbgEkfEuler::ConstSharedPtr ref_ekf_euler_msg)
{
  tf2::Quaternion orientation;

  // Compute orientation quaternion from euler angles (already converted from NED to ENU if needed).
  orientation.setRPY(ref_ekf_euler_msg->angle.x, ref_ekf_euler_msg->angle.y, ref_ekf_euler_msg->angle.z);

  return createRosOdoMessage(ref_sbg_imu_msg, ref_ekf_nav_msg, orientation, ref_ekf_euler_msg);
}

const nav_msgs::msg::Odometry CustomMessageWrapper::createRosOdoMessage(
  sbg_driver::msg::SbgImuData::ConstSharedPtr ref_sbg_imu_msg, sbg_driver::msg::SbgEkfNav::ConstSharedPtr ref_ekf_nav_msg, const tf2::Quaternion &ref_orientation, sbg_driver::msg::SbgEkfEuler::ConstSharedPtr ref_ekf_euler_msg)
{
  nav_msgs::msg::Odometry odo_ros_msg;
  std::string utm_zone;
  geometry_msgs::msg::TransformStamped transform;

  // The pose message provides the position and orientation of the robot relative to the frame specified in header.frame_id
  odo_ros_msg.header = createRosHeader(ref_sbg_imu_msg->time_stamp);
  odo_ros_msg.header.frame_id = odom_frame_id_;
  tf2::convert(ref_orientation, odo_ros_msg.pose.pose.orientation);

  // Convert latitude and longitude to UTM coordinates.
  if (!utm_.isInit())
  {
    utm_.init(ref_ekf_nav_msg->latitude, ref_ekf_nav_msg->longitude);
    const auto first_valid_easting_northing = utm_.computeEastingNorthing(ref_ekf_nav_msg->latitude, ref_ekf_nav_msg->longitude);
    first_valid_easting_ = first_valid_easting_northing[0];
    first_valid_northing_ = first_valid_easting_northing[1];
    first_valid_altitude_ = ref_ekf_nav_msg->altitude;

    RCLCPP_INFO(rclcpp::get_logger("Message wrapper"), "initialized from lat:%f long:%f UTM zone %d%c: easting:%fm (%dkm) northing:%fm (%dkm)"
    , ref_ekf_nav_msg->latitude, ref_ekf_nav_msg->longitude, utm_.getZoneNumber(), utm_.getLetterDesignator()
    , first_valid_easting_, (int)(first_valid_easting_) / 1000
    , first_valid_northing_, (int)(first_valid_northing_) / 1000
    );

    if (odom_publish_tf_)
    {
      // Publish UTM initial transformation.
      geometry_msgs::msg::Pose pose;
      pose.position.x = first_valid_easting_;
      pose.position.y = first_valid_northing_;
      pose.position.z = first_valid_altitude_;

      fillTransform(odom_init_frame_id_, odom_frame_id_, pose, transform);
      if (odom_invert_tf_) {
        transform = invertTransform(transform);
      }
      tf_broadcaster_->sendTransform(transform);
      static_tf_broadcaster_->sendTransform(transform);
    }
  }

  const auto easting_northing = utm_.computeEastingNorthing(ref_ekf_nav_msg->latitude, ref_ekf_nav_msg->longitude);
  odo_ros_msg.pose.pose.position.x = easting_northing[0] - first_valid_easting_;
  odo_ros_msg.pose.pose.position.y = easting_northing[1] - first_valid_northing_;
  odo_ros_msg.pose.pose.position.z = ref_ekf_nav_msg->altitude - first_valid_altitude_;

  // Compute convergence angle.
  double longitudeRad      = sbgDegToRadd(ref_ekf_nav_msg->longitude);
  double latitudeRad       = sbgDegToRadd(ref_ekf_nav_msg->latitude);
  double central_meridian  = sbgDegToRadd(utm_.getMeridian());
  double convergence_angle = atan(tan(longitudeRad - central_meridian) * sin(latitudeRad));

  // Convert position standard deviations to UTM frame.
  double std_east  = ref_ekf_nav_msg->position_accuracy.x;
  double std_north = ref_ekf_nav_msg->position_accuracy.y;
  double std_x = std_north * cos(convergence_angle) - std_east * sin(convergence_angle);
  double std_y = std_north * sin(convergence_angle) + std_east * cos(convergence_angle);
  double std_z = ref_ekf_nav_msg->position_accuracy.z;
  odo_ros_msg.pose.covariance[0*6 + 0] = std_x * std_x;
  odo_ros_msg.pose.covariance[1*6 + 1] = std_y * std_y;
  odo_ros_msg.pose.covariance[2*6 + 2] = std_z * std_z;
  odo_ros_msg.pose.covariance[3*6 + 3] = pow(ref_ekf_euler_msg->accuracy.x, 2);
  odo_ros_msg.pose.covariance[4*6 + 4] = pow(ref_ekf_euler_msg->accuracy.y, 2);
  odo_ros_msg.pose.covariance[5*6 + 5] = pow(ref_ekf_euler_msg->accuracy.z, 2);

  // The twist message gives the linear and angular velocity relative to the frame defined in child_frame_id
  odo_ros_msg.child_frame_id            = frame_id_;
  odo_ros_msg.twist.twist.linear.x      = ref_ekf_nav_msg->velocity.x;
  odo_ros_msg.twist.twist.linear.y      = ref_ekf_nav_msg->velocity.y;
  odo_ros_msg.twist.twist.linear.z      = ref_ekf_nav_msg->velocity.z;
  odo_ros_msg.twist.twist.angular.x     = ref_sbg_imu_msg->gyro.x;
  odo_ros_msg.twist.twist.angular.y     = ref_sbg_imu_msg->gyro.y;
  odo_ros_msg.twist.twist.angular.z     = ref_sbg_imu_msg->gyro.z;
  odo_ros_msg.twist.covariance[0*6 + 0] = pow(ref_ekf_nav_msg->velocity_accuracy.x, 2);
  odo_ros_msg.twist.covariance[1*6 + 1] = pow(ref_ekf_nav_msg->velocity_accuracy.y, 2);
  odo_ros_msg.twist.covariance[2*6 + 2] = pow(ref_ekf_nav_msg->velocity_accuracy.z, 2);
  odo_ros_msg.twist.covariance[3*6 + 3] = 0;
  odo_ros_msg.twist.covariance[4*6 + 4] = 0;
  odo_ros_msg.twist.covariance[5*6 + 5] = 0;

  if (odom_publish_tf_)
  {
    // Publish odom transformation.
    fillTransform(odo_ros_msg.header.frame_id, odom_base_frame_id_, odo_ros_msg.pose.pose, transform);
    if (odom_invert_tf_) {
      transform = invertTransform(transform);
    }
    tf_broadcaster_->sendTransform(transform);
  }

  return odo_ros_msg;
}

const sensor_msgs::msg::Temperature CustomMessageWrapper::createRosTemperatureMessage(sbg_driver::msg::SbgImuData::ConstSharedPtr ref_sbg_imu_msg) const
{
  sensor_msgs::msg::Temperature temperature_message;

  temperature_message.header      = createRosHeader(ref_sbg_imu_msg->time_stamp);
  temperature_message.temperature = ref_sbg_imu_msg->temp;
  temperature_message.variance    = 0.0;

  return temperature_message;
}

const sensor_msgs::msg::MagneticField CustomMessageWrapper::createRosMagneticMessage(sbg_driver::msg::SbgMag::ConstSharedPtr ref_sbg_mag_msg) const
{
  sensor_msgs::msg::MagneticField magnetic_message;

  magnetic_message.header         = createRosHeader(ref_sbg_mag_msg->time_stamp);
  magnetic_message.magnetic_field = ref_sbg_mag_msg->mag;

  return magnetic_message;
}

const geometry_msgs::msg::TwistStamped CustomMessageWrapper::createRosTwistStampedMessage(sbg_driver::msg::SbgEkfEuler::ConstSharedPtr ref_sbg_ekf_euler_msg, sbg_driver::msg::SbgEkfNav::ConstSharedPtr ref_sbg_ekf_nav_msg, sbg_driver::msg::SbgImuData::ConstSharedPtr ref_sbg_imu_msg) const
{
  sbg::SbgMatrix3f tdcm;
  tdcm.makeDcm(sbg::SbgVector3f(ref_sbg_ekf_euler_msg->angle.x, ref_sbg_ekf_euler_msg->angle.y, ref_sbg_ekf_euler_msg->angle.z));
  tdcm.transpose();

  const sbg::SbgVector3f res = tdcm * sbg::SbgVector3f(ref_sbg_ekf_nav_msg->velocity.x, ref_sbg_ekf_nav_msg->velocity.y, ref_sbg_ekf_nav_msg->velocity.z);

  return createRosTwistStampedMessage(res, ref_sbg_imu_msg);
}

const geometry_msgs::msg::TwistStamped CustomMessageWrapper::createRosTwistStampedMessage(
  sbg_driver::msg::SbgEkfQuat::ConstSharedPtr ref_sbg_ekf_quat_msg,
  sbg_driver::msg::SbgEkfNav::ConstSharedPtr ref_sbg_ekf_nav_msg,
  sbg_driver::msg::SbgImuData::ConstSharedPtr ref_sbg_imu_msg) const
{
  sbg::SbgMatrix3f tdcm;
  tdcm.makeDcm(ref_sbg_ekf_quat_msg->quaternion.w, ref_sbg_ekf_quat_msg->quaternion.x, ref_sbg_ekf_quat_msg->quaternion.y, ref_sbg_ekf_quat_msg->quaternion.z);
  tdcm.transpose();

  const sbg::SbgVector3f res = tdcm * sbg::SbgVector3f(ref_sbg_ekf_nav_msg->velocity.x, ref_sbg_ekf_nav_msg->velocity.y, ref_sbg_ekf_nav_msg->velocity.z);
  return createRosTwistStampedMessage(res, ref_sbg_imu_msg);
}

const geometry_msgs::msg::TwistStamped CustomMessageWrapper::createRosTwistStampedMessage(const sbg::SbgVector3f& body_vel, sbg_driver::msg::SbgImuData::ConstSharedPtr ref_sbg_imu_msg) const
{
  geometry_msgs::msg::TwistStamped twist_stamped_message;

  twist_stamped_message.header        = createRosHeader(ref_sbg_imu_msg->time_stamp);
  twist_stamped_message.twist.angular = ref_sbg_imu_msg->delta_angle;

  twist_stamped_message.twist.linear.x = body_vel(0);
  twist_stamped_message.twist.linear.y = body_vel(1);
  twist_stamped_message.twist.linear.z = body_vel(2);

  return twist_stamped_message;
}

const geometry_msgs::msg::PointStamped CustomMessageWrapper::createRosPointStampedMessage(const sbg_driver::msg::SbgEkfNav& ref_sbg_ekf_msg) const
{
  geometry_msgs::msg::PointStamped point_stamped_message;

  point_stamped_message.header = createRosHeader(ref_sbg_ekf_msg.time_stamp);

  const auto ecef_coordinates = helpers::convertLLAtoECEF(ref_sbg_ekf_msg.latitude, ref_sbg_ekf_msg.longitude, ref_sbg_ekf_msg.altitude);
  point_stamped_message.point.x = ecef_coordinates(0);
  point_stamped_message.point.y = ecef_coordinates(1);
  point_stamped_message.point.z = ecef_coordinates(2);

  return point_stamped_message;
}

const sensor_msgs::msg::TimeReference CustomMessageWrapper::createRosUtcTimeReferenceMessage(
  std::shared_ptr<const sbg_driver::msg::SbgUtcTime> ref_sbg_utc_msg)
{
  if (ref_sbg_utc_msg->clock_status.clock_utc_status == SBG_ECOM_UTC_STATUS_INVALID)
  {
    throw std::runtime_error("Invalid UTC time reference");
  }
  first_valid_utc_ = true;
  sensor_msgs::msg::TimeReference utc_reference_message;

  //
  // This message is defined to have comparison between the System time and the Utc reference.
  // Header of the ROS message will always be the System time, and the source is the computed time from Utc data.
  //
  last_sbg_utc_ = ref_sbg_utc_msg;
  utc_reference_message.header.stamp  = rclcpp::Clock().now();
  utc_reference_message.time_ref      = convertInsTimeToUnix(ref_sbg_utc_msg->time_stamp);
  utc_reference_message.source        = "UTC time from device converted to Epoch";

  return utc_reference_message;
}

const sensor_msgs::msg::NavSatFix CustomMessageWrapper::createRosNavSatFixMessage(
  sbg_driver::msg::SbgGpsPos::ConstSharedPtr ref_sbg_gps_msg) const
{
  sensor_msgs::msg::NavSatFix nav_sat_fix_message;

  nav_sat_fix_message.header = createRosHeader(ref_sbg_gps_msg->time_stamp);

  if (ref_sbg_gps_msg->status.type == SBG_ECOM_GNSS_POS_TYPE_NO_SOLUTION)
  {
    nav_sat_fix_message.status.status = nav_sat_fix_message.status.STATUS_NO_FIX;
  }
  else if (ref_sbg_gps_msg->status.type == SBG_ECOM_GNSS_POS_TYPE_SBAS)
  {
    nav_sat_fix_message.status.status = nav_sat_fix_message.status.STATUS_SBAS_FIX;
  }
  else
  {
    nav_sat_fix_message.status.status = nav_sat_fix_message.status.STATUS_FIX;
  }

  if (ref_sbg_gps_msg->status.glo_l1_used || ref_sbg_gps_msg->status.glo_l2_used)
  {
    nav_sat_fix_message.status.service = nav_sat_fix_message.status.SERVICE_GLONASS;
  }
  else
  {
    nav_sat_fix_message.status.service = nav_sat_fix_message.status.SERVICE_GPS;
  }

  nav_sat_fix_message.latitude  = ref_sbg_gps_msg->latitude;
  nav_sat_fix_message.longitude = ref_sbg_gps_msg->longitude;
  nav_sat_fix_message.altitude  = ref_sbg_gps_msg->altitude + ref_sbg_gps_msg->undulation;

  nav_sat_fix_message.position_covariance[0] = pow(ref_sbg_gps_msg->position_accuracy.x, 2);
  nav_sat_fix_message.position_covariance[4] = pow(ref_sbg_gps_msg->position_accuracy.y, 2);
  nav_sat_fix_message.position_covariance[8] = pow(ref_sbg_gps_msg->position_accuracy.z, 2);

  nav_sat_fix_message.position_covariance_type = nav_sat_fix_message.COVARIANCE_TYPE_DIAGONAL_KNOWN;

  return nav_sat_fix_message;
}


const sensor_msgs::msg::NavSatFix CustomMessageWrapper::createRosNavSatFixMessage(
  sbg_driver::msg::SbgEkfNav::ConstSharedPtr ref_sbg_ekf_nav_msg,
  sbg_driver::msg::SbgGpsPos::ConstSharedPtr ref_sbg_gps_msg) const
{
  sensor_msgs::msg::NavSatFix nav_sat_fix_message;
  nav_sat_fix_message.header = createRosHeader(ref_sbg_gps_msg->time_stamp);

  if (ref_sbg_gps_msg->status.type == SBG_ECOM_GNSS_POS_TYPE_NO_SOLUTION)
  {
    nav_sat_fix_message.status.status = nav_sat_fix_message.status.STATUS_NO_FIX;
  }
  else if (ref_sbg_gps_msg->status.type == SBG_ECOM_GNSS_POS_TYPE_SBAS)
  {
    nav_sat_fix_message.status.status = nav_sat_fix_message.status.STATUS_SBAS_FIX;
  }
  else
  {
    nav_sat_fix_message.status.status = nav_sat_fix_message.status.STATUS_FIX;
  }

  if (ref_sbg_gps_msg->status.glo_l1_used || ref_sbg_gps_msg->status.glo_l2_used)
  {
    nav_sat_fix_message.status.service = nav_sat_fix_message.status.SERVICE_GLONASS;
  }
  else
  {
    nav_sat_fix_message.status.service = nav_sat_fix_message.status.SERVICE_GPS;
  }

  nav_sat_fix_message.latitude  = ref_sbg_ekf_nav_msg->latitude;
  nav_sat_fix_message.longitude = ref_sbg_ekf_nav_msg->longitude;
  nav_sat_fix_message.altitude  = ref_sbg_ekf_nav_msg->altitude + ref_sbg_ekf_nav_msg->undulation;

  nav_sat_fix_message.position_covariance[0] = pow(ref_sbg_ekf_nav_msg->position_accuracy.x, 2);
  nav_sat_fix_message.position_covariance[4] = pow(ref_sbg_ekf_nav_msg->position_accuracy.y, 2);
  nav_sat_fix_message.position_covariance[8] = pow(ref_sbg_ekf_nav_msg->position_accuracy.z, 2);

  nav_sat_fix_message.position_covariance_type = nav_sat_fix_message.COVARIANCE_TYPE_DIAGONAL_KNOWN;

  return nav_sat_fix_message;
}

const sensor_msgs::msg::FluidPressure CustomMessageWrapper::createRosFluidPressureMessage(const sbg_driver::msg::SbgAirData& ref_sbg_air_msg) const
{
  sensor_msgs::msg::FluidPressure fluid_pressure_message;

  fluid_pressure_message.header         = createRosHeader(ref_sbg_air_msg.time_stamp);
  fluid_pressure_message.fluid_pressure = ref_sbg_air_msg.pressure_abs;
  fluid_pressure_message.variance       = 0.0;

  return fluid_pressure_message;
}

const geometry_msgs::msg::TransformStamped CustomMessageWrapper::invertTransform(const geometry_msgs::msg::TransformStamped &transform) const {

  tf2::Transform tf2_transform;
  tf2::fromMsg(transform.transform, tf2_transform);
  tf2::Transform inverted_tf2_transform = tf2_transform.inverse();

  geometry_msgs::msg::TransformStamped inverted_transform;
  inverted_transform.header = transform.header;
  inverted_transform.child_frame_id = transform.header.frame_id;
  inverted_transform.header.frame_id = transform.child_frame_id;
  inverted_transform.transform = tf2::toMsg(inverted_tf2_transform);
  return inverted_transform;

}
