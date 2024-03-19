/* includes //{ */

#include <memory>
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/iir_filter.h>
#include <mrs_lib/notch_filter.h>

#include <nodelet/nodelet.h>

#include <pluginlib/class_list_macros.h>

//}

namespace vins_imu_filter
{

struct NotchFilterStruct
{
  std::unique_ptr<mrs_lib::NotchFilter> notch_filter_x = nullptr;
  std::unique_ptr<mrs_lib::NotchFilter> notch_filter_y = nullptr;
  std::unique_ptr<mrs_lib::NotchFilter> notch_filter_z = nullptr;
};

struct IirFilterStruct
{
  std::unique_ptr<mrs_lib::IirFilter> iir_filter_x = nullptr;
  std::unique_ptr<mrs_lib::IirFilter> iir_filter_y = nullptr;
  std::unique_ptr<mrs_lib::IirFilter> iir_filter_z = nullptr;
};

/* class VinsImuFilter //{ */

class VinsImuFilter : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_ = false;

  /* ros parameters */
  bool            _acc_iir_filter_enabled_;
  Eigen::MatrixXd _acc_iir_filter_a_;
  Eigen::MatrixXd _acc_iir_filter_b_;
  bool            _acc_notch_filter_enabled_;
  double          _acc_notch_filter_sampling_rate_;
  Eigen::MatrixXd _acc_notch_filter_frequencies_;
  double          _acc_notch_filter_bandwidth_;

  bool            _gyro_iir_filter_enabled_;
  Eigen::MatrixXd _gyro_iir_filter_a_;
  Eigen::MatrixXd _gyro_iir_filter_b_;
  bool            _gyro_notch_filter_enabled_;
  double          _gyro_notch_filter_sampling_rate_;
  Eigen::MatrixXd _gyro_notch_filter_frequencies_;
  double          _gyro_notch_filter_bandwidth_;

  bool        _change_frame_id_enabled_ = false;
  std::string _target_frame_id_;

  // | ------------------------ callbacks ----------------------- |
  ros::Subscriber subscriber_imu_;
  ros::Subscriber subscriber_accel_;
  ros::Subscriber subscriber_gyro_;
  void            imuCallback(const sensor_msgs::ImuConstPtr &imu);
  void            accelCallback(const sensor_msgs::ImuConstPtr &imu);
  void            gyroCallback(const sensor_msgs::ImuConstPtr &imu);

  sensor_msgs::Imu filterAccelerometer(const sensor_msgs::Imu &imu);
  sensor_msgs::Imu filterGyro(const sensor_msgs::Imu &imu);

  ros::Publisher publisher_imu_;

  sensor_msgs::Imu last_accel_msg_;
  std::mutex       mutex_last_accel_msg_;

  std::vector<std::shared_ptr<NotchFilterStruct>> acc_notch_filter_vector_;
  std::vector<std::shared_ptr<NotchFilterStruct>> gyro_notch_filter_vector_;

  IirFilterStruct acc_iir_filter_;
  IirFilterStruct gyro_iir_filter_;

  bool imu_received_  = false;
  bool acc_received_  = false;
  bool gyro_received_ = false;
};

//}

/* onInit() //{ */

void VinsImuFilter::onInit() {
  const std::string node_name("VinsImuFilter");

  /* obtain node handle */
  /* ros::NodeHandle nh("~"); */
  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[%s]: Initializing", node_name.c_str());

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ---------- loading ros parameters using mrs_lib ---------- |
  ROS_INFO("[%s]: loading parameters using ParamLoader", node_name.c_str());

  mrs_lib::ParamLoader param_loader(nh_, node_name);
  param_loader.loadParam("accelerometer/iir_filter/enable", _acc_iir_filter_enabled_);
  param_loader.loadMatrixDynamic("accelerometer/iir_filter/a", _acc_iir_filter_a_, 1, -1);  // -1 indicates the dynamic dimension
  param_loader.loadMatrixDynamic("accelerometer/iir_filter/b", _acc_iir_filter_b_, 1, -1);
  param_loader.loadParam("accelerometer/notch_filter/enable", _acc_notch_filter_enabled_);
  param_loader.loadParam("accelerometer/notch_filter/sample_rate", _acc_notch_filter_sampling_rate_);
  param_loader.loadMatrixDynamic("accelerometer/notch_filter/frequencies", _acc_notch_filter_frequencies_, 1, -1);
  param_loader.loadParam("accelerometer/notch_filter/bandwidth", _acc_notch_filter_bandwidth_);
  param_loader.loadParam("gyro/iir_filter/enable", _gyro_iir_filter_enabled_);
  param_loader.loadMatrixDynamic("gyro/iir_filter/a", _gyro_iir_filter_a_, 1, -1);  // -1 indicates the dynamic dimension
  param_loader.loadMatrixDynamic("gyro/iir_filter/b", _gyro_iir_filter_b_, 1, -1);
  param_loader.loadParam("gyro/notch_filter/enable", _gyro_notch_filter_enabled_);
  param_loader.loadParam("gyro/notch_filter/sample_rate", _gyro_notch_filter_sampling_rate_);
  param_loader.loadMatrixDynamic("gyro/notch_filter/frequencies", _gyro_notch_filter_frequencies_, 1, -1);
  param_loader.loadParam("gyro/notch_filter/bandwidth", _gyro_notch_filter_bandwidth_);

  param_loader.loadParam("change_frame_id/enabled", _change_frame_id_enabled_, false);
  if (_change_frame_id_enabled_) {
    param_loader.loadParam("change_frame_id/target_frame_id", _target_frame_id_);
  }

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: parameter loading failure", node_name.c_str());
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  subscriber_imu_   = nh_.subscribe("imu_in", 10, &VinsImuFilter::imuCallback, this, ros::TransportHints().tcpNoDelay());
  subscriber_accel_ = nh_.subscribe("accel_in", 10, &VinsImuFilter::accelCallback, this, ros::TransportHints().tcpNoDelay());
  subscriber_gyro_  = nh_.subscribe("gyro_in", 10, &VinsImuFilter::gyroCallback, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  publisher_imu_ = nh_.advertise<sensor_msgs::Imu>("imu_out", 10);

  // create IIR filters
  std::vector<double> acc_iir_a(_acc_iir_filter_a_.data(), _acc_iir_filter_a_.data() + _acc_iir_filter_a_.size());
  std::vector<double> acc_iir_b(_acc_iir_filter_b_.data(), _acc_iir_filter_b_.data() + _acc_iir_filter_b_.size());
  acc_iir_filter_.iir_filter_x = std::make_unique<mrs_lib::IirFilter>(acc_iir_a, acc_iir_b);
  acc_iir_filter_.iir_filter_y = std::make_unique<mrs_lib::IirFilter>(acc_iir_a, acc_iir_b);
  acc_iir_filter_.iir_filter_z = std::make_unique<mrs_lib::IirFilter>(acc_iir_a, acc_iir_b);

  std::vector<double> gyro_iir_a(_gyro_iir_filter_a_.data(), _gyro_iir_filter_a_.data() + _gyro_iir_filter_a_.size());
  std::vector<double> gyro_iir_b(_gyro_iir_filter_b_.data(), _gyro_iir_filter_b_.data() + _gyro_iir_filter_b_.size());
  gyro_iir_filter_.iir_filter_x = std::make_unique<mrs_lib::IirFilter>(gyro_iir_a, gyro_iir_b);
  gyro_iir_filter_.iir_filter_y = std::make_unique<mrs_lib::IirFilter>(gyro_iir_a, gyro_iir_b);
  gyro_iir_filter_.iir_filter_z = std::make_unique<mrs_lib::IirFilter>(gyro_iir_a, gyro_iir_b);

  // create notch filters
  for (int i = 0; i < _acc_notch_filter_frequencies_.cols(); i++) {
    std::shared_ptr<NotchFilterStruct> nfs = std::make_shared<NotchFilterStruct>();
    nfs->notch_filter_x =
        std::make_unique<mrs_lib::NotchFilter>(_acc_notch_filter_sampling_rate_, _acc_notch_filter_frequencies_(i), _acc_notch_filter_bandwidth_);
    nfs->notch_filter_y =
        std::make_unique<mrs_lib::NotchFilter>(_acc_notch_filter_sampling_rate_, _acc_notch_filter_frequencies_(i), _acc_notch_filter_bandwidth_);
    nfs->notch_filter_z =
        std::make_unique<mrs_lib::NotchFilter>(_acc_notch_filter_sampling_rate_, _acc_notch_filter_frequencies_(i), _acc_notch_filter_bandwidth_);
    acc_notch_filter_vector_.push_back(nfs);
  }

  for (int i = 0; i < _gyro_notch_filter_frequencies_.cols(); i++) {
    std::shared_ptr<NotchFilterStruct> nfs = std::make_shared<NotchFilterStruct>();
    nfs->notch_filter_x =
        std::make_unique<mrs_lib::NotchFilter>(_gyro_notch_filter_sampling_rate_, _gyro_notch_filter_frequencies_(i), _gyro_notch_filter_bandwidth_);
    nfs->notch_filter_y =
        std::make_unique<mrs_lib::NotchFilter>(_gyro_notch_filter_sampling_rate_, _gyro_notch_filter_frequencies_(i), _gyro_notch_filter_bandwidth_);
    nfs->notch_filter_z =
        std::make_unique<mrs_lib::NotchFilter>(_gyro_notch_filter_sampling_rate_, _gyro_notch_filter_frequencies_(i), _gyro_notch_filter_bandwidth_);
    gyro_notch_filter_vector_.push_back(nfs);
  }

  is_initialized_ = true;

  ROS_INFO_ONCE("[%s]: initialized", node_name.c_str());
}
//}

/* imuCallback() //{ */

void VinsImuFilter::imuCallback(const sensor_msgs::ImuConstPtr &imu) {

  if (!is_initialized_) {
    return;
  }

  imu_received_ = true;

  if (acc_received_ || gyro_received_) {
    ROS_WARN_THROTTLE(1.0, "[%s]: Receiving IMU messages but also separate acc or gyro messages, check topic remapping.", ros::this_node::getName().c_str());
  }

  sensor_msgs::Imu imu_filtered = filterAccelerometer(*imu);
  imu_filtered                  = filterGyro(imu_filtered);

  if (_change_frame_id_enabled_) {
    imu_filtered.header.frame_id = _target_frame_id_;
  }


  ROS_INFO_THROTTLE(1.0, "[%s]: Filtering", ros::this_node::getName().c_str());

  publisher_imu_.publish(imu_filtered);
}

//}

/* accelCallback() //{ */

void VinsImuFilter::accelCallback(const sensor_msgs::ImuConstPtr &imu) {

  if (!is_initialized_) {
    return;
  }

  acc_received_ = true;

  // copy mode - filter incoming accelerometer data and save it
  sensor_msgs::Imu imu_filtered = filterAccelerometer(*imu);

  if (_change_frame_id_enabled_) {
    imu_filtered.header.frame_id = _target_frame_id_;
  }

  {
    std::scoped_lock lock(mutex_last_accel_msg_);
    last_accel_msg_ = imu_filtered;
  }

  ROS_INFO_THROTTLE(1.0, "[%s]: Filtering accelerometer msgs", ros::this_node::getName().c_str());
}

//}

/* gyroCallback() //{ */

void VinsImuFilter::gyroCallback(const sensor_msgs::ImuConstPtr &imu) {

  if (!is_initialized_) {
    return;
  }

  gyro_received_ = true;

  // copy mode - filter gyro msg, insert last accel msg into it and publish it
  sensor_msgs::Imu imu_filtered = filterGyro(*imu);
  
  if (_change_frame_id_enabled_) {
    imu_filtered.header.frame_id = _target_frame_id_;
  }

  {
    std::scoped_lock lock(mutex_last_accel_msg_);
    imu_filtered.linear_acceleration = last_accel_msg_.linear_acceleration;
  }

  ROS_INFO_THROTTLE(1.0, "[%s]: Filtering gyro msgs", ros::this_node::getName().c_str());

  publisher_imu_.publish(imu_filtered);
}

//}

/* filterAccelerometer() */ /*//{*/
sensor_msgs::Imu VinsImuFilter::filterAccelerometer(const sensor_msgs::Imu &imu) {

  sensor_msgs::Imu imu_filtered = imu;

  if (_acc_notch_filter_enabled_) {
    for (size_t i = 0; i < acc_notch_filter_vector_.size(); i++) {
      imu_filtered.linear_acceleration.x = acc_notch_filter_vector_.at(i)->notch_filter_x->iterate(imu_filtered.linear_acceleration.x);
      imu_filtered.linear_acceleration.y = acc_notch_filter_vector_.at(i)->notch_filter_y->iterate(imu_filtered.linear_acceleration.y);
      imu_filtered.linear_acceleration.z = acc_notch_filter_vector_.at(i)->notch_filter_z->iterate(imu_filtered.linear_acceleration.z);
    }
  }

  if (_acc_iir_filter_enabled_) {
    imu_filtered.linear_acceleration.x = acc_iir_filter_.iir_filter_x->iterate(imu_filtered.linear_acceleration.x);
    imu_filtered.linear_acceleration.y = acc_iir_filter_.iir_filter_y->iterate(imu_filtered.linear_acceleration.y);
    imu_filtered.linear_acceleration.z = acc_iir_filter_.iir_filter_z->iterate(imu_filtered.linear_acceleration.z);
  }

  return imu_filtered;
}
/*//}*/

/* filterGyro() */ /*//{*/
sensor_msgs::Imu VinsImuFilter::filterGyro(const sensor_msgs::Imu &imu) {

  sensor_msgs::Imu imu_filtered = imu;

  if (_gyro_notch_filter_enabled_) {
    for (size_t i = 0; i < gyro_notch_filter_vector_.size(); i++) {
      imu_filtered.angular_velocity.x = gyro_notch_filter_vector_.at(i)->notch_filter_x->iterate(imu_filtered.angular_velocity.x);
      imu_filtered.angular_velocity.y = gyro_notch_filter_vector_.at(i)->notch_filter_y->iterate(imu_filtered.angular_velocity.y);
      imu_filtered.angular_velocity.z = gyro_notch_filter_vector_.at(i)->notch_filter_z->iterate(imu_filtered.angular_velocity.z);
    }
  }

  if (_gyro_iir_filter_enabled_) {
    imu_filtered.angular_velocity.x = gyro_iir_filter_.iir_filter_x->iterate(imu_filtered.angular_velocity.x);
    imu_filtered.angular_velocity.y = gyro_iir_filter_.iir_filter_y->iterate(imu_filtered.angular_velocity.y);
    imu_filtered.angular_velocity.z = gyro_iir_filter_.iir_filter_z->iterate(imu_filtered.angular_velocity.z);
  }

  return imu_filtered;
}
/*//}*/

}  // namespace vins_imu_filter
/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(vins_imu_filter::VinsImuFilter, nodelet::Nodelet);
