/* includes //{ */

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/iir_filter.h>
#include <mrs_lib/notch_filter.h>

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

class VinsImuFilter : public rclcpp::Node {
public:
  VinsImuFilter(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::TimerBase::SharedPtr timer_preinitialization_;
  void                         timerPreInitialization();

  void initialize();

  /* flags */
  std::atomic<bool> is_initialized_ = false;

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

  bool _skip_spike_;
  double _gravity_;
  double _max_accel_factor_;

  // | ------------------------ subscribers --------------------- |

  mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu> sh_imu_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu> sh_accel_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu> sh_gyro_;

  // | ------------------------ callbacks ----------------------- |

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu);
  void accelCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu);
  void gyroCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu);

  sensor_msgs::msg::Imu filterAccelerometer(const sensor_msgs::msg::Imu &imu);
  sensor_msgs::msg::Imu filterGyro(const sensor_msgs::msg::Imu &imu);

  mrs_lib::PublisherHandler<sensor_msgs::msg::Imu> ph_imu_;

  sensor_msgs::msg::Imu last_accel_msg_;
  std::mutex            mutex_last_accel_msg_;

  std::vector<std::shared_ptr<NotchFilterStruct>> acc_notch_filter_vector_;
  std::vector<std::shared_ptr<NotchFilterStruct>> gyro_notch_filter_vector_;

  IirFilterStruct acc_iir_filter_;
  IirFilterStruct gyro_iir_filter_;

  bool imu_received_  = false;
  bool acc_received_  = false;
  bool gyro_received_ = false;
};

//}

/* VinsImuFilter() //{ */

VinsImuFilter::VinsImuFilter(rclcpp::NodeOptions options) : Node("VinsImuFilter", options) {
  timer_preinitialization_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&VinsImuFilter::timerPreInitialization, this));
}

//}

/* timerPreInitialization() //{ */

void VinsImuFilter::timerPreInitialization() {
  node_  = this->shared_from_this();
  clock_ = node_->get_clock();

  initialize();
  timer_preinitialization_->cancel();
}

//}

/* initialize() //{ */

void VinsImuFilter::initialize() {
  RCLCPP_INFO(node_->get_logger(), "initializing");

  // | ---------- loading ros parameters using mrs_lib ---------- |
  RCLCPP_INFO(node_->get_logger(), "loading parameters using ParamLoader");

  mrs_lib::ParamLoader param_loader(node_, "VinsImuFilter");

  param_loader.addYamlFileFromParam("public_config");

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

  param_loader.loadParam("change_frame_id/enabled", _change_frame_id_enabled_);

  param_loader.loadParam("skip_spike", _skip_spike_);
  param_loader.loadParam("gravity", _gravity_);
  param_loader.loadParam("max_accel_factor", _max_accel_factor_);
  
  if (_change_frame_id_enabled_) {
    param_loader.loadParam("change_frame_id/target_frame_id", _target_frame_id_);
  }

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "parameter loading failure");
    rclcpp::shutdown();
    exit(1);
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node = node_;

  sh_imu_   = mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>(shopts, "~/imu_in", &VinsImuFilter::imuCallback, this);
  sh_accel_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>(shopts, "~/accel_in", &VinsImuFilter::accelCallback, this);
  sh_gyro_  = mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>(shopts, "~/gyro_in", &VinsImuFilter::gyroCallback, this);

  // | ----------------------- publishers ----------------------- |

  ph_imu_ = mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>(node_, "~/imu_out");

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

  RCLCPP_INFO_ONCE(node_->get_logger(), "initialized");
}

//}

/* imuCallback() //{ */

void VinsImuFilter::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu) {

  if (!is_initialized_) {
    return;
  }

  imu_received_ = true;

  if (acc_received_ || gyro_received_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "receiving IMU messages but also separate acc or gyro messages, check topic remapping.");
  }

  if(_skip_spike_){
    // Check for acceleration spikes before filtering
    double accel_magnitude = std::sqrt(
      imu->linear_acceleration.x * imu->linear_acceleration.x +
      imu->linear_acceleration.y * imu->linear_acceleration.y +
      imu->linear_acceleration.z * imu->linear_acceleration.z
    );
    
    const double MAX_ACCEL = _max_accel_factor_ * _gravity_;  // 2G threshold
    
    if (accel_magnitude > MAX_ACCEL) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *clock_, 100,
        "IMU acceleration spike detected: %.2f m/s^2 (threshold: %.2f m/s^2), skipping this measurement",
        accel_magnitude, MAX_ACCEL
      );
      return;  // Skip this entire IMU message
    }
  }

  sensor_msgs::msg::Imu imu_filtered = filterAccelerometer(*imu);
  imu_filtered                       = filterGyro(imu_filtered);

  if (_change_frame_id_enabled_) {
    imu_filtered.header.frame_id = _target_frame_id_;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "filtering");

  ph_imu_.publish(imu_filtered);
}

//}

/* accelCallback() //{ */

void VinsImuFilter::accelCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu) {

  if (!is_initialized_) {
    return;
  }

  acc_received_ = true;

  // copy mode - filter incoming accelerometer data and save it
  sensor_msgs::msg::Imu imu_filtered = filterAccelerometer(*imu);

  if (_change_frame_id_enabled_) {
    imu_filtered.header.frame_id = _target_frame_id_;
  }

  {
    std::scoped_lock lock(mutex_last_accel_msg_);
    last_accel_msg_ = imu_filtered;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "filtering accelerometer msgs");
}

//}

/* gyroCallback() //{ */

void VinsImuFilter::gyroCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu) {

  if (!is_initialized_) {
    return;
  }

  gyro_received_ = true;

  // copy mode - filter gyro msg, insert last accel msg into it and publish it
  sensor_msgs::msg::Imu imu_filtered = filterGyro(*imu);
  
  if (_change_frame_id_enabled_) {
    imu_filtered.header.frame_id = _target_frame_id_;
  }

  {
    std::scoped_lock lock(mutex_last_accel_msg_);
    imu_filtered.linear_acceleration = last_accel_msg_.linear_acceleration;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "filtering gyro msgs");

  ph_imu_.publish(imu_filtered);
}

//}

/* filterAccelerometer() */ /*//{*/
sensor_msgs::msg::Imu VinsImuFilter::filterAccelerometer(const sensor_msgs::msg::Imu &imu) {

  sensor_msgs::msg::Imu imu_filtered = imu;

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
sensor_msgs::msg::Imu VinsImuFilter::filterGyro(const sensor_msgs::msg::Imu &imu) {

  sensor_msgs::msg::Imu imu_filtered = imu;

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

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vins_imu_filter::VinsImuFilter);
