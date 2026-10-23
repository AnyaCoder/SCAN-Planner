#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace lio_bridge
{
namespace
{
double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

class VelocityEstimator
{
public:
  VelocityEstimator(
    double window_sec, double max_speed, double max_gap_sec, double deadband)
  : window_sec_(window_sec),
    max_speed_(max_speed),
    max_gap_sec_(max_gap_sec),
    deadband_(deadband)
  {
  }

  void update(double stamp_sec, double x, double y, double z)
  {
    if (!samples_.empty()) {
      const double dt = stamp_sec - samples_.back().stamp;
      const double dx = x - samples_.back().x;
      const double dy = y - samples_.back().y;
      const double dz = z - samples_.back().z;
      const double displacement = std::sqrt(dx * dx + dy * dy + dz * dz);

      // A timestamp gap or an impossible per-sample speed usually means a LIO
      // reset/relocalization jump. Do not turn that discontinuity into velocity.
      if (dt <= 0.0 || dt > max_gap_sec_ || displacement / dt > max_speed_) {
        samples_.clear();
        velocity_ = {0.0, 0.0, 0.0};
      }
    }

    samples_.push_back({stamp_sec, x, y, z});
    while (samples_.size() > 2 &&
      samples_[1].stamp <= stamp_sec - window_sec_) {
      samples_.pop_front();
    }

    if (samples_.size() < 2) {
      velocity_ = {0.0, 0.0, 0.0};
      return;
    }

    const double span = samples_.back().stamp - samples_.front().stamp;
    if (span < 0.02) {
      velocity_ = {0.0, 0.0, 0.0};
      return;
    }

    velocity_[0] = (samples_.back().x - samples_.front().x) / span;
    velocity_[1] = (samples_.back().y - samples_.front().y) / span;
    velocity_[2] = (samples_.back().z - samples_.front().z) / span;

    const double speed = speedNorm();
    if (speed < deadband_) {
      velocity_ = {0.0, 0.0, 0.0};
      return;
    }
    if (speed > max_speed_) {
      const double scale = max_speed_ / speed;
      velocity_[0] *= scale;
      velocity_[1] *= scale;
      velocity_[2] *= scale;
    }
  }

  const std::array<double, 3> & velocity() const { return velocity_; }
  double speedNorm() const
  {
    return std::sqrt(
      velocity_[0] * velocity_[0] + velocity_[1] * velocity_[1] +
      velocity_[2] * velocity_[2]);
  }

private:
  struct Sample
  {
    double stamp;
    double x;
    double y;
    double z;
  };

  double window_sec_;
  double max_speed_;
  double max_gap_sec_;
  double deadband_;
  std::deque<Sample> samples_;
  std::array<double, 3> velocity_{0.0, 0.0, 0.0};
};
}  // namespace

class LioBridge : public rclcpp::Node
{
public:
  LioBridge()
  : Node("lio_bridge"),
    velocity_estimator_(1.0, 2.0, 0.25, 0.12)
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    cloud_topic_ = declare_parameter<std::string>(
        "cloud_topic", "/cloud_registered_body");
    output_vehicle_topic_ = declare_parameter<std::string>(
        "output_vehicle_topic", "/LIO/odom_vehicle");
    output_imu_topic_ = declare_parameter<std::string>(
        "output_imu_topic", "/LIO/odom_imu");
    output_cloud_topic_ = declare_parameter<std::string>(
        "output_cloud_topic", "/LIO/clouds_lidar");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "world");
    output_child_frame_id_ = declare_parameter<std::string>("output_child_frame_id", "base");
    velocity_window_sec_ = declare_parameter<double>("velocity_window_sec", 1.0);
    max_velocity_ = declare_parameter<double>("max_velocity", 2.0);
    velocity_deadband_ = declare_parameter<double>("velocity_deadband", 0.12);
    max_timestamp_gap_sec_ = declare_parameter<double>("max_timestamp_gap_sec", 0.25);

    if (velocity_window_sec_ <= 0.0 || max_velocity_ <= 0.0 ||
      velocity_deadband_ < 0.0 || velocity_deadband_ >= max_velocity_ ||
      max_timestamp_gap_sec_ <= 0.0)
    {
      throw std::invalid_argument("velocity and timestamp-gap parameters must be positive");
    }
    velocity_estimator_ = VelocityEstimator(
      velocity_window_sec_, max_velocity_, max_timestamp_gap_sec_, velocity_deadband_);

    // Keep the real-time bridge on the newest samples. Deep input queues
    // accumulate stale LIO data when the transport is busy.
    const auto odom_input_qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();
    const auto cloud_input_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    const auto odom_output_qos = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort();
    const auto cloud_output_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

    odom_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = odom_callback_group_;
    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.callback_group = cloud_callback_group_;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, odom_input_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onOdom(msg); },
        odom_options);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, cloud_input_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onCloud(msg); },
        cloud_options);

    vehicle_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        output_vehicle_topic_, odom_output_qos);
    imu_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        output_imu_topic_, odom_output_qos);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_cloud_topic_, cloud_output_qos);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    stats_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]() { logStats(); });

    RCLCPP_INFO(
        get_logger(),
        "LIO bridge ready: odom=%s, cloud=%s, output frame=%s child=%s",
        odom_topic_.c_str(), cloud_topic_.c_str(),
        output_frame_id_.c_str(), output_child_frame_id_.c_str());
  }

private:
  void fillOutputOdometry(
    const nav_msgs::msg::Odometry & input,
    nav_msgs::msg::Odometry & output) const
  {
    output = input;
    output.header.frame_id = output_frame_id_;
    output.child_frame_id = output_child_frame_id_;
    const auto & velocity = velocity_estimator_.velocity();
    output.twist.twist.linear.x = velocity[0];
    output.twist.twist.linear.y = velocity[1];
    output.twist.twist.linear.z = velocity[2];
  }

  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    odom_rx_.fetch_add(1, std::memory_order_relaxed);
    if (!odom_seen_.exchange(true, std::memory_order_relaxed)) {
      RCLCPP_INFO(
          get_logger(), "odom input frame=%s child=%s",
          msg->header.frame_id.c_str(), msg->child_frame_id.c_str());
    }

    double stamp = stampSeconds(msg->header.stamp);
    if (stamp <= 0.0) stamp = now().seconds();
    velocity_estimator_.update(
      stamp, msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z);

    nav_msgs::msg::Odometry output;
    fillOutputOdometry(*msg, output);
    vehicle_pub_->publish(output);
    imu_pub_->publish(output);
    velocity_norm_.store(velocity_estimator_.speedNorm());
    odom_vehicle_tx_.fetch_add(1, std::memory_order_relaxed);
    odom_imu_tx_.fetch_add(1, std::memory_order_relaxed);
    broadcastOdometryTransform(*msg, stamp);
  }

  void broadcastOdometryTransform(
    const nav_msgs::msg::Odometry & input, double stamp_sec)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = output_frame_id_;
    transform.child_frame_id = output_child_frame_id_;
    transform.header.stamp = input.header.stamp;
    if (stamp_sec <= 0.0) {
      transform.header.stamp = now();
    }
    transform.transform.translation.x = input.pose.pose.position.x;
    transform.transform.translation.y = input.pose.pose.position.y;
    transform.transform.translation.z = input.pose.pose.position.z;
    transform.transform.rotation = input.pose.pose.orientation;
    std::vector<geometry_msgs::msg::TransformStamped> transforms{transform};
    if (output_child_frame_id_ != "body") {
      auto sensor_transform = transform;
      sensor_transform.header.frame_id = output_child_frame_id_;
      sensor_transform.child_frame_id = "body";
      sensor_transform.transform = geometry_msgs::msg::Transform();
      sensor_transform.transform.rotation.w = 1.0;
      transforms.push_back(sensor_transform);
    }
    tf_broadcaster_->sendTransform(transforms);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    const uint64_t point_count = static_cast<uint64_t>(msg->width) * msg->height;
    cloud_rx_.fetch_add(1, std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      cloud_frame_ = msg->header.frame_id;
      cloud_points_ = point_count;
    }
    if (!cloud_seen_.exchange(true, std::memory_order_relaxed)) {
      RCLCPP_INFO(
          get_logger(), "cloud input frame=%s points=%llu",
          msg->header.frame_id.c_str(), static_cast<unsigned long long>(point_count));
    }
    cloud_pub_->publish(*msg);
    cloud_tx_.fetch_add(1, std::memory_order_relaxed);
  }

  void logStats()
  {
    const auto odom = odom_rx_.exchange(0, std::memory_order_relaxed);
    const auto cloud = cloud_rx_.exchange(0, std::memory_order_relaxed);
    const auto vehicle_tx = odom_vehicle_tx_.exchange(0, std::memory_order_relaxed);
    const auto imu_tx = odom_imu_tx_.exchange(0, std::memory_order_relaxed);
    const auto cloud_tx = cloud_tx_.exchange(0, std::memory_order_relaxed);

    std::string cloud_frame;
    uint64_t cloud_points = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      cloud_frame = cloud_frame_;
      cloud_points = cloud_points_;
    }

    RCLCPP_INFO(
        get_logger(),
        "rx/s odom=%llu cloud=%llu | tx/s vehicle=%llu imu=%llu cloud=%llu | "
        "cloud_frame=%s points=%llu",
        static_cast<unsigned long long>(odom), static_cast<unsigned long long>(cloud),
        static_cast<unsigned long long>(vehicle_tx), static_cast<unsigned long long>(imu_tx),
        static_cast<unsigned long long>(cloud_tx), cloud_frame.c_str(),
        static_cast<unsigned long long>(cloud_points));
    RCLCPP_INFO(
        get_logger(), "velocity estimate: %.3f m/s", velocity_norm_.load());
  }

  std::string odom_topic_;
  std::string cloud_topic_;
  std::string output_vehicle_topic_;
  std::string output_imu_topic_;
  std::string output_cloud_topic_;
  std::string output_frame_id_;
  std::string output_child_frame_id_;
  double velocity_window_sec_;
  double max_velocity_;
  double velocity_deadband_;
  double max_timestamp_gap_sec_;
  VelocityEstimator velocity_estimator_;

  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::atomic<uint64_t> odom_rx_{0};
  std::atomic<uint64_t> cloud_rx_{0};
  std::atomic<uint64_t> odom_vehicle_tx_{0};
  std::atomic<uint64_t> odom_imu_tx_{0};
  std::atomic<uint64_t> cloud_tx_{0};
  std::atomic<double> velocity_norm_{0.0};
  std::atomic<bool> odom_seen_{false};
  std::atomic<bool> cloud_seen_{false};
  std::mutex state_mutex_;
  std::string cloud_frame_;
  uint64_t cloud_points_ = 0;
};
}  // namespace lio_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lio_bridge::LioBridge>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
