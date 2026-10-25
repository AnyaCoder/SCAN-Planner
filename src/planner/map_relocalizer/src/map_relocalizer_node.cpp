#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace map_relocalizer
{
namespace
{
double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  const double norm =
      q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w) && norm > 1e-12;
}

Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond rotation(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  rotation.normalize();
  transform.linear() = rotation.toRotationMatrix();
  transform.translation() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

geometry_msgs::msg::Pose isometryToPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Vector3d translation = transform.translation();
  const Eigen::Quaterniond rotation(transform.linear());
  pose.position.x = translation.x();
  pose.position.y = translation.y();
  pose.position.z = translation.z();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

Eigen::Isometry3d odometryToIsometry(const nav_msgs::msg::Odometry & odom)
{
  return poseToIsometry(odom.pose.pose);
}
}  // namespace

class MapRelocalizer : public rclcpp::Node
{
public:
  MapRelocalizer()
  : Node("map_relocalizer")
  {
    prior_map_file_ = declare_parameter<std::string>("prior_map_file", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "world");
    local_frame_ = declare_parameter<std::string>("local_frame", "fastlio_world");

    input_vehicle_topic_ =
        declare_parameter<std::string>("input_vehicle_topic", "/LIO/odom_vehicle");
    input_imu_topic_ =
        declare_parameter<std::string>("input_imu_topic", "/LIO/odom_imu");
    input_cloud_topic_ =
        declare_parameter<std::string>("input_cloud_topic", "/LIO/clouds_lidar");
    initial_pose_topic_ =
        declare_parameter<std::string>("initial_pose_topic", "/initialpose");

    output_vehicle_topic_ = declare_parameter<std::string>(
        "output_vehicle_topic", "/LIO/relocalized/odom_vehicle");
    output_imu_topic_ = declare_parameter<std::string>(
        "output_imu_topic", "/LIO/relocalized/odom_imu");
    status_topic_ =
        declare_parameter<std::string>("status_topic", "/relocalization/status");
    fitness_topic_ =
        declare_parameter<std::string>("fitness_topic", "/relocalization/fitness");
    prior_map_topic_ =
        declare_parameter<std::string>("prior_map_topic", "/relocalization/map");

    prior_map_voxel_size_ =
        declare_parameter<double>("prior_map_voxel_size", 0.10);
    scan_voxel_size_ = declare_parameter<double>("scan_voxel_size", 0.10);
    target_crop_radius_ = declare_parameter<double>("target_crop_radius", 12.0);
    target_crop_z_ = declare_parameter<double>("target_crop_z", 3.0);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 100);
    min_target_points_ = declare_parameter<int>("min_target_points", 200);

    icp_max_correspondence_distance_ =
        declare_parameter<double>("icp_max_correspondence_distance", 0.60);
    icp_max_iterations_ = declare_parameter<int>("icp_max_iterations", 40);
    icp_fitness_threshold_ =
        declare_parameter<double>("icp_fitness_threshold", 0.05);
    icp_max_translation_correction_ =
        declare_parameter<double>("icp_max_translation_correction", 2.0);
    icp_max_rotation_correction_deg_ =
        declare_parameter<double>("icp_max_rotation_correction_deg", 45.0);
    max_scan_odom_dt_ = declare_parameter<double>("max_scan_odom_dt", 0.15);

    relocalization_period_ =
        declare_parameter<double>("relocalization_period", 1.0);
    tf_publish_period_ = declare_parameter<double>("tf_publish_period", 0.05);

    if (prior_map_file_.empty())
      throw std::invalid_argument("prior_map_file must name a world-aligned PCD");
    if (map_frame_ == local_frame_)
      throw std::invalid_argument("map_frame and local_frame must differ");
    if (prior_map_voxel_size_ <= 0.0 || scan_voxel_size_ <= 0.0 ||
        target_crop_radius_ <= 0.0 || target_crop_z_ <= 0.0 ||
        icp_max_correspondence_distance_ <= 0.0 ||
        icp_fitness_threshold_ <= 0.0 ||
        icp_max_translation_correction_ <= 0.0 ||
        icp_max_rotation_correction_deg_ <= 0.0 ||
        relocalization_period_ <= 0.0 || tf_publish_period_ <= 0.0)
    {
      throw std::invalid_argument("relocalizer distance and period parameters must be positive");
    }
    min_scan_points_ = std::max(10, min_scan_points_);
    min_target_points_ = std::max(50, min_target_points_);

    loadPriorMap();

    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    const auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();
    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();

    prior_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        prior_map_topic_, latched_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        status_topic_, latched_qos);
    fitness_pub_ = create_publisher<std_msgs::msg::Float32>(
        fitness_topic_, latched_qos);
    vehicle_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        output_vehicle_topic_, sensor_qos);
    imu_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        output_imu_topic_, sensor_qos);

    vehicle_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_vehicle_topic_, sensor_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onVehicle(msg); });
    imu_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_imu_topic_, sensor_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onImu(msg); });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, cloud_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onCloud(msg); });
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic_, rclcpp::QoS(1).reliable(),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
          onInitialPose(msg);
        });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_timer_ = create_wall_timer(
        std::chrono::duration<double>(tf_publish_period_),
        std::bind(&MapRelocalizer::publishTransform, this));
    relocalization_timer_ = create_wall_timer(
        std::chrono::duration<double>(relocalization_period_),
        std::bind(&MapRelocalizer::runRelocalization, this));

    publishPriorMap();
    publishStatus("waiting_initial_pose");
    RCLCPP_INFO(
        get_logger(),
        "Map relocalizer ready: prior=%s points=%zu map=%s local=%s",
        prior_map_file_.c_str(), prior_map_->size(), map_frame_.c_str(),
        local_frame_.c_str());
  }

private:
  struct OdomSample
  {
    nav_msgs::msg::Odometry msg;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    bool valid = false;
  };

  void loadPriorMap()
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr loaded(new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile(prior_map_file_, *loaded) < 0)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>());
      if (pcl::io::loadPCDFile(prior_map_file_, *xyz) < 0)
        throw std::runtime_error("failed to load prior PCD: " + prior_map_file_);
      loaded->reserve(xyz->size());
      for (const auto & source : xyz->points)
      {
        pcl::PointXYZI point;
        point.x = source.x;
        point.y = source.y;
        point.z = source.z;
        point.intensity = 0.0f;
        loaded->push_back(point);
      }
    }

    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(loaded);
    voxel.setLeafSize(
        static_cast<float>(prior_map_voxel_size_),
        static_cast<float>(prior_map_voxel_size_),
        static_cast<float>(prior_map_voxel_size_));
    prior_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    voxel.filter(*prior_map_);
    if (prior_map_->empty())
      throw std::runtime_error("prior PCD is empty after downsampling");
  }

  void publishPriorMap()
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*prior_map_, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    prior_map_pub_->publish(msg);
  }

  void publishStatus(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "relocalization: %s", status.c_str());
  }

  void publishFitness(double fitness)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(fitness);
    fitness_pub_->publish(msg);
  }

  void onVehicle(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    latest_vehicle_.msg = *msg;
    latest_vehicle_.pose = odometryToIsometry(*msg);
    latest_vehicle_.valid = true;
    if (pending_initial_pose_ && !have_map_local_)
      applyInitialPose(*pending_initial_pose_);
    if (converged_)
      publishCorrectedOdom(latest_vehicle_, vehicle_pub_);
  }

  void onImu(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    latest_imu_.msg = *msg;
    latest_imu_.pose = odometryToIsometry(*msg);
    latest_imu_.valid = true;
    if (converged_)
      publishCorrectedOdom(latest_imu_, imu_pub_);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);
    latest_cloud_ = cloud;
    latest_cloud_stamp_ = stampSeconds(msg->header.stamp);
    latest_cloud_valid_ = true;
  }

  void onInitialPose(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
  {
    if (!finitePose(msg->pose.pose))
    {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite initial pose");
      return;
    }
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_)
    {
      RCLCPP_WARN(
          get_logger(), "Initial pose frame is %s, expected %s",
          msg->header.frame_id.c_str(), map_frame_.c_str());
    }
    pending_initial_pose_ = *msg;
    if (latest_vehicle_.valid)
      applyInitialPose(*pending_initial_pose_);
    else
      publishStatus("waiting_local_odometry");
  }

  void applyInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    const Eigen::Isometry3d map_base_guess = poseToIsometry(msg.pose.pose);
    T_map_local_ = map_base_guess * latest_vehicle_.pose.inverse();
    have_map_local_ = true;
    converged_ = false;
    pending_initial_pose_.reset();
    last_icp_attempt_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    publishStatus("relocalizing");
  }

  void publishTransform()
  {
    if (!have_map_local_)
      return;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = local_frame_;
    transform.transform.translation.x = T_map_local_.translation().x();
    transform.transform.translation.y = T_map_local_.translation().y();
    transform.transform.translation.z = T_map_local_.translation().z();
    const Eigen::Quaterniond rotation(T_map_local_.linear());
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();
    tf_broadcaster_->sendTransform(transform);
  }

  void publishCorrectedOdom(
      const OdomSample & sample,
      const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & publisher)
  {
    if (!have_map_local_ || !converged_ || !sample.valid)
      return;

    nav_msgs::msg::Odometry output = sample.msg;
    const Eigen::Isometry3d map_sensor = T_map_local_ * sample.pose;
    output.header.frame_id = map_frame_;
    output.pose.pose = isometryToPose(map_sensor);

    const Eigen::Matrix3d rotation = T_map_local_.linear();
    const Eigen::Vector3d linear(
        sample.msg.twist.twist.linear.x,
        sample.msg.twist.twist.linear.y,
        sample.msg.twist.twist.linear.z);
    const Eigen::Vector3d angular(
        sample.msg.twist.twist.angular.x,
        sample.msg.twist.twist.angular.y,
        sample.msg.twist.twist.angular.z);
    const Eigen::Vector3d linear_map = rotation * linear;
    const Eigen::Vector3d angular_map = rotation * angular;
    output.twist.twist.linear.x = linear_map.x();
    output.twist.twist.linear.y = linear_map.y();
    output.twist.twist.linear.z = linear_map.z();
    output.twist.twist.angular.x = angular_map.x();
    output.twist.twist.angular.y = angular_map.y();
    output.twist.twist.angular.z = angular_map.z();
    publisher->publish(output);
  }

  void runRelocalization()
  {
    if (!have_map_local_ || converged_)
      return;
    if (!latest_cloud_valid_ || !latest_imu_.valid)
    {
      publishStatus("waiting_cloud_and_odometry");
      return;
    }

    const rclcpp::Time current_time = now();
    if (last_icp_attempt_.seconds() > 0.0 &&
        (current_time - last_icp_attempt_).seconds() < relocalization_period_)
      return;
    last_icp_attempt_ = current_time;

    const double cloud_age =
        std::fabs(stampSeconds(latest_imu_.msg.header.stamp) - latest_cloud_stamp_);
    if (cloud_age > max_scan_odom_dt_)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping ICP: cloud/odometry stamp mismatch %.3f s", cloud_age);
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(latest_cloud_);
    voxel.setLeafSize(
        static_cast<float>(scan_voxel_size_),
        static_cast<float>(scan_voxel_size_),
        static_cast<float>(scan_voxel_size_));
    voxel.filter(*scan);
    if (static_cast<int>(scan->size()) < min_scan_points_)
    {
      publishStatus("scan_too_small");
      return;
    }

    const Eigen::Isometry3d map_body_guess = T_map_local_ * latest_imu_.pose;
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_map(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(
        *scan, *scan_map, Eigen::Matrix4f(map_body_guess.matrix().cast<float>()));

    const Eigen::Vector3d center = map_body_guess.translation();
    pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>());
    for (const auto & point : prior_map_->points)
    {
      const double dx = point.x - center.x();
      const double dy = point.y - center.y();
      if (dx * dx + dy * dy > target_crop_radius_ * target_crop_radius_)
        continue;
      if (std::fabs(point.z - center.z()) > target_crop_z_)
        continue;
      target->push_back(point);
    }
    if (static_cast<int>(target->size()) < min_target_points_)
    {
      publishStatus("prior_map_crop_too_small");
      return;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(scan_map);
    icp.setInputTarget(target);
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-8);

    pcl::PointCloud<pcl::PointXYZI> aligned;
    icp.align(aligned, Eigen::Matrix4f::Identity());
    if (!icp.hasConverged())
    {
      publishStatus("icp_not_converged");
      return;
    }

    const Eigen::Isometry3d correction(icp.getFinalTransformation().cast<double>());
    const double correction_translation = correction.translation().norm();
    const double correction_rotation =
        Eigen::AngleAxisd(correction.linear()).angle();
    const double fitness = icp.getFitnessScore();
    publishFitness(fitness);

    if (!std::isfinite(fitness) || fitness > icp_fitness_threshold_ ||
        correction_translation > icp_max_translation_correction_ ||
        correction_rotation > icp_max_rotation_correction_deg_ * M_PI / 180.0)
    {
      publishStatus(
          "icp_rejected fitness=" + std::to_string(fitness) +
          " translation=" + std::to_string(correction_translation) +
          " rotation_deg=" + std::to_string(correction_rotation * 180.0 / M_PI));
      return;
    }

    const Eigen::Isometry3d map_body_corrected = correction * map_body_guess;
    T_map_local_ = map_body_corrected * latest_imu_.pose.inverse();
    converged_ = true;
    publishStatus("converged fitness=" + std::to_string(fitness));
    if (latest_vehicle_.valid)
      publishCorrectedOdom(latest_vehicle_, vehicle_pub_);
    publishCorrectedOdom(latest_imu_, imu_pub_);
  }

  std::string prior_map_file_;
  std::string map_frame_;
  std::string local_frame_;
  std::string input_vehicle_topic_;
  std::string input_imu_topic_;
  std::string input_cloud_topic_;
  std::string initial_pose_topic_;
  std::string output_vehicle_topic_;
  std::string output_imu_topic_;
  std::string status_topic_;
  std::string fitness_topic_;
  std::string prior_map_topic_;

  double prior_map_voxel_size_;
  double scan_voxel_size_;
  double target_crop_radius_;
  double target_crop_z_;
  int min_scan_points_;
  int min_target_points_;
  double icp_max_correspondence_distance_;
  int icp_max_iterations_;
  double icp_fitness_threshold_;
  double icp_max_translation_correction_;
  double icp_max_rotation_correction_deg_;
  double max_scan_odom_dt_;
  double relocalization_period_;
  double tf_publish_period_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr prior_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud_;
  double latest_cloud_stamp_{0.0};
  bool latest_cloud_valid_{false};

  OdomSample latest_vehicle_;
  OdomSample latest_imu_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> pending_initial_pose_;

  Eigen::Isometry3d T_map_local_ = Eigen::Isometry3d::Identity();
  bool have_map_local_{false};
  bool converged_{false};
  rclcpp::Time last_icp_attempt_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fitness_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr relocalization_timer_;
};
}  // namespace map_relocalizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<map_relocalizer::MapRelocalizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
