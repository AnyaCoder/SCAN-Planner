#include <chrono>
#include <functional>
#include <iomanip>
#include <locale>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

namespace cmd_vel_sport_bridge
{
class CmdVelSportBridge : public rclcpp::Node
{
public:
  CmdVelSportBridge() : Node("cmd_vel_sport_bridge")
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    sport_request_topic_ = declare_parameter<std::string>(
        "sport_request_topic", "/api/sport/request");
    api_id_ = declare_parameter<int>("api_id", 1008);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);
    if (command_timeout_ <= 0.0)
      command_timeout_ = 0.5;

    request_pub_ = create_publisher<unitree_api::msg::Request>(
        sport_request_topic_, rclcpp::QoS(10).reliable());
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::QoS(20).reliable(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { forward(msg); });
    watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&CmdVelSportBridge::watchdogCallback, this));

    RCLCPP_INFO(
        get_logger(), "Direct bridge ready: %s -> %s (api_id=%d)",
        cmd_vel_topic_.c_str(), sport_request_topic_.c_str(), api_id_);
  }

private:
  void publishCommand(double x, double y, double yaw_rate)
  {
    unitree_api::msg::Request request;
    request.header.identity.api_id = api_id_;

    std::ostringstream json;
    json.imbue(std::locale::classic());
    json << std::setprecision(9)
         << "{\"x\":" << x
         << ",\"y\":" << y
         << ",\"z\":" << yaw_rate
         << '}';
    request.parameter = json.str();
    request_pub_->publish(request);
  }

  void forward(const geometry_msgs::msg::Twist::ConstSharedPtr & msg)
  {
    last_command_time_ = now();
    has_command_ = true;
    timed_out_ = false;
    publishCommand(msg->linear.x, msg->linear.y, msg->angular.z);
    if (!logged_first_command_) {
      logged_first_command_ = true;
      RCLCPP_INFO(
          get_logger(), "First command: linear.x=%.9g linear.y=%.9g angular.z=%.9g",
          msg->linear.x, msg->linear.y, msg->angular.z);
    }
  }

  void watchdogCallback()
  {
    if (!has_command_ || timed_out_)
      return;
    if ((now() - last_command_time_).seconds() < command_timeout_)
      return;

    publishCommand(0.0, 0.0, 0.0);
    timed_out_ = true;
    RCLCPP_WARN(
        get_logger(), "No /cmd_vel for %.2f s; published one zero command",
        command_timeout_);
  }

  std::string cmd_vel_topic_;
  std::string sport_request_topic_;
  int api_id_;
  double command_timeout_;
  bool logged_first_command_{false};
  bool has_command_{false};
  bool timed_out_{false};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};
}  // namespace cmd_vel_sport_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cmd_vel_sport_bridge::CmdVelSportBridge>());
  rclcpp::shutdown();
  return 0;
}
