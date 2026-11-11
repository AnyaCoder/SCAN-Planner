#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

namespace scan_planner
{

class VlmApprovalPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit VlmApprovalPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void approveGoal();
  void requestReselect();
  void refreshStatus();

private:
  void publishCommand(const std::string & command);

  QLabel * status_label_{nullptr};
  QPushButton * approve_button_{nullptr};
  QPushButton * reselect_button_{nullptr};
  QTimer * status_timer_{nullptr};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  std::mutex status_mutex_;
  std::string latest_status_;
};

}  // namespace scan_planner
