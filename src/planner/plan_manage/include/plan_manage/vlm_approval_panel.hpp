#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QSize>
#include <QString>
#include <QTextEdit>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/image.hpp>
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
  void clearMemory();
  void refreshStatus();
  void topdownCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void streamCallback(
    const std_msgs::msg::String::ConstSharedPtr msg);

private:
  void publishCommand(const std::string & command);
  void refreshMedia();

  QLabel * status_label_{nullptr};
  QLabel * topdown_label_{nullptr};
  QTextEdit * stream_view_{nullptr};
  QPushButton * approve_button_{nullptr};
  QPushButton * reselect_button_{nullptr};
  QPushButton * clear_memory_button_{nullptr};
  QTimer * status_timer_{nullptr};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr topdown_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stream_sub_;
  std::mutex status_mutex_;
  std::string latest_status_;
  std::mutex media_mutex_;
  QImage latest_topdown_;
  bool topdown_dirty_{false};
  QSize rendered_topdown_size_;
  bool clear_stream_{false};
  std::deque<QString> stream_chunks_;
};

}  // namespace scan_planner
