#include "plan_manage/vlm_approval_panel.hpp"

#include <sstream>

#include <QVBoxLayout>
#include <json/json.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace scan_planner
{

VlmApprovalPanel::VlmApprovalPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  auto * title = new QLabel("VLM 导航审批", this);
  QFont title_font = title->font();
  title_font.setBold(true);
  title->setFont(title_font);
  layout->addWidget(title);

  status_label_ = new QLabel(
    "等待 VLM 候选点。候选生成后不会自动运动。", this);
  status_label_->setWordWrap(true);
  status_label_->setMinimumHeight(84);
  layout->addWidget(status_label_);

  approve_button_ = new QPushButton("批准并发送到 SCAN", this);
  approve_button_->setStyleSheet(
    "QPushButton { background-color: #218c3a; color: white; "
    "font-weight: bold; padding: 8px; }");
  approve_button_->setEnabled(false);
  layout->addWidget(approve_button_);

  reselect_button_ = new QPushButton("让 VLM 重选", this);
  reselect_button_->setStyleSheet(
    "QPushButton { background-color: #b35c00; color: white; "
    "font-weight: bold; padding: 8px; }");
  layout->addWidget(reselect_button_);
  layout->addStretch();

  connect(approve_button_, &QPushButton::clicked,
    this, &VlmApprovalPanel::approveGoal);
  connect(reselect_button_, &QPushButton::clicked,
    this, &VlmApprovalPanel::requestReselect);

  status_timer_ = new QTimer(this);
  status_timer_->setInterval(150);
  connect(status_timer_, &QTimer::timeout,
    this, &VlmApprovalPanel::refreshStatus);
  status_timer_->start();
}

void VlmApprovalPanel::onInitialize()
{
  auto ros_node = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node) {
    status_label_->setText("错误：RViz ROS 节点不可用。");
    approve_button_->setEnabled(false);
    reselect_button_->setEnabled(false);
    return;
  }

  auto node = ros_node->get_raw_node();
  command_pub_ = node->create_publisher<std_msgs::msg::String>(
    "/go2/vlm_approval", 10);
  rclcpp::QoS status_qos(1);
  status_qos.transient_local().reliable();
  status_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/vlm_pixel_shadow/status", status_qos,
    [this](std_msgs::msg::String::ConstSharedPtr message) {
      std::lock_guard<std::mutex> guard(status_mutex_);
      latest_status_ = message->data;
    });
}

void VlmApprovalPanel::publishCommand(const std::string & command)
{
  if (!command_pub_) {
    status_label_->setText("错误：审批命令发布器未初始化。");
    return;
  }
  std_msgs::msg::String message;
  message.data = command;
  command_pub_->publish(message);
}

void VlmApprovalPanel::approveGoal()
{
  if (!approve_button_->isEnabled()) {
    status_label_->setText("当前没有待批准的 VLM 候选点。");
    return;
  }
  publishCommand("approve");
  status_label_->setText("已提交批准，等待 VLM 节点发送 SCAN 目标。");
}

void VlmApprovalPanel::requestReselect()
{
  publishCommand("reselect");
  status_label_->setText("已请求重选，正在等待新的 VLM 候选点。");
}

void VlmApprovalPanel::refreshStatus()
{
  std::string status;
  {
    std::lock_guard<std::mutex> guard(status_mutex_);
    status = latest_status_;
  }
  if (status.empty()) {
    return;
  }

  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errors;
  std::istringstream stream(status);
  if (!Json::parseFromStream(builder, stream, &root, &errors)) {
    approve_button_->setEnabled(false);
    status_label_->setText(QString::fromStdString(status));
    return;
  }

  const bool approval_required =
    root.get("approval_required", false).asBool();
  approve_button_->setEnabled(approval_required);
  std::ostringstream display;
  if (approval_required) {
    display << "等待审批，狗保持停止。\n";
    if (root.isMember("pixel") && root["pixel"].isArray()) {
      display << "像素: [";
      for (Json::ArrayIndex index = 0; index < root["pixel"].size(); ++index) {
        if (index > 0) {
          display << ", ";
        }
        display << root["pixel"][index].asInt();
      }
      display << "]\n";
    }
    if (root.isMember("target_xyz") && root["target_xyz"].isArray()) {
      display << "世界点: [";
      for (Json::ArrayIndex index = 0;
        index < root["target_xyz"].size() && index < 2; ++index)
      {
        if (index > 0) {
          display << ", ";
        }
        display << root["target_xyz"][index].asFloat();
      }
      display << "]\n";
    }
    if (root.isMember("reason")) {
      display << "理由: " << root["reason"].asString();
    }
  } else if (root.get("goal_publishing", false).asBool()) {
    display << "已批准，目标已发送给 SCAN。\n";
    if (root.isMember("target_xyz") && root["target_xyz"].isArray()) {
      display << "目标: [";
      for (Json::ArrayIndex index = 0;
        index < root["target_xyz"].size() && index < 2; ++index)
      {
        if (index > 0) {
          display << ", ";
        }
        display << root["target_xyz"][index].asFloat();
      }
      display << "]";
    }
  } else if (root.get("reselecting", false).asBool()) {
    display << "已解除锁定，VLM 正在重新选点。";
  } else if (!root.get("ok", true).asBool()) {
    display << "VLM 查询失败: "
            << root.get("error", "unknown error").asString();
  } else if (!root.get("walkable", true).asBool()) {
    display << "VLM 判定当前不可走，等待下次重试。";
  } else {
    display << "VLM 正在生成候选点。";
  }
  status_label_->setText(QString::fromStdString(display.str()));
}

}  // namespace scan_planner

PLUGINLIB_EXPORT_CLASS(scan_planner::VlmApprovalPanel, rviz_common::Panel)
