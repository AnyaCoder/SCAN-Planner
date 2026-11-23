#include "plan_manage/vlm_approval_panel.hpp"

#include <sstream>

#include <QDockWidget>
#include <QMainWindow>
#include <QPixmap>
#include <QSizePolicy>
#include <QTextCursor>
#include <QTextDocument>
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

  // Candidate details are shown once in the streaming view. Keep the status
  // widget for internal command feedback, but do not duplicate it in the UI.
  status_label_ = new QLabel(this);
  status_label_->hide();

  auto * topdown_title = new QLabel("10 m 俯视导航", this);
  QFont section_font = topdown_title->font();
  section_font.setBold(true);
  topdown_title->setFont(section_font);
  layout->addWidget(topdown_title);

  topdown_label_ = new QLabel("等待俯视图", this);
  topdown_label_->setAlignment(Qt::AlignCenter);
  topdown_label_->setMinimumSize(320, 320);
  topdown_label_->setSizePolicy(
    QSizePolicy::Expanding, QSizePolicy::Expanding);
  topdown_label_->setStyleSheet(
    "QLabel { background-color: #090c0a; color: #7f947b; "
    "border: 1px solid #334433; }");
  layout->addWidget(topdown_label_, 1);

  auto * stream_title = new QLabel("VLM 流式输出", this);
  stream_title->setFont(section_font);
  layout->addWidget(stream_title);

  stream_view_ = new QTextEdit(this);
  stream_view_->setReadOnly(true);
  stream_view_->setMinimumHeight(150);
  stream_view_->document()->setMaximumBlockCount(5000);
  stream_view_->setPlaceholderText("等待远程 VLM 输出...");
  layout->addWidget(stream_view_);

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

  clear_memory_button_ = new QPushButton("清空 VLM 记忆", this);
  clear_memory_button_->setStyleSheet(
    "QPushButton { background-color: #4a4f52; color: white; "
    "font-weight: bold; padding: 8px; }");
  layout->addWidget(clear_memory_button_);
  layout->addStretch();

  connect(approve_button_, &QPushButton::clicked,
    this, &VlmApprovalPanel::approveGoal);
  connect(reselect_button_, &QPushButton::clicked,
    this, &VlmApprovalPanel::requestReselect);
  connect(clear_memory_button_, &QPushButton::clicked,
    this, &VlmApprovalPanel::clearMemory);

  status_timer_ = new QTimer(this);
  status_timer_->setInterval(150);
  connect(status_timer_, &QTimer::timeout,
    this, &VlmApprovalPanel::refreshStatus);
  status_timer_->start();
}

void VlmApprovalPanel::onInitialize()
{
  QWidget * ancestor = parentWidget();
  while (ancestor != nullptr) {
    auto * dock = qobject_cast<QDockWidget *>(ancestor);
    if (dock != nullptr) {
      auto * main_window =
        qobject_cast<QMainWindow *>(dock->window());
      if (main_window != nullptr) {
        main_window->addDockWidget(Qt::RightDockWidgetArea, dock);
      }
      break;
    }
    ancestor = ancestor->parentWidget();
  }

  auto ros_node = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node) {
    status_label_->setText("错误：RViz ROS 节点不可用。");
    approve_button_->setEnabled(false);
    reselect_button_->setEnabled(false);
    clear_memory_button_->setEnabled(false);
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

  rclcpp::QoS image_qos(1);
  image_qos.transient_local().reliable();
  topdown_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    "/vlm_navigation/topdown", image_qos,
    [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
      topdownCallback(message);
    });
  rclcpp::QoS stream_qos(500);
  stream_qos.transient_local().reliable();
  stream_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/vlm_pixel_shadow/stream", stream_qos,
    [this](std_msgs::msg::String::ConstSharedPtr message) {
      streamCallback(message);
    });
}

void VlmApprovalPanel::topdownCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (msg->width == 0 || msg->height == 0 || msg->data.empty()) {
    return;
  }
  QImage image;
  if (msg->encoding == "bgr8") {
    const QImage view(
      msg->data.data(), static_cast<int>(msg->width),
      static_cast<int>(msg->height), static_cast<int>(msg->step),
      QImage::Format_RGB888);
    if (!view.isNull()) {
      image = view.rgbSwapped();
    }
  } else if (msg->encoding == "rgb8") {
    const QImage view(
      msg->data.data(), static_cast<int>(msg->width),
      static_cast<int>(msg->height), static_cast<int>(msg->step),
      QImage::Format_RGB888);
    if (!view.isNull()) {
      image = view.copy();
    }
  }
  if (image.isNull()) {
    return;
  }
  std::lock_guard<std::mutex> guard(media_mutex_);
  latest_topdown_ = image;
  topdown_dirty_ = true;
}

void VlmApprovalPanel::streamCallback(
  const std_msgs::msg::String::ConstSharedPtr msg)
{
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errors;
  std::istringstream stream(msg->data);
  if (!Json::parseFromStream(builder, stream, &root, &errors)) {
    return;
  }
  const std::string type = root.get("type", "").asString();
  std::lock_guard<std::mutex> guard(media_mutex_);
  if (type == "reset") {
    clear_stream_ = true;
    stream_chunks_.clear();
  } else if (type == "delta") {
    const std::string text = root.get("text", "").asString();
    if (!text.empty()) {
      stream_chunks_.push_back(
        QString::fromUtf8(text.data(), static_cast<int>(text.size())));
    }
  } else if (type == "done") {
    stream_chunks_.push_back("\n\n[VLM 输出完成]\n");
  } else if (type == "coords") {
    QString message = "\n[落点已读出";
    if (root.isMember("pixel") && root["pixel"].isArray()) {
      message += QString(" | pixel [%1, %2]")
        .arg(root["pixel"][0].asInt())
        .arg(root["pixel"][1].asInt());
    } else if (root.isMember("action")) {
      message += QString(" | %1 %2 deg")
        .arg(
          QString::fromStdString(
            root["action"].asString()))
        .arg(root.get("heading_delta_deg", 0.0).asFloat(), 0, 'f', 0);
    }
    if (root.isMember("target_distance_m")) {
      message += QString(" | 距狗 %1 m")
        .arg(root["target_distance_m"].asFloat(), 0, 'f', 2);
    }
    stream_chunks_.push_back(message + "]\n");
  } else if (type == "error") {
    stream_chunks_.push_back(
      "\n[VLM 请求失败] " +
      QString::fromStdString(root.get("text", "").asString()) + "\n");
  } else if (type == "memory_cleared") {
    clear_stream_ = true;
    stream_chunks_.clear();
    stream_chunks_.push_back("[VLM 记忆已清空]\n");
  }
}

void VlmApprovalPanel::refreshMedia()
{
  std::lock_guard<std::mutex> guard(media_mutex_);
  if (clear_stream_) {
    stream_view_->clear();
    clear_stream_ = false;
  }
  while (!stream_chunks_.empty()) {
    stream_view_->moveCursor(QTextCursor::End);
    stream_view_->insertPlainText(stream_chunks_.front());
    stream_chunks_.pop_front();
  }

  if (!latest_topdown_.isNull() &&
    (topdown_dirty_ || rendered_topdown_size_ != topdown_label_->size()))
  {
    topdown_label_->setPixmap(
      QPixmap::fromImage(latest_topdown_).scaled(
        topdown_label_->size(), Qt::KeepAspectRatio,
        Qt::SmoothTransformation));
    rendered_topdown_size_ = topdown_label_->size();
    topdown_dirty_ = false;
  }
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
    status_label_->setText("当前没有待批准的 VLM 落点。");
    return;
  }
  publishCommand("approve");
  status_label_->setText("已提交批准，等待 VLM 节点发送 SCAN 目标。");
}

void VlmApprovalPanel::requestReselect()
{
  publishCommand("reselect");
  status_label_->setText("已请求重选，正在等待新的 VLM 落点。");
}

void VlmApprovalPanel::clearMemory()
{
  publishCommand("clear_memory");
  status_label_->setText("已请求清空 VLM 记忆。");
}

void VlmApprovalPanel::refreshStatus()
{
  refreshMedia();
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
  const QString action = QString::fromStdString(
    root.get("recommended_action", "").asString());
  const float heading_delta_deg =
    root.get("heading_delta_deg", 0.0).asFloat();
  approve_button_->setEnabled(approval_required);
  approve_button_->setText("批准该落点并发送到 SCAN");
  std::ostringstream display;
  if (root.get("preliminary", false).asBool()) {
    display << "VLM 落点已读出并显示，后台正在完成校验。\n";
    if (!action.isEmpty()) {
      display << "动作: " << action.toStdString();
      if (action.startsWith("turn")) {
        display << " | 转角: " << heading_delta_deg << " deg";
      }
      display << "\n";
    }
    if (root.isMember("pixel") && root["pixel"].isArray()) {
      display << "像素: [";
      for (Json::ArrayIndex index = 0; index < root["pixel"].size(); ++index) {
        if (index > 0) {
          display << ", ";
        }
        display << root["pixel"][index].asInt();
      }
      display << "]";
    }
  } else if (approval_required) {
    display << "等待审批，狗保持停止。\n";
    if (!action.isEmpty()) {
      display << "动作: " << action.toStdString();
      if (action.startsWith("turn")) {
        display << " | 转角: " << heading_delta_deg << " deg";
      }
      display << "\n";
    }
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
    display << "正在重采相机和 10 m 俯视图，等待 VLM 新落点。";
  } else if (!root.get("ok", true).asBool()) {
    display << "VLM 查询失败: "
            << root.get("error", "unknown error").asString();
  } else if (!root.get("walkable", true).asBool()) {
    display << "VLM 判定当前不可走，等待下次重试。";
  } else {
    display << "VLM 正在分析相机和俯视图并生成落点。";
  }
  status_label_->setText(QString::fromStdString(display.str()));
}

}  // namespace scan_planner

PLUGINLIB_EXPORT_CLASS(scan_planner::VlmApprovalPanel, rviz_common::Panel)
