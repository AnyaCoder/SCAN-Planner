#include <curl/curl.h>
#include <json/json.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

struct Detection {
  std::string label;
  float confidence = 0.0F;
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
};

using Clock = std::chrono::steady_clock;

size_t WriteString(void* contents, size_t size, size_t count, void* output) {
  const size_t bytes = size * count;
  auto* buffer = static_cast<std::string*>(output);
  buffer->append(static_cast<char*>(contents), bytes);
  return bytes;
}

std::string FetchUrl(CURL* curl, const std::string& url) {
  std::string data;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteString);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &data);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 800L);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 300L);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  const CURLcode result = curl_easy_perform(curl);
  if (result != CURLE_OK) {
    throw std::runtime_error(curl_easy_strerror(result));
  }
  return data;
}

std::vector<Detection> ParseDetections(const std::string& json) {
  Json::CharReaderBuilder builder;
  Json::CharReader* reader = builder.newCharReader();
  Json::Value root;
  std::string error;
  const bool parsed =
      reader->parse(json.data(), json.data() + json.size(), &root, &error);
  delete reader;
  if (!parsed) {
    throw std::runtime_error("invalid detection JSON: " + error);
  }

  std::vector<Detection> detections;
  const Json::Value& objects = root["objects"];
  if (!objects.isArray()) {
    return detections;
  }
  detections.reserve(objects.size());
  for (const Json::Value& object : objects) {
    const Json::Value& box = object["bbox"];
    if (!box.isArray() || box.size() != 4) {
      continue;
    }
    Detection detection;
    detection.label = object["class"].asString();
    detection.confidence = object["confidence"].asFloat();
    detection.x = box[0].asInt();
    detection.y = box[1].asInt();
    detection.width = box[2].asInt();
    detection.height = box[3].asInt();
    if (!detection.label.empty() && detection.width > 0 &&
        detection.height > 0) {
      detections.push_back(detection);
    }
  }
  return detections;
}

class DetectionPoller {
 public:
  DetectionPoller(std::string url, double interval, double stale_after)
      : url_(std::move(url)),
        interval_(interval),
        stale_after_(stale_after),
        curl_(curl_easy_init()) {
    if (!curl_) {
      throw std::runtime_error("curl_easy_init failed");
    }
    thread_ = std::thread(&DetectionPoller::Run, this);
  }

  ~DetectionPoller() { Stop(); }

  std::vector<Detection> Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (Clock::now() - received_ > std::chrono::duration<double>(stale_after_)) {
      return {};
    }
    return detections_;
  }

  void Stop() {
    stop_ = true;
    if (thread_.joinable()) {
      thread_.join();
    }
    if (curl_) {
      curl_easy_cleanup(curl_);
      curl_ = nullptr;
    }
  }

 private:
  void Run() {
    while (!stop_) {
      try {
        const std::string json = FetchUrl(curl_, url_);
        if (json != last_json_) {
          std::vector<Detection> parsed = ParseDetections(json);
          std::lock_guard<std::mutex> lock(mutex_);
          detections_ = std::move(parsed);
          received_ = Clock::now();
          last_json_ = json;
        }
      } catch (const std::exception&) {
      }
      const auto deadline =
          Clock::now() + std::chrono::duration<double>(interval_);
      while (!stop_ && Clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
  }

  std::string url_;
  double interval_;
  double stale_after_;
  CURL* curl_ = nullptr;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  mutable std::mutex mutex_;
  std::vector<Detection> detections_;
  Clock::time_point received_{};
  std::string last_json_;
};

class ImagePoller {
 public:
  ImagePoller(std::string url, double interval, double stale_after)
      : url_(std::move(url)),
        interval_(interval),
        stale_after_(stale_after),
        curl_(curl_easy_init()) {
    if (!curl_) {
      throw std::runtime_error("curl_easy_init failed");
    }
    thread_ = std::thread(&ImagePoller::Run, this);
  }

  ~ImagePoller() { Stop(); }

  cv::Mat Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (Clock::now() - received_ >
        std::chrono::duration<double>(stale_after_)) {
      return {};
    }
    return image_.clone();
  }

  void Stop() {
    stop_ = true;
    if (thread_.joinable()) {
      thread_.join();
    }
    if (curl_) {
      curl_easy_cleanup(curl_);
      curl_ = nullptr;
    }
  }

 private:
  void Run() {
    while (!stop_) {
      try {
        const std::string encoded = FetchUrl(curl_, url_);
        if (!encoded.empty()) {
          const std::vector<uchar> bytes(encoded.begin(), encoded.end());
          cv::Mat decoded = cv::imdecode(bytes, cv::IMREAD_COLOR);
          if (!decoded.empty()) {
            std::lock_guard<std::mutex> lock(mutex_);
            image_ = std::move(decoded);
            received_ = Clock::now();
          }
        }
      } catch (const std::exception&) {
      }
      const auto deadline =
          Clock::now() + std::chrono::duration<double>(interval_);
      while (!stop_ && Clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
  }

  std::string url_;
  double interval_;
  double stale_after_;
  CURL* curl_ = nullptr;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  mutable std::mutex mutex_;
  cv::Mat image_;
  Clock::time_point received_{};
};

cv::Scalar DetectionColor(const std::string& label) {
  static const cv::Scalar colors[] = {
      cv::Scalar(0, 210, 0),   cv::Scalar(0, 165, 255),
      cv::Scalar(255, 100, 80), cv::Scalar(80, 100, 255),
      cv::Scalar(255, 210, 0), cv::Scalar(220, 80, 220)};
  size_t hash = 0;
  for (const char value : label) {
    hash = hash * 131 + static_cast<unsigned char>(value);
  }
  return colors[hash % (sizeof(colors) / sizeof(colors[0]))];
}

void DrawDetections(cv::Mat* frame, const std::vector<Detection>& detections) {
  const int width = frame->cols;
  const int height = frame->rows;
  for (const Detection& detection : detections) {
    const int x1 = std::clamp(detection.x, 0, width - 1);
    const int y1 = std::clamp(detection.y, 0, height - 1);
    const int x2 =
        std::clamp(detection.x + detection.width, 0, width - 1);
    const int y2 =
        std::clamp(detection.y + detection.height, 0, height - 1);
    if (x2 <= x1 || y2 <= y1) {
      continue;
    }

    const cv::Scalar color = DetectionColor(detection.label);
    cv::rectangle(*frame, cv::Rect(x1, y1, x2 - x1, y2 - y1), color, 2);

    char text[128];
    std::snprintf(text, sizeof(text), "%s %.2f", detection.label.c_str(),
                  detection.confidence);
    int baseline = 0;
    const cv::Size text_size = cv::getTextSize(
        text, cv::FONT_HERSHEY_SIMPLEX, 0.55, 2, &baseline);
    const int label_height = text_size.height + baseline + 6;
    const int label_top = y1 >= label_height ? y1 - label_height : y1;
    const int label_bottom = std::min(height - 1, label_top + label_height);
    const int label_left =
        std::max(0, std::min(x1, width - 1 - text_size.width - 6));
    const int label_right =
        std::min(width - 1, label_left + text_size.width + 6);
    cv::rectangle(*frame, cv::Rect(label_left, label_top, label_right - label_left,
                                  label_bottom - label_top),
                  color, cv::FILLED);
    cv::putText(*frame, text,
                cv::Point(label_left + 3,
                          std::min(height - 4, label_top + text_size.height + 2)),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 2,
                cv::LINE_AA);
  }
}

class YoloOverlay : public rclcpp::Node {
 public:
  YoloOverlay(const std::string& input_topic, const std::string& output_topic,
              const std::string& detections_url,
              const std::string& semantic_url, const std::string& frame_id,
              double poll_interval, double semantic_interval,
              double stale_after, int output_width)
      : Node("go2_combined_camera_overlay"),
        frame_id_(frame_id),
        output_width_(output_width),
        poller_(detections_url, poll_interval, stale_after),
        semantic_poller_(semantic_url, semantic_interval, stale_after) {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    publisher_ = create_publisher<sensor_msgs::msg::Image>(output_topic, qos);
    subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        input_topic, qos,
        [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
          OnImage(std::move(message));
        });
    RCLCPP_INFO(get_logger(), "%s + SegFormer + YOLO -> %s",
                input_topic.c_str(), output_topic.c_str());
  }

  ~YoloOverlay() override {
    poller_.Stop();
    semantic_poller_.Stop();
  }

 private:
  void OnImage(sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
    cv::Mat frame = semantic_poller_.Snapshot();
    if (frame.empty()) {
      const std::vector<uchar> encoded(message->data.begin(),
                                      message->data.end());
      frame = cv::imdecode(encoded, cv::IMREAD_COLOR);
    }
    if (frame.empty()) {
      WarnThrottled("camera JPEG decode failed");
      return;
    }

    const std::vector<Detection> detections = poller_.Snapshot();
    if (!detections.empty()) {
      DrawDetections(&frame, detections);
    }
    if (output_width_ > 0 && frame.cols > output_width_) {
      const int output_height =
          static_cast<int>(std::lround(
              static_cast<double>(frame.rows) * output_width_ / frame.cols));
      cv::resize(frame, frame, cv::Size(output_width_, output_height), 0.0,
                 0.0, cv::INTER_AREA);
    }

    sensor_msgs::msg::Image output;
    output.header.stamp = message->header.stamp;
    output.header.frame_id =
        message->header.frame_id.empty() ? frame_id_ : message->header.frame_id;
    output.height = static_cast<uint32_t>(frame.rows);
    output.width = static_cast<uint32_t>(frame.cols);
    output.encoding = "bgr8";
    output.is_bigendian = 0;
    output.step = output.width * 3;
    output.data.resize(static_cast<size_t>(output.step) * output.height);
    std::memcpy(output.data.data(), frame.data, output.data.size());
    publisher_->publish(output);
  }

  void WarnThrottled(const std::string& text) {
    const auto now = Clock::now();
    if (now - last_warning_ < std::chrono::seconds(10)) {
      return;
    }
    last_warning_ = now;
    RCLCPP_WARN(get_logger(), "%s", text.c_str());
  }

  std::string frame_id_;
  int output_width_;
  DetectionPoller poller_;
  ImagePoller semantic_poller_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      subscription_;
  Clock::time_point last_warning_{};
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::string input_topic = "/go2/front_camera/image_raw/compressed";
  std::string output_topic = "/go2/front_camera/image_raw";
  const char* robot_host_env = std::getenv("GO2_WIFI_HOST");
  const std::string robot_host =
      robot_host_env == nullptr ? "192.168.77.18" : robot_host_env;
  std::string detections_url =
      "http://" + robot_host + ":8767/latest_detections.json";
  std::string semantic_url = "http://" + robot_host + ":8768/overlay.jpg";
  std::string frame_id = "go2_front_camera_optical_frame";
  double poll_interval = 0.1;
  double semantic_interval = 0.2;
  double stale_after = 0.8;
  int output_width = 0;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--input-topic" && index + 1 < argc) {
      input_topic = argv[++index];
    } else if (argument == "--output-topic" && index + 1 < argc) {
      output_topic = argv[++index];
    } else if (argument == "--detections-url" && index + 1 < argc) {
      detections_url = argv[++index];
    } else if (argument == "--semantic-url" && index + 1 < argc) {
      semantic_url = argv[++index];
    } else if (argument == "--frame-id" && index + 1 < argc) {
      frame_id = argv[++index];
    } else if (argument == "--poll-interval" && index + 1 < argc) {
      poll_interval = std::stod(argv[++index]);
    } else if (argument == "--semantic-interval" && index + 1 < argc) {
      semantic_interval = std::stod(argv[++index]);
    } else if (argument == "--stale-after" && index + 1 < argc) {
      stale_after = std::stod(argv[++index]);
    } else if (argument == "--output-width" && index + 1 < argc) {
      output_width = std::stoi(argv[++index]);
    }
  }
  if (poll_interval <= 0.0 || semantic_interval <= 0.0 ||
      stale_after <= 0.0 || output_width < 0) {
    std::cerr << "poll intervals and stale timeout must be positive, "
                 "output width must be non-negative\n";
    rclcpp::shutdown();
    return 2;
  }

  curl_global_init(CURL_GLOBAL_DEFAULT);
  try {
    auto node = std::make_shared<YoloOverlay>(
        input_topic, output_topic, detections_url, semantic_url, frame_id,
        poll_interval, semantic_interval, stale_after, output_width);
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    curl_global_cleanup();
    rclcpp::shutdown();
    return 1;
  }
  curl_global_cleanup();
  rclcpp::shutdown();
  return 0;
}
