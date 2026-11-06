#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <curl/curl.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr int kInputWidth = 640;
constexpr int kInputHeight = 640;
constexpr int kOutputElements = 84 * 8400;
constexpr float kDefaultConfidence = 0.25f;
constexpr float kDefaultNms = 0.45f;

const char* kCocoNames[] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"};

class Logger final : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* message) noexcept override {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT] " << message << '\n';
    }
  }
};

struct Detection {
  int class_id;
  float score;
  cv::Rect box;
};

struct Letterbox {
  cv::Mat image;
  float scale;
  float pad_x;
  float pad_y;
};

size_t WriteString(void* contents, size_t size, size_t count, void* output) {
  const size_t bytes = size * count;
  auto* buffer = static_cast<std::string*>(output);
  buffer->append(static_cast<char*>(contents), bytes);
  return bytes;
}

std::string FetchUrl(const std::string& url) {
  CURL* curl = curl_easy_init();
  if (curl == nullptr) {
    throw std::runtime_error("curl_easy_init failed");
  }
  std::string data;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteString);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &data);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 2000L);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 500L);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  const CURLcode result = curl_easy_perform(curl);
  curl_easy_cleanup(curl);
  if (result != CURLE_OK) {
    throw std::runtime_error(std::string("curl: ") +
                             curl_easy_strerror(result));
  }
  return data;
}

cv::Mat DecodeJpeg(const std::string& bytes) {
  const std::vector<uchar> encoded(bytes.begin(), bytes.end());
  cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
  if (image.empty()) {
    throw std::runtime_error("JPEG decode failed");
  }
  return image;
}

Letterbox ResizeLetterbox(const cv::Mat& source) {
  const float scale =
      std::min(kInputWidth / static_cast<float>(source.cols),
               kInputHeight / static_cast<float>(source.rows));
  const int width = static_cast<int>(std::round(source.cols * scale));
  const int height = static_cast<int>(std::round(source.rows * scale));
  const int pad_x = (kInputWidth - width) / 2;
  const int pad_y = (kInputHeight - height) / 2;

  cv::Mat output(kInputHeight, kInputWidth, CV_8UC3, cv::Scalar(114, 114, 114));
  cv::Mat resized;
  cv::resize(source, resized, cv::Size(width, height));
  resized.copyTo(output(cv::Rect(pad_x, pad_y, width, height)));
  return {output, scale, static_cast<float>(pad_x), static_cast<float>(pad_y)};
}

std::vector<Detection> DecodeDetections(const std::vector<float>& output,
                                        const Letterbox& letterbox,
                                        const cv::Size& original,
                                        float confidence_threshold,
                                        float nms_threshold) {
  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  std::vector<int> class_ids;

  for (int index = 0; index < 8400; ++index) {
    int best_class = 0;
    float best_score = output[4 * 8400 + index];
    for (int class_id = 1; class_id < 80; ++class_id) {
      const float score = output[(4 + class_id) * 8400 + index];
      if (score > best_score) {
        best_score = score;
        best_class = class_id;
      }
    }
    if (best_score < confidence_threshold) {
      continue;
    }

    const float center_x = output[index];
    const float center_y = output[8400 + index];
    const float width = output[2 * 8400 + index];
    const float height = output[3 * 8400 + index];
    const float left =
        (center_x - width * 0.5f - letterbox.pad_x) / letterbox.scale;
    const float top =
        (center_y - height * 0.5f - letterbox.pad_y) / letterbox.scale;
    const float scaled_width = width / letterbox.scale;
    const float scaled_height = height / letterbox.scale;

    const int x1 = std::max(0, std::min(original.width - 1,
                                        static_cast<int>(std::round(left))));
    const int y1 = std::max(0, std::min(original.height - 1,
                                        static_cast<int>(std::round(top))));
    const int x2 = std::max(0, std::min(original.width,
                                        static_cast<int>(std::round(left + scaled_width))));
    const int y2 = std::max(0, std::min(original.height,
                                        static_cast<int>(std::round(top + scaled_height))));
    if (x2 <= x1 || y2 <= y1) {
      continue;
    }
    boxes.emplace_back(x1, y1, x2 - x1, y2 - y1);
    scores.push_back(best_score);
    class_ids.push_back(best_class);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, scores, confidence_threshold, nms_threshold,
                    indices);
  std::vector<Detection> detections;
  for (const int index : indices) {
    detections.push_back({class_ids[index], scores[index], boxes[index]});
  }
  return detections;
}

void DrawDetections(cv::Mat* image, const std::vector<Detection>& detections) {
  static const cv::Scalar colors[] = {
      cv::Scalar(0, 200, 0), cv::Scalar(0, 160, 255), cv::Scalar(255, 80, 80),
      cv::Scalar(80, 80, 255), cv::Scalar(255, 200, 0)};
  for (const Detection& detection : detections) {
    const cv::Scalar& color = colors[detection.class_id % 5];
    cv::rectangle(*image, detection.box, color, 2);
    char text[128];
    std::snprintf(text, sizeof(text), "%s %.2f",
                  kCocoNames[detection.class_id], detection.score);
    int baseline = 0;
    const cv::Size size =
        cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.55, 2, &baseline);
    cv::rectangle(*image,
                  cv::Rect(detection.box.x,
                           std::max(0, detection.box.y - size.height - 8),
                           size.width + 6, size.height + 8),
                  color, cv::FILLED);
    cv::putText(*image, text,
                cv::Point(detection.box.x + 3,
                          std::max(size.height + 2, detection.box.y - 5)),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 2);
  }
}

void WriteJson(const std::string& path,
               const std::vector<Detection>& detections,
               double total_ms) {
  const std::string temporary_path = path + ".tmp";
  std::ofstream output(temporary_path);
  const double timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  output << std::fixed << std::setprecision(3)
         << "{\"timestamp\":" << timestamp
         << ",\"latency_ms\":" << total_ms << ",\"objects\":[";
  for (size_t index = 0; index < detections.size(); ++index) {
    const Detection& detection = detections[index];
    if (index != 0) {
      output << ',';
    }
    output << "{\"class\":\"" << kCocoNames[detection.class_id]
           << "\",\"confidence\":" << detection.score << ",\"bbox\":["
           << detection.box.x << ',' << detection.box.y << ','
           << detection.box.width << ',' << detection.box.height << "]}";
  }
  output << "]}";
  output.close();
  std::rename(temporary_path.c_str(), path.c_str());
}

class YoloEngine {
 public:
  explicit YoloEngine(const std::string& engine_path)
      : input_bytes_(3 * kInputWidth * kInputHeight * sizeof(float)),
        output_bytes_(kOutputElements * sizeof(float)) {
    std::ifstream file(engine_path, std::ios::binary);
    if (!file) {
      throw std::runtime_error("Cannot open engine: " + engine_path);
    }
    file.seekg(0, std::ios::end);
    const std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    if (!file.read(buffer.data(), size)) {
      throw std::runtime_error("Cannot read engine: " + engine_path);
    }

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      throw std::runtime_error("createInferRuntime failed");
    }
    engine_.reset(runtime_->deserializeCudaEngine(buffer.data(), size));
    if (!engine_) {
      throw std::runtime_error("deserializeCudaEngine failed");
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("createExecutionContext failed");
    }

    input_index_ = engine_->getBindingIndex("images");
    output_index_ = engine_->getBindingIndex("output0");
    if (input_index_ < 0 || output_index_ < 0) {
      throw std::runtime_error("Unexpected engine bindings");
    }
    if (cudaStreamCreate(&stream_) != cudaSuccess ||
        cudaMalloc(&device_input_, input_bytes_) != cudaSuccess ||
        cudaMalloc(&device_output_, output_bytes_) != cudaSuccess) {
      throw std::runtime_error("CUDA allocation failed");
    }
    bindings_[input_index_] = device_input_;
    bindings_[output_index_] = device_output_;
  }

  ~YoloEngine() {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
    cudaFree(device_input_);
    cudaFree(device_output_);
  }

  std::vector<float> Infer(const cv::Mat& blob) {
    cudaMemcpyAsync(device_input_, blob.ptr<float>(), input_bytes_,
                    cudaMemcpyHostToDevice, stream_);
    if (!context_->enqueueV2(bindings_, stream_, nullptr)) {
      throw std::runtime_error("TensorRT enqueueV2 failed");
    }
    std::vector<float> output(kOutputElements);
    cudaMemcpyAsync(output.data(), device_output_, output_bytes_,
                    cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);
    return output;
  }

 private:
  Logger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  cudaStream_t stream_ = nullptr;
  void* device_input_ = nullptr;
  void* device_output_ = nullptr;
  void* bindings_[2] = {nullptr, nullptr};
  int input_index_ = -1;
  int output_index_ = -1;
  size_t input_bytes_;
  size_t output_bytes_;
};

double ElapsedMs(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

}  // namespace

int main(int argc, char** argv) {
  std::string engine_path =
      "/home/unitree/SCAN-Planner/perception/engines/yolov8n_fp16.engine";
  std::string url = "http://127.0.0.1:8765/frame.jpg";
  std::string image_path =
      "/home/unitree/SCAN-Planner/perception/data/latest_detected.jpg";
  std::string json_path =
      "/home/unitree/SCAN-Planner/perception/data/latest_detections.json";
  int frames = 0;
  double interval = 0.2;
  float confidence = kDefaultConfidence;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--engine" && index + 1 < argc) {
      engine_path = argv[++index];
    } else if (argument == "--url" && index + 1 < argc) {
      url = argv[++index];
    } else if (argument == "--image" && index + 1 < argc) {
      image_path = argv[++index];
    } else if (argument == "--json" && index + 1 < argc) {
      json_path = argv[++index];
    } else if (argument == "--frames" && index + 1 < argc) {
      frames = std::stoi(argv[++index]);
    } else if (argument == "--interval" && index + 1 < argc) {
      interval = std::stod(argv[++index]);
    } else if (argument == "--confidence" && index + 1 < argc) {
      confidence = std::stof(argv[++index]);
    } else {
      std::cerr << "Unknown argument: " << argument << '\n';
      return 2;
    }
  }

  curl_global_init(CURL_GLOBAL_DEFAULT);
  try {
    YoloEngine model(engine_path);
    int frame_index = 0;
    while (frames <= 0 || frame_index < frames) {
      const auto start = std::chrono::steady_clock::now();
      const auto fetch_start = start;
      const std::string bytes = FetchUrl(url);
      const double fetch_ms = ElapsedMs(fetch_start);
      const auto decode_start = std::chrono::steady_clock::now();
      const cv::Mat image = DecodeJpeg(bytes);
      const double decode_ms = ElapsedMs(decode_start);

      const auto preprocess_start = std::chrono::steady_clock::now();
      const Letterbox letterbox = ResizeLetterbox(image);
      cv::Mat blob;
      cv::dnn::blobFromImage(letterbox.image, blob, 1.0 / 255.0,
                             cv::Size(kInputWidth, kInputHeight),
                             cv::Scalar(), true, false, CV_32F);
      const double preprocess_ms = ElapsedMs(preprocess_start);

      const auto inference_start = std::chrono::steady_clock::now();
      const std::vector<float> output = model.Infer(blob);
      const double inference_ms = ElapsedMs(inference_start);

      const auto postprocess_start = std::chrono::steady_clock::now();
      const std::vector<Detection> detections =
          DecodeDetections(output, letterbox, image.size(), confidence,
                           kDefaultNms);
      const double postprocess_ms = ElapsedMs(postprocess_start);
      const double total_ms = ElapsedMs(start);

      cv::Mat annotated = image.clone();
      DrawDetections(&annotated, detections);
      cv::imwrite(image_path, annotated);
      WriteJson(json_path, detections, total_ms);

      std::cout << "frame=" << frame_index
                << " objects=" << detections.size()
                << " fetch_ms=" << fetch_ms
                << " decode_ms=" << decode_ms
                << " preprocess_ms=" << preprocess_ms
                << " inference_ms=" << inference_ms
                << " postprocess_ms=" << postprocess_ms
                << " total_ms=" << total_ms << '\n';
      for (const Detection& detection : detections) {
        std::cout << "  " << kCocoNames[detection.class_id] << " "
                  << detection.score << " [" << detection.box.x << ","
                  << detection.box.y << "," << detection.box.width << ","
                  << detection.box.height << "]\n";
      }
      std::cout.flush();

      ++frame_index;
      if (frames <= 0 || frame_index < frames) {
        const double remaining =
            interval * 1000.0 - ElapsedMs(start);
        if (remaining > 0) {
          std::this_thread::sleep_for(
              std::chrono::milliseconds(static_cast<int>(remaining)));
        }
      }
    }
  } catch (const std::exception& error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    curl_global_cleanup();
    return 1;
  }
  curl_global_cleanup();
  return 0;
}
