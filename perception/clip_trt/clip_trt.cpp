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
#include <vector>

namespace {

class Logger final : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* message) noexcept override {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT] " << message << '\n';
    }
  }
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

cv::Mat LoadImage(const std::string& source) {
  cv::Mat image;
  if (source.rfind("http://", 0) == 0 || source.rfind("https://", 0) == 0) {
    const std::string bytes = FetchUrl(source);
    const std::vector<uchar> encoded(bytes.begin(), bytes.end());
    image = cv::imdecode(encoded, cv::IMREAD_COLOR);
  } else {
    image = cv::imread(source, cv::IMREAD_COLOR);
  }
  if (image.empty()) {
    throw std::runtime_error("Cannot load image: " + source);
  }
  return image;
}

struct ClipConfig {
  int size;
  float mean[3];
  float std[3];
  float logit_scale;
  const char* output_binding;
};

ClipConfig GetClipConfig(const std::string& model) {
  if (model == "mobileclip") {
    return {256,
            {0.0f, 0.0f, 0.0f},
            {1.0f, 1.0f, 1.0f},
            64.329887f,
            "image_embeddings"};
  }
  if (model == "clip") {
    return {224,
            {0.48145466f, 0.4578275f, 0.40821073f},
            {0.26862954f, 0.26130258f, 0.27577711f},
            100.0f,
            "image_embeds"};
  }
  throw std::runtime_error("Unknown model: " + model);
}

std::vector<float> Preprocess(const cv::Mat& source, const ClipConfig& config) {
  const float scale =
      config.size / static_cast<float>(std::min(source.cols, source.rows));
  cv::Mat resized;
  cv::resize(source, resized,
             cv::Size(static_cast<int>(std::round(source.cols * scale)),
                      static_cast<int>(std::round(source.rows * scale))));
  const int left = (resized.cols - config.size) / 2;
  const int top = (resized.rows - config.size) / 2;
  cv::Mat crop = resized(cv::Rect(left, top, config.size, config.size));
  cv::Mat rgb;
  cv::cvtColor(crop, rgb, cv::COLOR_BGR2RGB);
  rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);

  std::vector<float> blob(3 * config.size * config.size);
  const int plane = config.size * config.size;
  for (int y = 0; y < config.size; ++y) {
    const auto* row = rgb.ptr<cv::Vec3f>(y);
    for (int x = 0; x < config.size; ++x) {
      for (int channel = 0; channel < 3; ++channel) {
        blob[channel * plane + y * config.size + x] =
            (row[x][channel] - config.mean[channel]) / config.std[channel];
      }
    }
  }
  return blob;
}

size_t Volume(const nvinfer1::Dims& dims) {
  size_t volume = 1;
  for (int index = 0; index < dims.nbDims; ++index) {
    volume *= static_cast<size_t>(dims.d[index]);
  }
  return volume;
}

class ClipEngine {
 public:
  ClipEngine(const std::string& engine_path, const char* output_binding) {
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
    engine_.reset(runtime_->deserializeCudaEngine(buffer.data(), size));
    if (!engine_) {
      throw std::runtime_error("deserializeCudaEngine failed");
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("createExecutionContext failed");
    }

    input_index_ = engine_->getBindingIndex("pixel_values");
    if (input_index_ < 0) {
      input_index_ = engine_->getBindingIndex("images");
    }
    output_index_ = engine_->getBindingIndex(output_binding);
    if (input_index_ < 0 || output_index_ < 0) {
      throw std::runtime_error("Unexpected CLIP engine bindings");
    }
    input_elements_ = Volume(engine_->getBindingDimensions(input_index_));
    output_elements_ = Volume(engine_->getBindingDimensions(output_index_));
    if (cudaStreamCreate(&stream_) != cudaSuccess ||
        cudaMalloc(&device_input_, input_elements_ * sizeof(float)) !=
            cudaSuccess ||
        cudaMalloc(&device_output_, output_elements_ * sizeof(float)) !=
            cudaSuccess) {
      throw std::runtime_error("CUDA allocation failed");
    }
    bindings_[input_index_] = device_input_;
    bindings_[output_index_] = device_output_;
  }

  ~ClipEngine() {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
    cudaFree(device_input_);
    cudaFree(device_output_);
  }

  std::vector<float> Infer(const std::vector<float>& input) {
    if (input.size() != input_elements_) {
      throw std::runtime_error("CLIP input size mismatch");
    }
    cudaMemcpyAsync(device_input_, input.data(),
                    input_elements_ * sizeof(float), cudaMemcpyHostToDevice,
                    stream_);
    if (!context_->enqueueV2(bindings_, stream_, nullptr)) {
      throw std::runtime_error("TensorRT enqueueV2 failed");
    }
    std::vector<float> output(output_elements_);
    cudaMemcpyAsync(output.data(), device_output_,
                    output_elements_ * sizeof(float), cudaMemcpyDeviceToHost,
                    stream_);
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
  size_t input_elements_ = 0;
  size_t output_elements_ = 0;
};

std::vector<std::string> ReadLabels(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Cannot open labels: " + path);
  }
  std::vector<std::string> labels;
  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (!line.empty()) {
      labels.push_back(line);
    }
  }
  return labels;
}

std::vector<float> ReadEmbeddings(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("Cannot open embeddings: " + path);
  }
  input.seekg(0, std::ios::end);
  const std::streamsize bytes = input.tellg();
  input.seekg(0, std::ios::beg);
  std::vector<float> values(bytes / sizeof(float));
  input.read(reinterpret_cast<char*>(values.data()), bytes);
  return values;
}

void WriteJson(const std::string& path, const std::string& model,
               const std::vector<std::pair<std::string, float>>& labels,
               double engine_load_ms, double preprocess_ms,
               double inference_ms) {
  const std::string temporary_path = path + ".tmp";
  std::ofstream output(temporary_path);
  const double timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  output << std::fixed << std::setprecision(3)
         << "{\"timestamp\":" << timestamp << ",\"model\":\"" << model
         << "\",\"engine_load_ms\":" << engine_load_ms
         << ",\"preprocess_ms\":" << preprocess_ms
         << ",\"inference_ms\":" << inference_ms
         << ",\"latency_ms\":" << preprocess_ms + inference_ms
         << ",\"scene\":[";
  for (size_t index = 0; index < labels.size(); ++index) {
    if (index != 0) {
      output << ',';
    }
    output << "{\"label\":\"" << labels[index].first
           << "\",\"score\":" << labels[index].second << "}";
  }
  output << "]}";
  output.close();
  std::rename(temporary_path.c_str(), path.c_str());
}

}  // namespace

int main(int argc, char** argv) {
  std::string engine_path;
  std::string model = "mobileclip";
  std::string image_source = "http://127.0.0.1:8765/frame.jpg";
  std::string labels_path =
      "/home/unitree/SCAN-Planner/perception/models/mobileclip2_s0/labels.txt";
  std::string embeddings_path =
      "/home/unitree/SCAN-Planner/perception/models/mobileclip2_s0/"
      "text_embeddings.bin";
  std::string json_path;
  int crop_x = 0;
  int crop_y = 0;
  int crop_width = 0;
  int crop_height = 0;
  int top_k = 5;
  int bench = 1;
  float logit_scale = 0.0f;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--engine" && index + 1 < argc) {
      engine_path = argv[++index];
    } else if (argument == "--model" && index + 1 < argc) {
      model = argv[++index];
    } else if (argument == "--image" && index + 1 < argc) {
      image_source = argv[++index];
    } else if (argument == "--labels" && index + 1 < argc) {
      labels_path = argv[++index];
    } else if (argument == "--embeddings" && index + 1 < argc) {
      embeddings_path = argv[++index];
    } else if (argument == "--json" && index + 1 < argc) {
      json_path = argv[++index];
    } else if (argument == "--crop" && index + 4 < argc) {
      crop_x = std::stoi(argv[++index]);
      crop_y = std::stoi(argv[++index]);
      crop_width = std::stoi(argv[++index]);
      crop_height = std::stoi(argv[++index]);
    } else if (argument == "--topk" && index + 1 < argc) {
      top_k = std::stoi(argv[++index]);
    } else if (argument == "--bench" && index + 1 < argc) {
      bench = std::max(1, std::stoi(argv[++index]));
    } else if (argument == "--logit-scale" && index + 1 < argc) {
      logit_scale = std::stof(argv[++index]);
    } else {
      std::cerr << "Unknown argument: " << argument << '\n';
      return 2;
    }
  }
  if (engine_path.empty()) {
    std::cerr << "--engine is required\n";
    return 2;
  }

  ClipConfig config = GetClipConfig(model);
  if (logit_scale > 0.0f) {
    config.logit_scale = logit_scale;
  }
  curl_global_init(CURL_GLOBAL_DEFAULT);
  try {
    const auto preprocess_started = std::chrono::steady_clock::now();
    cv::Mat image = LoadImage(image_source);
    if (crop_width > 0 && crop_height > 0) {
      image = image(cv::Rect(
          std::max(0, std::min(crop_x, image.cols - 1)),
          std::max(0, std::min(crop_y, image.rows - 1)),
          std::min(crop_width, image.cols - std::max(0, crop_x)),
          std::min(crop_height, image.rows - std::max(0, crop_y))));
    }
    const std::vector<float> input = Preprocess(image, config);
    const std::vector<std::string> labels = ReadLabels(labels_path);
    const std::vector<float> text = ReadEmbeddings(embeddings_path);
    if (text.size() % labels.size() != 0) {
      throw std::runtime_error("Embedding/label size mismatch");
    }
    const size_t dimension = text.size() / labels.size();
    const double preprocess_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - preprocess_started)
            .count();

    const auto engine_started = std::chrono::steady_clock::now();
    ClipEngine engine(engine_path, config.output_binding);
    const double engine_load_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - engine_started)
            .count();

    std::vector<float> output;
    double inference_ms = 0.0;
    if (bench > 1) {
      engine.Infer(input);
      const auto inference_started = std::chrono::steady_clock::now();
      for (int run = 0; run < bench; ++run) {
        output = engine.Infer(input);
      }
      inference_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - inference_started)
              .count() /
          bench;
    } else {
      const auto inference_started = std::chrono::steady_clock::now();
      output = engine.Infer(input);
      inference_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - inference_started)
              .count();
    }
    if (output.size() != dimension) {
      throw std::runtime_error("CLIP output dimension mismatch");
    }

    std::vector<std::pair<std::string, float>> scores;
    scores.reserve(labels.size());
    float norm = 0.0f;
    for (const float value : output) {
      norm += value * value;
    }
    norm = std::sqrt(std::max(norm, 1e-12f));
    float maximum = -1e30f;
    std::vector<float> logits(labels.size());
    for (size_t label = 0; label < labels.size(); ++label) {
      float dot = 0.0f;
      for (size_t index = 0; index < dimension; ++index) {
        dot += text[label * dimension + index] * output[index] / norm;
      }
      logits[label] = dot * config.logit_scale;
      maximum = std::max(maximum, logits[label]);
    }
    float total = 0.0f;
    for (float& value : logits) {
      value = std::exp(value - maximum);
      total += value;
    }
    for (size_t label = 0; label < labels.size(); ++label) {
      scores.emplace_back(labels[label], logits[label] / total);
    }
    std::sort(scores.begin(), scores.end(),
              [](const auto& left, const auto& right) {
                return left.second > right.second;
              });
    if (top_k > 0 && static_cast<size_t>(top_k) < scores.size()) {
      scores.resize(top_k);
    }

    std::cout << std::fixed << std::setprecision(3) << "model=" << model
              << " engine_load_ms=" << engine_load_ms
              << " preprocess_ms=" << preprocess_ms
              << " inference_ms=" << inference_ms
              << " latency_ms=" << preprocess_ms + inference_ms << '\n';
    for (const auto& item : scores) {
      std::cout << "  " << item.first << " " << item.second << '\n';
    }
    if (!json_path.empty()) {
      WriteJson(json_path, model, scores, engine_load_ms, preprocess_ms,
                inference_ms);
    }
  } catch (const std::exception& error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    curl_global_cleanup();
    return 1;
  }
  curl_global_cleanup();
  return 0;
}
