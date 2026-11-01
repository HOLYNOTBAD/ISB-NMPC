#include "yolo/yolo_detector.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "backend/detector_backend.h"
#include "config.h"
#include "postprocess.h"
#include "preprocess.h"
#include "types.h"

namespace yolo {
namespace {

constexpr int kOutputSize =
    kMaxNumOutputBbox * sizeof(::Detection) / sizeof(float) + 1;

std::mutex& preprocessLifecycleMutex() {
  static std::mutex mutex;
  return mutex;
}

int& preprocessUserCount() {
  static int users = 0;
  return users;
}

void acquirePreprocess() {
  std::lock_guard<std::mutex> lock(preprocessLifecycleMutex());
  if (preprocessUserCount()++ == 0) {
    cuda_preprocess_init(kMaxInputImageSize);
  }
}

void releasePreprocess() {
  std::lock_guard<std::mutex> lock(preprocessLifecycleMutex());
  if (--preprocessUserCount() == 0) {
    cuda_preprocess_destroy();
  }
}

}  // namespace

class YoloDetector::Impl {
 public:
  Impl(const std::string& engine_file, float confidence_threshold,
       float nms_threshold)
      : confidence_threshold_(confidence_threshold),
        nms_threshold_(nms_threshold),
        backend_(createDetectorBackend()) {
    if (engine_file.empty()) {
      throw std::invalid_argument("YOLO engine_file is empty");
    }
    if (confidence_threshold_ < 0.0F || confidence_threshold_ > 1.0F ||
        nms_threshold_ < 0.0F || nms_threshold_ > 1.0F) {
      throw std::invalid_argument("YOLO confidence/NMS threshold is invalid");
    }
    const cudaError_t device_result = cudaSetDevice(kGpuId);
    if (device_result != cudaSuccess) {
      throw std::runtime_error(std::string("cudaSetDevice failed: ") +
                               cudaGetErrorString(device_result));
    }
    backend_->initialize(engine_file);
    acquirePreprocess();
    owns_preprocess_ = true;
  }

  ~Impl() {
    if (owns_preprocess_) {
      releasePreprocess();
    }
  }

  std::vector<DetectionBox> detect(const cv::Mat& image) {
    if (image.empty()) {
      return {};
    }

    std::lock_guard<std::mutex> lock(inference_mutex_);
    // OpenCV ROI views retain the parent image row stride and are therefore
    // usually non-contiguous.  The CUDA preprocessor currently copies
    // width * height * channels bytes and assumes tightly packed rows, so a
    // non-contiguous ROI would be interpreted as a scrambled image.
    const cv::Mat contiguous_image =
        image.isContinuous() ? image : image.clone();
    std::vector<cv::Mat> batch{contiguous_image};
    cuda_batch_preprocess(batch, backend_->inputBuffer(), kInputW, kInputH,
                          backend_->stream());
    float* output = backend_->infer();

    std::vector<std::vector<::Detection>> batches;
    batch_nms(batches, output, 1, kOutputSize, confidence_threshold_,
              nms_threshold_);

    std::vector<DetectionBox> result;
    result.reserve(batches.front().size());
    cv::Mat image_view = image;
    const cv::Rect image_bounds(0, 0, image.cols, image.rows);
    for (const ::Detection& raw : batches.front()) {
      const cv::Rect clipped = get_rect(image_view, const_cast<float*>(raw.bbox)) &
                               image_bounds;
      if (clipped.width <= 0 || clipped.height <= 0) {
        continue;
      }
      DetectionBox box;
      box.rect = cv::Rect2f(clipped);
      box.confidence = raw.conf;
      box.class_id = static_cast<int>(raw.class_id);
      result.push_back(box);
    }
    return result;
  }

  const char* platformName() const { return backend_->platformName(); }

 private:
  float confidence_threshold_;
  float nms_threshold_;
  std::unique_ptr<DetectorBackend> backend_;
  bool owns_preprocess_{false};
  std::mutex inference_mutex_;
};

YoloDetector::YoloDetector(const std::string& engine_file,
                           float confidence_threshold, float nms_threshold)
    : impl_(new Impl(engine_file, confidence_threshold, nms_threshold)) {}

YoloDetector::~YoloDetector() = default;

std::vector<DetectionBox> YoloDetector::detect(const cv::Mat& image) {
  return impl_->detect(image);
}

const char* YoloDetector::platformName() const { return impl_->platformName(); }

}  // namespace yolo
