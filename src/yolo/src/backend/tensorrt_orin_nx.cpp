#include "detector_backend.h"

#include "config.h"
#include "cuda_utils.h"
#include "logging.h"
#include "types.h"

#include <NvInfer.h>

#include <cassert>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace {

constexpr int kOutputSize =
    kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;

class OrinNxBackend final : public DetectorBackend {
 public:
  ~OrinNxBackend() override {
    delete[] host_output_;
    if (buffers_[1]) cudaFree(buffers_[1]);
    if (buffers_[0]) cudaFree(buffers_[0]);
    if (stream_) cudaStreamDestroy(stream_);
    if (context_) context_->destroy();
    if (engine_) engine_->destroy();
    if (runtime_) runtime_->destroy();
  }

  void initialize(const std::string& engine_file) override {
    std::ifstream file(engine_file, std::ios::binary | std::ios::ate);
    if (!file) throw std::runtime_error("failed to read TensorRT engine: " + engine_file);
    const std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> serialized(static_cast<size_t>(size));
    if (!file.read(serialized.data(), size)) {
      throw std::runtime_error("failed to load TensorRT engine: " + engine_file);
    }

    runtime_ = nvinfer1::createInferRuntime(logger_);
    engine_ = runtime_ ? runtime_->deserializeCudaEngine(serialized.data(), serialized.size()) : nullptr;
    context_ = engine_ ? engine_->createExecutionContext() : nullptr;
    if (!context_) {
      throw std::runtime_error("failed to initialize Orin TensorRT engine: " + engine_file);
    }

    assert(engine_->getNbBindings() == 2);
    assert(engine_->getBindingIndex(kInputTensorName) == 0);
    assert(engine_->getBindingIndex(kOutputTensorName) == 1);
    CUDA_CHECK(cudaStreamCreate(&stream_));
    CUDA_CHECK(cudaMalloc(&buffers_[0],
                          kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&buffers_[1],
                          kBatchSize * kOutputSize * sizeof(float)));
    host_output_ = new float[kBatchSize * kOutputSize];
  }

  float* inputBuffer() override { return static_cast<float*>(buffers_[0]); }
  cudaStream_t stream() const override { return stream_; }

  float* infer() override {
    if (!context_->enqueue(kBatchSize, buffers_, stream_, nullptr)) {
      throw std::runtime_error("Orin TensorRT enqueue failed");
    }
    CUDA_CHECK(cudaMemcpyAsync(host_output_, buffers_[1],
                               kBatchSize * kOutputSize * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaStreamSynchronize(stream_));
    return host_output_;
  }

  const char* platformName() const override { return "Jetson Orin NX"; }

 private:
  Logger logger_;
  nvinfer1::IRuntime* runtime_{nullptr};
  nvinfer1::ICudaEngine* engine_{nullptr};
  nvinfer1::IExecutionContext* context_{nullptr};
  cudaStream_t stream_{nullptr};
  void* buffers_[2]{nullptr, nullptr};
  float* host_output_{nullptr};
};

}  // namespace

std::unique_ptr<DetectorBackend> createDetectorBackend() {
  return std::unique_ptr<DetectorBackend>(new OrinNxBackend());
}
