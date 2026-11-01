#include "detector_backend.h"

#include "config.h"
#include "cuda_utils.h"
#include "logging.h"
#include "types.h"

#include <NvInfer.h>

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr int kOutputSize =
    kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;

class Rtx5080Backend final : public DetectorBackend {
 public:
  ~Rtx5080Backend() override {
    delete[] host_output_;
    if (output_buffer_) cudaFree(output_buffer_);
    if (input_buffer_) cudaFree(input_buffer_);
    if (stream_) cudaStreamDestroy(stream_);
    delete context_;
    delete engine_;
    delete runtime_;
  }

  void initialize(const std::string& engine_file) override {
    std::ifstream file(engine_file, std::ios::binary | std::ios::ate);
    if (!file) {
      throw std::runtime_error("failed to read TensorRT engine: " + engine_file);
    }
    const std::streamsize size = file.tellg();
    if (size <= 0) {
      throw std::runtime_error("TensorRT engine is empty: " + engine_file);
    }
    file.seekg(0, std::ios::beg);
    std::vector<char> serialized(static_cast<size_t>(size));
    if (!file.read(serialized.data(), size)) {
      throw std::runtime_error("failed to load TensorRT engine: " + engine_file);
    }

    runtime_ = nvinfer1::createInferRuntime(logger_);
    if (!runtime_) throw std::runtime_error("failed to create TensorRT runtime");
    engine_ = runtime_->deserializeCudaEngine(serialized.data(), serialized.size());
    if (!engine_) {
      throw std::runtime_error(
          "failed to deserialize TensorRT engine; rebuild it on the RTX 5080 "
          "with TensorRT 10.8: " + engine_file);
    }
    context_ = engine_->createExecutionContext();
    if (!context_) throw std::runtime_error("failed to create execution context");

    validateTensor(kInputTensorName, nvinfer1::TensorIOMode::kINPUT);
    validateTensor(kOutputTensorName, nvinfer1::TensorIOMode::kOUTPUT);

    CUDA_CHECK(cudaStreamCreate(&stream_));
    CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&input_buffer_),
                          kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&output_buffer_),
                          kBatchSize * kOutputSize * sizeof(float)));
    host_output_ = new float[kBatchSize * kOutputSize];

    if (!context_->setTensorAddress(kInputTensorName, input_buffer_) ||
        !context_->setTensorAddress(kOutputTensorName, output_buffer_)) {
      throw std::runtime_error("failed to bind TensorRT input/output tensors");
    }
  }

  float* inputBuffer() override { return input_buffer_; }
  cudaStream_t stream() const override { return stream_; }

  float* infer() override {
    if (!context_->enqueueV3(stream_)) {
      throw std::runtime_error("TensorRT enqueueV3 failed");
    }
    CUDA_CHECK(cudaMemcpyAsync(host_output_, output_buffer_,
                               kBatchSize * kOutputSize * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaStreamSynchronize(stream_));
    return host_output_;
  }

  const char* platformName() const override { return "RTX 5080 / TensorRT 10"; }

 private:
  void validateTensor(const char* name, nvinfer1::TensorIOMode expected_mode) {
    bool found = false;
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char* tensor_name = engine_->getIOTensorName(i);
      if (tensor_name && std::string(tensor_name) == name) {
        found = engine_->getTensorIOMode(tensor_name) == expected_mode;
        break;
      }
    }
    if (!found) {
      throw std::runtime_error("TensorRT engine has no expected tensor: " +
                               std::string(name));
    }
  }

  Logger logger_;
  nvinfer1::IRuntime* runtime_{nullptr};
  nvinfer1::ICudaEngine* engine_{nullptr};
  nvinfer1::IExecutionContext* context_{nullptr};
  cudaStream_t stream_{nullptr};
  float* input_buffer_{nullptr};
  float* output_buffer_{nullptr};
  float* host_output_{nullptr};
};

}  // namespace

std::unique_ptr<DetectorBackend> createDetectorBackend() {
  return std::unique_ptr<DetectorBackend>(new Rtx5080Backend());
}
