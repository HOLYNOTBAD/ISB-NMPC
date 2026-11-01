#pragma once

#include <cuda_runtime.h>

#include <memory>
#include <string>

class DetectorBackend {
 public:
  virtual ~DetectorBackend() = default;

  virtual void initialize(const std::string& engine_file) = 0;
  virtual float* inputBuffer() = 0;
  virtual cudaStream_t stream() const = 0;
  virtual float* infer() = 0;
  virtual const char* platformName() const = 0;
};

std::unique_ptr<DetectorBackend> createDetectorBackend();
