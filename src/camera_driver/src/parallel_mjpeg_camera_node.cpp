#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <turbojpeg.h>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace camera_driver {
namespace {

int xioctl(int fd, unsigned long request, void* argument) {
  int result;
  do {
    result = ioctl(fd, request, argument);
  } while (result == -1 && errno == EINTR);
  return result;
}

std::runtime_error systemError(const std::string& operation) {
  return std::runtime_error(operation + ": " + std::strerror(errno));
}

std::string fourccToString(uint32_t value) {
  std::string result(4, ' ');
  result[0] = static_cast<char>(value & 0xff);
  result[1] = static_cast<char>((value >> 8) & 0xff);
  result[2] = static_cast<char>((value >> 16) & 0xff);
  result[3] = static_cast<char>((value >> 24) & 0xff);
  return result;
}

void rotateRgb180(std::vector<unsigned char>* data) {
  if (data == nullptr || data->size() < 6) {
    return;
  }

  unsigned char* left = data->data();
  unsigned char* right = data->data() + data->size() - 3;
  while (left < right) {
    std::swap(left[0], right[0]);
    std::swap(left[1], right[1]);
    std::swap(left[2], right[2]);
    left += 3;
    right -= 3;
  }
}

void rotateRgb90Or270(std::vector<unsigned char>* data, int width, int height,
                      int rotation_angle) {
  std::vector<unsigned char> rotated(data->size());
  const int output_width = height;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const size_t source =
          (static_cast<size_t>(y) * width + x) * 3;
      const int output_x =
          rotation_angle == 90 ? height - 1 - y : y;
      const int output_y =
          rotation_angle == 90 ? x : width - 1 - x;
      const size_t destination =
          (static_cast<size_t>(output_y) * output_width + output_x) * 3;
      rotated[destination] = (*data)[source];
      rotated[destination + 1] = (*data)[source + 1];
      rotated[destination + 2] = (*data)[source + 2];
    }
  }

  data->swap(rotated);
}

bool hasJpegBoundaries(const std::vector<unsigned char>& data) {
  return data.size() >= 4 && data[0] == 0xff && data[1] == 0xd8 &&
         data[data.size() - 2] == 0xff && data[data.size() - 1] == 0xd9;
}

}  // namespace

class ParallelMjpegCamera {
 public:
  ParallelMjpegCamera()
      : nh_(), pnh_("~"), image_transport_(nh_) {
    loadParameters();
    try {
      initializeDevice();
      initializePublisher();
      startThreads();
    } catch (...) {
      cleanupDevice();
      throw;
    }
  }

  ~ParallelMjpegCamera() { stop(); }

  ParallelMjpegCamera(const ParallelMjpegCamera&) = delete;
  ParallelMjpegCamera& operator=(const ParallelMjpegCamera&) = delete;

  void stop() {
    if (!started_.exchange(false)) {
      cleanupDevice();
      return;
    }

    running_.store(false);
    compressed_cv_.notify_all();
    decoded_cv_.notify_all();

    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    for (auto& worker : decoder_threads_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    if (publisher_thread_.joinable()) {
      publisher_thread_.join();
    }

    cleanupDevice();
    logStatistics("final");
  }

 private:
  struct MmapBuffer {
    void* address = MAP_FAILED;
    size_t length = 0;
  };

  struct CompressedFrame {
    uint64_t sequence = 0;
    ros::Time stamp;
    std::vector<unsigned char> jpeg;
  };

  struct DecodedFrame {
    uint64_t sequence = 0;
    sensor_msgs::ImagePtr image;
  };

  void loadParameters() {
    pnh_.param<std::string>("video_device", video_device_, "/dev/video0");
    pnh_.param("image_width", requested_width_, 2592);
    pnh_.param("image_height", requested_height_, 1944);
    pnh_.param("framerate", requested_fps_, 30);
    pnh_.param<std::string>("pixel_format", pixel_format_, "mjpeg");
    pnh_.param<std::string>("io_method", io_method_, "mmap");
    pnh_.param<std::string>("camera_frame_id", frame_id_, "camera");
    pnh_.param<std::string>("camera_name", camera_name_, "camera");
    pnh_.param<std::string>("camera_info_url", camera_info_url_, "");
    pnh_.param<std::string>("output_topic", output_topic_, "image_raw");
    pnh_.param<std::string>("compressed_output_topic",
                            compressed_output_topic_, "image_raw/mjpeg");
    pnh_.param("worker_threads", worker_count_, 3);
    pnh_.param("capture_buffers", capture_buffer_count_, 6);
    pnh_.param("capture_queue_size", capture_queue_size_, 6);
    pnh_.param("decoded_queue_size", decoded_queue_size_, 3);
    pnh_.param("fast_upsample", fast_upsample_, true);
    pnh_.param("rotation_angle", rotation_angle_, 0);
    pnh_.param("statistics_period", statistics_period_, 5.0);
    pnh_.param("invalid_frame_restart_threshold",
                invalid_frame_restart_threshold_, 5);

    worker_count_ = std::max(1, std::min(worker_count_, 16));
    capture_buffer_count_ = std::max(2, capture_buffer_count_);
    capture_queue_size_ = std::max(1, capture_queue_size_);
    decoded_queue_size_ = std::max(1, decoded_queue_size_);
    statistics_period_ = std::max(1.0, statistics_period_);
    invalid_frame_restart_threshold_ =
        std::max(1, invalid_frame_restart_threshold_);
    rotation_angle_ = ((rotation_angle_ % 360) + 360) % 360;
    if (rotation_angle_ != 0 && rotation_angle_ != 90 &&
        rotation_angle_ != 180 && rotation_angle_ != 270) {
      throw std::runtime_error(
          "rotation_angle must be one of 0, 90, 180 or 270 degrees");
    }

    std::string normalized_format = pixel_format_;
    std::transform(
        normalized_format.begin(), normalized_format.end(),
        normalized_format.begin(),
        [](unsigned char character) { return std::tolower(character); });
    if (normalized_format != "mjpeg" && normalized_format != "mjpg") {
      throw std::runtime_error(
          "parallel_mjpeg_camera_node supports only MJPEG input");
    }
    if (io_method_ != "mmap") {
      throw std::runtime_error(
          "parallel_mjpeg_camera_node supports only mmap I/O");
    }
  }

  void initializeDevice() {
    fd_ = open(video_device_.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd_ == -1) {
      throw systemError("open " + video_device_);
    }

    v4l2_capability capability{};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &capability) == -1) {
      throw systemError("VIDIOC_QUERYCAP");
    }
    if (!(capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
        !(capability.capabilities & V4L2_CAP_STREAMING)) {
      throw std::runtime_error(
          video_device_ + " is not a streaming V4L2 capture device");
    }

    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = requested_width_;
    format.fmt.pix.height = requested_height_;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    format.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(fd_, VIDIOC_S_FMT, &format) == -1) {
      throw systemError("VIDIOC_S_FMT");
    }

    width_ = static_cast<int>(format.fmt.pix.width);
    height_ = static_cast<int>(format.fmt.pix.height);
    if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
      throw std::runtime_error(
          "camera negotiated " + fourccToString(format.fmt.pix.pixelformat) +
          " instead of MJPG");
    }

    v4l2_streamparm stream_parameters{};
    stream_parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stream_parameters.parm.capture.timeperframe.numerator = 1;
    stream_parameters.parm.capture.timeperframe.denominator = requested_fps_;
    if (xioctl(fd_, VIDIOC_S_PARM, &stream_parameters) == -1) {
      throw systemError("VIDIOC_S_PARM");
    }
    const auto& time_per_frame =
        stream_parameters.parm.capture.timeperframe;
    actual_fps_ = time_per_frame.numerator == 0
                      ? 0.0
                      : static_cast<double>(time_per_frame.denominator) /
                            time_per_frame.numerator;

    v4l2_requestbuffers request{};
    request.count = static_cast<uint32_t>(capture_buffer_count_);
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &request) == -1) {
      throw systemError("VIDIOC_REQBUFS");
    }
    if (request.count < 2) {
      throw std::runtime_error("camera returned fewer than two mmap buffers");
    }

    mmap_buffers_.resize(request.count);
    for (uint32_t index = 0; index < request.count; ++index) {
      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      buffer.index = index;
      if (xioctl(fd_, VIDIOC_QUERYBUF, &buffer) == -1) {
        throw systemError("VIDIOC_QUERYBUF");
      }

      mmap_buffers_[index].length = buffer.length;
      mmap_buffers_[index].address =
          mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_,
               buffer.m.offset);
      if (mmap_buffers_[index].address == MAP_FAILED) {
        throw systemError("mmap");
      }
      if (xioctl(fd_, VIDIOC_QBUF, &buffer) == -1) {
        throw systemError("VIDIOC_QBUF");
      }
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
      throw systemError("VIDIOC_STREAMON");
    }
    streaming_ = true;

    ROS_INFO(
        "parallel MJPEG camera: %s %dx%d@%.3f FPS, buffers=%zu, workers=%d",
        video_device_.c_str(), width_, height_, actual_fps_,
        mmap_buffers_.size(), worker_count_);
    if (width_ != requested_width_ || height_ != requested_height_) {
      ROS_WARN("requested %dx%d but camera negotiated %dx%d", requested_width_,
               requested_height_, width_, height_);
    }
  }

  void initializePublisher() {
    camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(
        nh_, camera_name_, camera_info_url_));
    camera_publisher_ = image_transport_.advertiseCamera(output_topic_, 1);
    compressed_publisher_ =
        nh_.advertise<sensor_msgs::CompressedImage>(compressed_output_topic_, 1);
  }

  void startThreads() {
    running_.store(true);
    started_.store(true);
    statistics_start_ = ros::WallTime::now();
    last_statistics_time_ = statistics_start_;

    capture_thread_ = std::thread(&ParallelMjpegCamera::captureLoop, this);
    decoder_threads_.reserve(worker_count_);
    for (int index = 0; index < worker_count_; ++index) {
      decoder_threads_.emplace_back(&ParallelMjpegCamera::decodeLoop, this,
                                    index);
    }
    publisher_thread_ = std::thread(&ParallelMjpegCamera::publisherLoop, this);
  }

  void captureLoop() {
    pollfd descriptor{};
    descriptor.fd = fd_;
    descriptor.events = POLLIN;

    while (running_.load() && ros::ok()) {
      if (stream_restart_requested_.exchange(false)) {
        std::string error;
        if (!restartCaptureStream(&error)) {
          failFromThread(error);
          break;
        }
        consecutive_invalid_captures_.store(0);
        consecutive_decode_failures_.store(0);
        continue;
      }

      const int poll_result = poll(&descriptor, 1, 1000);
      if (poll_result == -1) {
        if (errno == EINTR) {
          continue;
        }
        failFromThread(systemError("poll").what());
        break;
      }
      if (poll_result == 0) {
        ROS_WARN_THROTTLE(5.0, "timeout waiting for a camera frame");
        continue;
      }
      if (!(descriptor.revents & POLLIN)) {
        continue;
      }

      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      if (xioctl(fd_, VIDIOC_DQBUF, &buffer) == -1) {
        if (errno == EAGAIN) {
          continue;
        }
        failFromThread(systemError("VIDIOC_DQBUF").what());
        break;
      }

      CompressedFrame frame;
      frame.sequence = capture_sequence_++;
      frame.stamp = ros::Time::now();
      if (buffer.index < mmap_buffers_.size() && buffer.bytesused > 0 &&
          buffer.bytesused <= mmap_buffers_[buffer.index].length) {
        const auto* begin = static_cast<const unsigned char*>(
            mmap_buffers_[buffer.index].address);
        frame.jpeg.assign(begin, begin + buffer.bytesused);
      }

      if (xioctl(fd_, VIDIOC_QBUF, &buffer) == -1) {
        failFromThread(systemError("VIDIOC_QBUF").what());
        break;
      }

      if (frame.jpeg.empty()) {
        invalid_frames_.fetch_add(1);
        noteInvalidCapture("empty V4L2 buffer", 0, 0);
        continue;
      }

      if (!hasJpegBoundaries(frame.jpeg)) {
        invalid_frames_.fetch_add(1);
        const unsigned int first = frame.jpeg.size() > 0 ? frame.jpeg[0] : 0;
        const unsigned int second = frame.jpeg.size() > 1 ? frame.jpeg[1] : 0;
        noteInvalidCapture("invalid JPEG boundaries", first, second);
        continue;
      }

      consecutive_invalid_captures_.store(0);
      captured_frames_.fetch_add(1);
      sensor_msgs::CompressedImage compressed_message;
      compressed_message.header.seq = static_cast<uint32_t>(frame.sequence);
      compressed_message.header.stamp = frame.stamp;
      compressed_message.header.frame_id = frame_id_;
      compressed_message.format = "jpeg";
      compressed_message.data = frame.jpeg;
      compressed_publisher_.publish(compressed_message);
      {
        std::lock_guard<std::mutex> lock(compressed_mutex_);
        while (compressed_queue_.size() >=
               static_cast<size_t>(capture_queue_size_)) {
          compressed_queue_.pop_front();
          dropped_before_decode_.fetch_add(1);
        }
        compressed_queue_.emplace_back(std::move(frame));
      }
      compressed_cv_.notify_one();
    }

    running_.store(false);
    compressed_cv_.notify_all();
    decoded_cv_.notify_all();
  }

  void decodeLoop(int worker_index) {
    tjhandle decoder = tjInitDecompress();
    if (decoder == nullptr) {
      failFromThread("tjInitDecompress failed");
      return;
    }

    while (running_.load() || hasCompressedFrames()) {
      CompressedFrame frame;
      {
        std::unique_lock<std::mutex> lock(compressed_mutex_);
        compressed_cv_.wait(lock, [this] {
          return !running_.load() || !compressed_queue_.empty();
        });
        if (compressed_queue_.empty()) {
          continue;
        }
        frame = std::move(compressed_queue_.front());
        compressed_queue_.pop_front();
      }

      int jpeg_width = 0;
      int jpeg_height = 0;
      int subsampling = 0;
      int colorspace = 0;
      if (tjDecompressHeader3(decoder, frame.jpeg.data(), frame.jpeg.size(),
                              &jpeg_width, &jpeg_height, &subsampling,
                              &colorspace) != 0) {
        invalid_frames_.fetch_add(1);
        ROS_WARN_THROTTLE(2.0, "TurboJPEG header error: %s",
                          tjGetErrorStr2(decoder));
        noteDecodeFailure();
        continue;
      }
      (void)subsampling;
      (void)colorspace;

      sensor_msgs::ImagePtr image(new sensor_msgs::Image());
      image->header.seq = static_cast<uint32_t>(frame.sequence);
      image->header.stamp = frame.stamp;
      image->header.frame_id = frame_id_;
      image->height = static_cast<uint32_t>(jpeg_height);
      image->width = static_cast<uint32_t>(jpeg_width);
      image->encoding = "rgb8";
      image->is_bigendian = false;
      image->step = static_cast<uint32_t>(jpeg_width * 3);
      image->data.resize(static_cast<size_t>(jpeg_width) * jpeg_height * 3);

      int flags = TJFLAG_FASTDCT;
      if (fast_upsample_) {
        flags |= TJFLAG_FASTUPSAMPLE;
      }
      if (tjDecompress2(decoder, frame.jpeg.data(), frame.jpeg.size(),
                        image->data.data(), jpeg_width, 0, jpeg_height, TJPF_RGB,
                        flags) != 0) {
        invalid_frames_.fetch_add(1);
        ROS_WARN_THROTTLE(2.0, "TurboJPEG worker %d decode error: %s",
                          worker_index, tjGetErrorStr2(decoder));
        noteDecodeFailure();
        continue;
      }

      consecutive_decode_failures_.store(0);
      if (rotation_angle_ == 180) {
        rotateRgb180(&image->data);
      } else if (rotation_angle_ == 90 || rotation_angle_ == 270) {
        rotateRgb90Or270(&image->data, jpeg_width, jpeg_height,
                         rotation_angle_);
        image->width = static_cast<uint32_t>(jpeg_height);
        image->height = static_cast<uint32_t>(jpeg_width);
        image->step = static_cast<uint32_t>(jpeg_height * 3);
      }

      decoded_frames_.fetch_add(1);
      DecodedFrame decoded;
      decoded.sequence = frame.sequence;
      decoded.image = std::move(image);
      {
        std::lock_guard<std::mutex> lock(decoded_mutex_);
        while (decoded_queue_.size() >=
               static_cast<size_t>(decoded_queue_size_)) {
          decoded_queue_.pop_front();
          dropped_before_publish_.fetch_add(1);
        }
        decoded_queue_.emplace_back(std::move(decoded));
      }
      decoded_cv_.notify_one();
    }

    tjDestroy(decoder);
  }

  void publisherLoop() {
    uint64_t last_published_sequence = 0;
    bool has_published = false;

    while (running_.load() || hasDecodedFrames()) {
      DecodedFrame decoded;
      {
        std::unique_lock<std::mutex> lock(decoded_mutex_);
        decoded_cv_.wait(lock, [this] {
          return !running_.load() || !decoded_queue_.empty();
        });
        if (decoded_queue_.empty()) {
          continue;
        }

        auto newest = std::max_element(
            decoded_queue_.begin(), decoded_queue_.end(),
            [](const DecodedFrame& left, const DecodedFrame& right) {
              return left.sequence < right.sequence;
            });
        decoded = std::move(*newest);
        if (decoded_queue_.size() > 1) {
          dropped_before_publish_.fetch_add(decoded_queue_.size() - 1);
        }
        decoded_queue_.clear();
      }

      if (has_published && decoded.sequence <= last_published_sequence) {
        dropped_before_publish_.fetch_add(1);
        continue;
      }

      sensor_msgs::CameraInfo camera_info =
          camera_info_manager_->getCameraInfo();
      camera_info.header = decoded.image->header;
      camera_info.width = decoded.image->width;
      camera_info.height = decoded.image->height;
      camera_publisher_.publish(*decoded.image, camera_info);

      last_published_sequence = decoded.sequence;
      has_published = true;
      published_frames_.fetch_add(1);

      const ros::WallTime now = ros::WallTime::now();
      if ((now - last_statistics_time_).toSec() >= statistics_period_) {
        logStatistics("running");
        last_statistics_time_ = now;
      }
    }
  }

  bool hasCompressedFrames() {
    std::lock_guard<std::mutex> lock(compressed_mutex_);
    return !compressed_queue_.empty();
  }

  bool hasDecodedFrames() {
    std::lock_guard<std::mutex> lock(decoded_mutex_);
    return !decoded_queue_.empty();
  }

  void noteInvalidCapture(const char* reason, unsigned int first,
                          unsigned int second) {
    const int count = consecutive_invalid_captures_.fetch_add(1) + 1;
    ROS_WARN_THROTTLE(
        2.0,
        "discarding malformed MJPEG frame (%s, first bytes 0x%02x 0x%02x, "
        "consecutive=%d/%d)",
        reason, first, second, count, invalid_frame_restart_threshold_);
    if (count >= invalid_frame_restart_threshold_) {
      stream_restart_requested_.store(true);
    }
  }

  void noteDecodeFailure() {
    const int count = consecutive_decode_failures_.fetch_add(1) + 1;
    if (count >= invalid_frame_restart_threshold_) {
      stream_restart_requested_.store(true);
    }
  }

  bool restartCaptureStream(std::string* error) {
    ROS_WARN("restarting V4L2 stream after consecutive malformed MJPEG frames");

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMOFF, &type) == -1) {
      *error = systemError("VIDIOC_STREAMOFF during recovery").what();
      return false;
    }
    streaming_ = false;

    {
      std::lock_guard<std::mutex> lock(compressed_mutex_);
      compressed_queue_.clear();
    }
    {
      std::lock_guard<std::mutex> lock(decoded_mutex_);
      decoded_queue_.clear();
    }

    for (uint32_t index = 0; index < mmap_buffers_.size(); ++index) {
      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      buffer.index = index;
      if (xioctl(fd_, VIDIOC_QBUF, &buffer) == -1) {
        *error = systemError("VIDIOC_QBUF during recovery").what();
        return false;
      }
    }

    if (xioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
      *error = systemError("VIDIOC_STREAMON during recovery").what();
      return false;
    }
    streaming_ = true;
    stream_restarts_.fetch_add(1);
    ROS_INFO("V4L2 stream recovery completed");
    return true;
  }

  void failFromThread(const std::string& message) {
    ROS_ERROR("parallel MJPEG camera stopped: %s", message.c_str());
    running_.store(false);
    compressed_cv_.notify_all();
    decoded_cv_.notify_all();
    ros::requestShutdown();
  }

  void logStatistics(const char* state) const {
    const double elapsed =
        std::max(1e-6, (ros::WallTime::now() - statistics_start_).toSec());
    ROS_INFO(
        "camera stats (%s): capture=%.2f Hz decode=%.2f Hz publish=%.2f Hz "
        "drop_capture=%llu drop_publish=%llu invalid=%llu restarts=%llu",
        state, captured_frames_.load() / elapsed, decoded_frames_.load() / elapsed,
        published_frames_.load() / elapsed,
        static_cast<unsigned long long>(dropped_before_decode_.load()),
        static_cast<unsigned long long>(dropped_before_publish_.load()),
        static_cast<unsigned long long>(invalid_frames_.load()),
        static_cast<unsigned long long>(stream_restarts_.load()));
  }

  void cleanupDevice() {
    if (streaming_ && fd_ != -1) {
      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (xioctl(fd_, VIDIOC_STREAMOFF, &type) == -1 && errno != ENODEV) {
        ROS_WARN("VIDIOC_STREAMOFF failed: %s", std::strerror(errno));
      }
      streaming_ = false;
    }

    for (auto& buffer : mmap_buffers_) {
      if (buffer.address != MAP_FAILED) {
        munmap(buffer.address, buffer.length);
        buffer.address = MAP_FAILED;
      }
    }
    mmap_buffers_.clear();

    if (fd_ != -1) {
      close(fd_);
      fd_ = -1;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport image_transport_;
  image_transport::CameraPublisher camera_publisher_;
  ros::Publisher compressed_publisher_;
  std::unique_ptr<camera_info_manager::CameraInfoManager>
      camera_info_manager_;

  std::string video_device_;
  std::string pixel_format_;
  std::string io_method_;
  std::string frame_id_;
  std::string camera_name_;
  std::string camera_info_url_;
  std::string output_topic_;
  std::string compressed_output_topic_;
  int requested_width_ = 2592;
  int requested_height_ = 1944;
  int requested_fps_ = 30;
  int width_ = 0;
  int height_ = 0;
  double actual_fps_ = 0.0;
  int worker_count_ = 3;
  int capture_buffer_count_ = 6;
  int capture_queue_size_ = 6;
  int decoded_queue_size_ = 3;
  bool fast_upsample_ = true;
  int rotation_angle_ = 0;
  double statistics_period_ = 5.0;
  int invalid_frame_restart_threshold_ = 5;

  int fd_ = -1;
  bool streaming_ = false;
  std::vector<MmapBuffer> mmap_buffers_;

  std::atomic<bool> running_{false};
  std::atomic<bool> started_{false};
  std::thread capture_thread_;
  std::vector<std::thread> decoder_threads_;
  std::thread publisher_thread_;

  std::mutex compressed_mutex_;
  std::condition_variable compressed_cv_;
  std::deque<CompressedFrame> compressed_queue_;
  std::mutex decoded_mutex_;
  std::condition_variable decoded_cv_;
  std::deque<DecodedFrame> decoded_queue_;

  uint64_t capture_sequence_ = 0;
  std::atomic<uint64_t> captured_frames_{0};
  std::atomic<uint64_t> decoded_frames_{0};
  std::atomic<uint64_t> published_frames_{0};
  std::atomic<uint64_t> dropped_before_decode_{0};
  std::atomic<uint64_t> dropped_before_publish_{0};
  std::atomic<uint64_t> invalid_frames_{0};
  std::atomic<uint64_t> stream_restarts_{0};
  std::atomic<int> consecutive_invalid_captures_{0};
  std::atomic<int> consecutive_decode_failures_{0};
  std::atomic<bool> stream_restart_requested_{false};
  ros::WallTime statistics_start_;
  ros::WallTime last_statistics_time_;
};

}  // namespace camera_driver

int main(int argc, char** argv) {
  ros::init(argc, argv, "parallel_mjpeg_camera");
  try {
    camera_driver::ParallelMjpegCamera camera;
    ros::waitForShutdown();
    camera.stop();
  } catch (const std::exception& error) {
    ROS_FATAL("failed to start parallel MJPEG camera: %s", error.what());
    return 1;
  }
  return 0;
}
