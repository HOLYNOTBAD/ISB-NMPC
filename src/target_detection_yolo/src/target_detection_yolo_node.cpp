#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <swarm_msgs/BoundingBox.h>
#include <swarm_msgs/BoundingBoxSequence.h>
#include <swarm_msgs/BoundingBoxes.h>
#include <yolo/yolo_detector.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "target_detection_yolo/detection_fsm.h"
#include "target_detection_yolo/detection_strategy.h"
#include "target_detection_yolo/reacquire_hr.h"
#include "target_detection_yolo/track_hr.h"
#include "target_detection_yolo/track_lr.h"

namespace target_detection_yolo {
namespace {

// ============================================================================
// 1. YOLO Detector Adapter（YOLO 检测器适配器）
// ============================================================================
// 将 yolo 包提供的检测器适配为本模块统一的 Detector 接口。三个检测策略
// 共享同一个适配器实例，从而只加载一份 TensorRT engine 和 CUDA 运行资源。
class YoloDetectorAdapter final : public Detector {
 public:
  // 创建底层 YOLO 检测器，并设置推理阶段使用的置信度及 NMS 阈值。
  YoloDetectorAdapter(const std::string& engine_file, float confidence_threshold,
                      float nms_threshold)
      : detector_(engine_file, confidence_threshold, nms_threshold) {}

  // 执行推理，并将 yolo::DetectionBox 转换为策略层使用的 DetectionBox。
  // 底层检测器返回的坐标已经对应传入图像，而不是网络 letterbox 坐标。
  std::vector<DetectionBox> detect(const cv::Mat& image) override {
    const std::vector<yolo::DetectionBox> yolo_boxes = detector_.detect(image);
    std::vector<DetectionBox> boxes;
    boxes.reserve(yolo_boxes.size());
    for (const yolo::DetectionBox& source : yolo_boxes) {
      DetectionBox destination;
      destination.rect = source.rect;
      destination.confidence = source.confidence;
      destination.class_id = source.class_id;
      boxes.push_back(destination);
    }
    return boxes;
  }

  const char* platformName() const { return detector_.platformName(); }

 private:
  yolo::YoloDetector detector_;
};

}  // namespace

// ============================================================================
// 2. Target Detection Node（目标检测节点）
// ============================================================================
// 节点负责连接 ROS 图像流、三种检测策略和尺度切换状态机：
//   REACQUIRE_HR：逐帧扫描原图图块，重新捕获目标；
//   TRACK_HR：围绕上一帧目标裁剪原生分辨率 ROI；
//   TRACK_LR：在完整图像上检测，由 YOLO 后端完成网络输入缩放。
class TargetDetectionNode {
 public:
  // ========================================================================
  // 2.1 Constructor（构造函数）
  // ========================================================================
  TargetDetectionNode()
      : nh_(), private_nh_("~"), image_transport_(nh_) {
    // 节点私有参数用于配置模型文件和 ROS 话题；检测器公共阈值由
    // target_detection_yolo/config/detection_fsm.yaml 加载到全局 NodeHandle 命名空间。
    std::string engine_file;
    std::string input_image_topic = "/camera/image_raw";
    std::string output_bbox_topic = "/tracker/bbox_raw";
    std::string mode_topic = "/target_detection/mode";
    std::string roi_topic = "/target_detection/roi";
    std::string debug_image_topic = "/target_detection/fpv_source";
    std::string ekf_prediction_topic = "/tracker/bbox";
    double detector_confidence = 0.30;
    double detector_nms = 0.30;
    private_nh_.param<std::string>("engine_file", engine_file, "");
    private_nh_.param<std::string>("input_image_topic", input_image_topic,
                                   input_image_topic);
    private_nh_.param<std::string>("output_bbox_topic", output_bbox_topic,
                                   output_bbox_topic);
    private_nh_.param<std::string>("mode_topic", mode_topic, mode_topic);
    private_nh_.param<std::string>("roi_topic", roi_topic, roi_topic);
    private_nh_.param<std::string>("debug_image_topic", debug_image_topic,
                                   debug_image_topic);
    private_nh_.param("debug_image_rate_hz", debug_image_rate_hz_,
                      debug_image_rate_hz_);
    if (!std::isfinite(debug_image_rate_hz_) || debug_image_rate_hz_ < 0.0) {
      throw std::invalid_argument("invalid debug image rate");
    }
    nh_.param("detector_confidence_threshold", detector_confidence,
              detector_confidence);
    nh_.param("detector_nms_threshold", detector_nms, detector_nms);

    // scale_switch 参数加载到状态机配置中。配置文件由 launch 加载到全局
    // NodeHandle 命名空间，因此这里通过 nh_ 读取。
    DetectionFsmConfig fsm_config;
    nh_.param("scale_switch/downscale_factor", fsm_config.downscale_factor,
              fsm_config.downscale_factor);
    nh_.param("scale_switch/reacquire_min_confidence",
              fsm_config.reacquire_min_confidence,
              fsm_config.reacquire_min_confidence);
    nh_.param("scale_switch/reacquire_consecutive_detections",
              fsm_config.reacquire_consecutive_detections,
              fsm_config.reacquire_consecutive_detections);
    nh_.param("scale_switch/hr_lost_confidence",
              fsm_config.hr_lost_confidence, fsm_config.hr_lost_confidence);
    nh_.param("scale_switch/hr_lost_consecutive_failures",
              fsm_config.hr_lost_consecutive_failures,
              fsm_config.hr_lost_consecutive_failures);
    nh_.param("scale_switch/lr_enter_min_target_size_px",
              fsm_config.lr_enter_min_target_size_px,
              fsm_config.lr_enter_min_target_size_px);
    nh_.param("scale_switch/lr_enter_min_confidence",
              fsm_config.lr_enter_min_confidence,
              fsm_config.lr_enter_min_confidence);
    nh_.param("scale_switch/lr_enter_consecutive_detections",
              fsm_config.lr_enter_consecutive_detections,
              fsm_config.lr_enter_consecutive_detections);
    nh_.param("scale_switch/lr_exit_target_size_px",
              fsm_config.lr_exit_target_size_px,
              fsm_config.lr_exit_target_size_px);
    nh_.param("scale_switch/lr_exit_confidence",
              fsm_config.lr_exit_confidence, fsm_config.lr_exit_confidence);
    nh_.param("scale_switch/lr_exit_consecutive_failures",
              fsm_config.lr_exit_consecutive_failures,
              fsm_config.lr_exit_consecutive_failures);

    std::string reason;
    if (!fsm_.configure(fsm_config, &reason)) {
      throw std::invalid_argument("invalid detection FSM config: " + reason);
    }

    // 加载高分辨率分块重捕获策略参数。
    ReacquireHrConfig reacquire_config;
    nh_.param("strategies/reacquire_hr/tile_width",
              reacquire_config.tile_width, reacquire_config.tile_width);
    nh_.param("strategies/reacquire_hr/tile_height",
              reacquire_config.tile_height, reacquire_config.tile_height);
    nh_.param("strategies/reacquire_hr/overlap_ratio",
              reacquire_config.overlap_ratio, reacquire_config.overlap_ratio);
    nh_.param("strategies/reacquire_hr/min_confidence",
              reacquire_config.min_confidence,
              reacquire_config.min_confidence);
    nh_.param("strategies/reacquire_hr/confirmation_min_confidence",
              reacquire_config.confirmation_min_confidence,
              reacquire_config.confirmation_min_confidence);

    // 加载高分辨率 ROI 跟踪策略参数。
    TrackHrConfig track_hr_config;
    nh_.param("strategies/track_hr/roi_width", track_hr_config.roi_width,
              track_hr_config.roi_width);
    nh_.param("strategies/track_hr/roi_height", track_hr_config.roi_height,
              track_hr_config.roi_height);
    nh_.param("strategies/track_hr/min_confidence",
              track_hr_config.min_confidence, track_hr_config.min_confidence);
    nh_.param("strategies/track_hr/prediction_association_radius_px",
              track_hr_config.prediction_association_radius_px,
              track_hr_config.prediction_association_radius_px);
    nh_.param("strategies/track_hr/use_ekf_prediction",
              use_ekf_prediction_, use_ekf_prediction_);
    nh_.param("strategies/track_hr/ekf_prediction_topic",
              ekf_prediction_topic, ekf_prediction_topic);
    nh_.param("strategies/track_hr/ekf_prediction_timeout_s",
              ekf_prediction_timeout_s_, ekf_prediction_timeout_s_);
    nh_.param("strategies/track_hr/ekf_prediction_history_s",
              ekf_prediction_history_s_, ekf_prediction_history_s_);
    nh_.param("strategies/track_hr/ekf_prediction_max_sequences",
              ekf_prediction_max_sequences_, ekf_prediction_max_sequences_);
    if (!std::isfinite(ekf_prediction_timeout_s_) ||
        ekf_prediction_timeout_s_ <= 0.0 ||
        !std::isfinite(ekf_prediction_history_s_) ||
        ekf_prediction_history_s_ <= 0.0 ||
        ekf_prediction_max_sequences_ <= 0) {
      throw std::invalid_argument("invalid TRACK_HR EKF prediction timing");
    }

    // 加载低分辨率全图跟踪策略参数。
    TrackLrConfig track_lr_config;
    nh_.param("strategies/track_lr/min_confidence",
              track_lr_config.min_confidence, track_lr_config.min_confidence);

    // 三种模式使用相同的跨帧关联门限，保证模式切换前后目标身份判据一致。
    TargetAssociationConfig association_config;
    nh_.param("association/min_iou", association_config.min_iou,
              association_config.min_iou);
    nh_.param("association/max_center_distance_factor",
              association_config.max_center_distance_factor,
              association_config.max_center_distance_factor);
    if (!association_config.isValid()) {
      throw std::invalid_argument("invalid target association config");
    }
    reacquire_config.association = association_config;
    track_hr_config.association = association_config;
    track_lr_config.association = association_config;

    // 初始化 YOLO detector
    detector_.reset(new YoloDetectorAdapter(
        engine_file, static_cast<float>(detector_confidence),
        static_cast<float>(detector_nms)));
    // 初始化三种检测策略
    reacquire_hr_.reset(new ReacquireHrStrategy(detector_.get(), reacquire_config));
    track_hr_.reset(new TrackHrStrategy(detector_.get(), track_hr_config));
    track_lr_.reset(new TrackLrStrategy(detector_.get(), track_lr_config));

    // 发布检测框和当前模式，并订阅相机原始图像。模式话题使用 latch，
    // 使后启动的订阅者也能立即获得当前状态。
    bbox_publisher_ = nh_.advertise<swarm_msgs::BoundingBoxes>(output_bbox_topic, 1);
    mode_publisher_ = nh_.advertise<std_msgs::String>(mode_topic, 1, true);
    roi_publisher_ =
        nh_.advertise<sensor_msgs::RegionOfInterest>(roi_topic, 1);
    debug_image_publisher_ = image_transport_.advertise(debug_image_topic, 1);

    image_subscriber_ = image_transport_.subscribe(
        input_image_topic, 1, &TargetDetectionNode::imageCallback, this);
    if (use_ekf_prediction_) {
      ekf_prediction_subscriber_ = nh_.subscribe(
          ekf_prediction_topic, 1,
          &TargetDetectionNode::ekfPredictionCallback, this);
    }
    reset_trial_service_ = private_nh_.advertiseService(
        "reset_trial", &TargetDetectionNode::resetTrial, this);

    publishMode();

    ROS_INFO(
        "target_detection_yolo: backend=%s image=%s bbox=%s initial_mode=%s "
        "ekf_roi=%s timeout=%.3fs debug_image=%s@%.1fHz",
        detector_->platformName(), input_image_topic.c_str(),
        output_bbox_topic.c_str(), DetectionFsm::toString(fsm_.mode()),
        use_ekf_prediction_ ? ekf_prediction_topic.c_str() : "disabled",
        ekf_prediction_timeout_s_, debug_image_topic.c_str(),
        debug_image_rate_hz_);
  }

 private:
  bool resetTrial(std_srvs::Trigger::Request&,
                  std_srvs::Trigger::Response& response) {
    fsm_.reset(DetectionMode::REACQUIRE_HR);
    context_ = TrackingContext();
    ekf_prediction_history_.clear();
    reacquire_hr_->reset();
    track_hr_->reset();
    track_lr_->reset();
    next_debug_image_time_ = ros::WallTime();

    swarm_msgs::BoundingBoxes empty;
    empty.header.stamp = ros::Time::now();
    empty.image_header.stamp = empty.header.stamp;
    bbox_publisher_.publish(empty);
    publishMode();

    response.success = true;
    response.message = "target detection state reset to REACQUIRE_HR";
    ROS_INFO("target_detection_yolo: trial state reset to REACQUIRE_HR.");
    return true;
  }

  // ========================================================================
  // 2.2 Strategy Dispatch（检测策略分派）
  // ========================================================================
  // 根据 FSM 当前模式返回对应策略。返回指针由本节点拥有，调用方不负责释放。
  DetectionStrategy* activeStrategy() {
    switch (fsm_.mode()) {
      case DetectionMode::REACQUIRE_HR:
        return reacquire_hr_.get();
      case DetectionMode::TRACK_HR:
        return track_hr_.get();
      case DetectionMode::TRACK_LR:
        return track_lr_.get();
    }
    throw std::logic_error("unknown detection mode");
  }

  // ========================================================================
  // 2.3 Image Callback（图像回调）
  // ========================================================================
  // 每收到一帧图像，依次完成：图像转换、当前策略推理、跟踪上下文更新、
  // FSM 状态转移以及检测结果发布。
  void imageCallback(const sensor_msgs::ImageConstPtr& message) {
    try {
      // toCvShare 尽量复用 ROS 消息内存，避免无必要的整帧图像复制。
      const cv_bridge::CvImageConstPtr image =
          cv_bridge::toCvShare(message, "bgr8");
      updateEkfPredictionContext(image->image.size(), message->header.stamp);
      const DetectionResult result =
          activeStrategy()->detect(image->image, context_);
      publishRoi(result.processed_region);

      // FSM 统一接收原图坐标下的目标最短边和置信度。策略内部产生的局部
      // 坐标在返回前已经恢复到原图坐标。
      DetectionObservation observation;
      observation.valid = result.valid;
      // A valid EKF prediction keeps TRACK_HR alive during a short visual
      // dropout. It is deliberately not copied into target/confidence below:
      // prediction is a coast state, never a visual measurement.
      observation.predicted = !result.valid &&
                              context_.has_predicted_target_center;
      if (result.valid) {
        observation.target_min_size_full_px =
            std::min(result.target.rect.width, result.target.rect.height);
        observation.confidence = result.target.confidence;
        context_.has_previous_target = true;
        context_.previous_target = result.target;
      }

      // 状态发生变化后重置新策略的私有状态，例如重捕获策略的图块游标。
      const DetectionFsmUpdate update = fsm_.update(observation);
      if (update.changed) {
        activeStrategy()->reset();
        publishMode();
        if (update.previous_mode == DetectionMode::TRACK_HR &&
            update.mode == DetectionMode::REACQUIRE_HR) {
          ROS_WARN(
              "target_detection_yolo mode: TRACK_HR -> REACQUIRE_HR "
              "valid=%s candidates=%zu confidence=%.3f roi=[%d,%d %dx%d] "
              "ekf_prediction=%s center=[%.1f,%.1f]",
              result.valid ? "true" : "false", result.candidates.size(),
              result.valid ? result.target.confidence : 0.0,
              result.processed_region.x, result.processed_region.y,
              result.processed_region.width, result.processed_region.height,
              context_.has_predicted_target_center ? "true" : "false",
              context_.predicted_target_center.x,
              context_.predicted_target_center.y);
        } else {
          ROS_INFO("target_detection_yolo mode: %s -> %s",
                   DetectionFsm::toString(update.previous_mode),
                   DetectionFsm::toString(update.mode));
        }
      }
      publishDetections(message, result);
      // 调试底图最后发布，使同一帧的检测框和 ROI 先进入下游，避免 overlay
      // 使用上一帧的识别状态绘制当前画面。
      publishDebugImage(message);
    } catch (const cv_bridge::Exception& error) {
      ROS_ERROR_THROTTLE(1.0, "target_detection_yolo image conversion failed: %s",
                         error.what());
    } catch (const std::exception& error) {
      ROS_ERROR_THROTTLE(1.0, "target_detection_yolo inference failed: %s",
                         error.what());
    }
  }

  void publishDebugImage(const sensor_msgs::ImageConstPtr& message) {
    if (debug_image_publisher_.getNumSubscribers() == 0) {
      return;
    }
    const ros::WallTime now = ros::WallTime::now();
    if (debug_image_rate_hz_ <= 0.0) {
      debug_image_publisher_.publish(message);
      return;
    }

    const ros::WallDuration period(1.0 / debug_image_rate_hz_);
    if (next_debug_image_time_.isZero()) {
      next_debug_image_time_ = now;
    }
    if (now < next_debug_image_time_) {
      return;
    }
    debug_image_publisher_.publish(message);
    do {
      next_debug_image_time_ += period;
    } while (next_debug_image_time_ <= now);
  }

  void ekfPredictionCallback(
      const swarm_msgs::BoundingBoxSequence::ConstPtr& message) {
    const ros::Time received = ros::Time::now();
    // Simulated time can jump backwards between runs. Do not retain sequences
    // from the previous clock epoch because their absolute prediction stamps
    // could accidentally bracket a new image.
    if (!ekf_prediction_history_.empty() &&
        received < ekf_prediction_history_.back().received) {
      ekf_prediction_history_.clear();
    }
    ekf_prediction_history_.push_back({received, message});

    // Simulated time starts at zero. During the first history window, an
    // unconditional subtraction would create a negative ros::Time and throw.
    ros::Time oldest_allowed(0);
    if (received.toSec() > ekf_prediction_history_s_) {
      oldest_allowed =
          received - ros::Duration(ekf_prediction_history_s_);
    }
    while (!ekf_prediction_history_.empty() &&
           (ekf_prediction_history_.front().received < oldest_allowed ||
            static_cast<int>(ekf_prediction_history_.size()) >
                ekf_prediction_max_sequences_)) {
      ekf_prediction_history_.pop_front();
    }
  }

  void updateEkfPredictionContext(const cv::Size& image_size,
                                  const ros::Time& image_stamp) {
    context_.has_predicted_target_center = false;
    if (!use_ekf_prediction_ || fsm_.mode() != DetectionMode::TRACK_HR ||
        ekf_prediction_history_.empty()) {
      return;
    }

    const ros::Time now = ros::Time::now();
    // Select the prediction for this image's acquisition time. Do not use
    // callback/inference completion time: the ROI is cropped from this
    // already-acquired image.
    const ros::Time target_time = image_stamp.isZero() ? now : image_stamp;
    const swarm_msgs::BoundingBoxSequence* selected_sequence = nullptr;
    // The newest sequence often starts after the image acquisition stamp due
    // to camera transport latency. Search recent sequences newest-to-oldest
    // and retain the freshest one whose absolute-time interval brackets this
    // image. This uses a prediction made before/at exposure instead of trying
    // to project a current state backwards in time.
    for (auto it = ekf_prediction_history_.rbegin();
         it != ekf_prediction_history_.rend(); ++it) {
      // A Gazebo reset may move /clock backwards before the next prediction
      // callback has had a chance to clear the old epoch.
      if (now < it->received) {
        continue;
      }
      const double receipt_age = (now - it->received).toSec();
      if (!std::isfinite(receipt_age) || receipt_age < 0.0 ||
          receipt_age > ekf_prediction_timeout_s_ || !it->message ||
          it->message->predictions.empty()) {
        continue;
      }
      const auto& candidate = it->message->predictions;
      if (candidate.front().predicted_time <= target_time &&
          target_time <= candidate.back().predicted_time) {
        selected_sequence = it->message.get();
        break;
      }
    }
    if (selected_sequence == nullptr) {
      return;
    }

    const auto& predictions = selected_sequence->predictions;
    const auto after = std::lower_bound(
        predictions.begin(), predictions.end(), target_time,
        [](const swarm_msgs::TimedBoundingBox& prediction,
           const ros::Time& stamp) {
          return prediction.predicted_time < stamp;
        });
    if (after == predictions.end() ||
        (after == predictions.begin() &&
         after->predicted_time != target_time)) {
      return;
    }

    double u = 0.0;
    double v = 0.0;
    if (after == predictions.begin() ||
        after->predicted_time == target_time) {
      u = 0.5 * (static_cast<double>(after->bbox.xmin) + after->bbox.xmax);
      v = 0.5 * (static_cast<double>(after->bbox.ymin) + after->bbox.ymax);
    } else {
      const auto before = std::prev(after);
      const double span =
          (after->predicted_time - before->predicted_time).toSec();
      if (!std::isfinite(span) || span <= 1e-6) {
        return;
      }
      const double alpha =
          std::max(0.0, std::min(
                            1.0,
                            (target_time - before->predicted_time).toSec() /
                                span));
      const double before_u =
          0.5 * (static_cast<double>(before->bbox.xmin) + before->bbox.xmax);
      const double before_v =
          0.5 * (static_cast<double>(before->bbox.ymin) + before->bbox.ymax);
      const double after_u =
          0.5 * (static_cast<double>(after->bbox.xmin) + after->bbox.xmax);
      const double after_v =
          0.5 * (static_cast<double>(after->bbox.ymin) + after->bbox.ymax);
      u = before_u + alpha * (after_u - before_u);
      v = before_v + alpha * (after_v - before_v);
    }
    if (!std::isfinite(u) || !std::isfinite(v) || u < 0.0 || v < 0.0 ||
        u >= image_size.width || v >= image_size.height) {
      return;
    }

    context_.predicted_target_center = cv::Point2f(
        static_cast<float>(u), static_cast<float>(v));
    context_.has_predicted_target_center = true;
  }

  // ========================================================================
  // 2.4 ROS Publishers（ROS 消息发布）
  // ========================================================================
  // 将策略输出的所有候选框转换为 swarm_msgs::BoundingBoxes。消息时间戳和
  // frame_id 沿用输入图像，便于下游节点进行时序同步和坐标解释。
  void publishDetections(const sensor_msgs::ImageConstPtr& image,
                         const DetectionResult& result) {
    swarm_msgs::BoundingBoxes message;
    message.header = image->header;
    message.image_header = image->header;
    if (!result.valid) {
      bbox_publisher_.publish(message);
      return;
    }

    const auto append_box = [&message](const DetectionBox& candidate) {
      swarm_msgs::BoundingBox box;
      box.probability = candidate.confidence;
      box.xmin = static_cast<int>(candidate.rect.x);
      box.ymin = static_cast<int>(candidate.rect.y);
      box.xmax = static_cast<int>(candidate.rect.x + candidate.rect.width);
      box.ymax = static_cast<int>(candidate.rect.y + candidate.rect.height);
      box.id = static_cast<int16_t>(candidate.class_id);
      message.bounding_boxes.push_back(box);
    };

    // 下游历史节点会直接读取 bounding_boxes.front()，因此无论检测器原始
    // 输出顺序如何，都先发布本帧经过关联后选中的目标。
    append_box(result.target);
    bool skipped_selected_candidate = false;
    for (const DetectionBox& candidate : result.candidates) {
      const bool is_selected =
          !skipped_selected_candidate &&
          candidate.class_id == result.target.class_id &&
          candidate.confidence == result.target.confidence &&
          candidate.rect == result.target.rect;
      if (is_selected) {
        skipped_selected_candidate = true;
        continue;
      }
      append_box(candidate);
    }
    bbox_publisher_.publish(message);
  }

  // 发布可读的 FSM 模式名称。该发布器为 latch 模式。
  void publishMode() {
    std_msgs::String message;
    message.data = DetectionFsm::toString(fsm_.mode());
    mode_publisher_.publish(message);
  }

  // 发布本帧真正送入检测器的原图区域。TRACK_LR 对应完整图像，
  // TRACK_HR 和 REACQUIRE_HR 分别对应跟踪 ROI 与当前扫描图块。
  void publishRoi(const cv::Rect& region) {
    sensor_msgs::RegionOfInterest message;
    if (region.width > 0 && region.height > 0) {
      message.x_offset = static_cast<uint32_t>(std::max(0, region.x));
      message.y_offset = static_cast<uint32_t>(std::max(0, region.y));
      message.width = static_cast<uint32_t>(region.width);
      message.height = static_cast<uint32_t>(region.height);
    }
    message.do_rectify = false;
    roi_publisher_.publish(message);
  }

  // ========================================================================
  // 2.5 ROS Resources and Runtime State（ROS 资源与运行状态）
  // ========================================================================
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber image_subscriber_;
  image_transport::Publisher debug_image_publisher_;
  ros::Subscriber ekf_prediction_subscriber_;
  ros::Publisher bbox_publisher_;
  ros::Publisher mode_publisher_;
  ros::Publisher roi_publisher_;
  ros::ServiceServer reset_trial_service_;

  // FSM 只负责模式决策；TrackingContext 保存跨帧目标信息；检测器由三个
  // 策略共享，而每个策略分别维护自己的区域选择状态。
  DetectionFsm fsm_;
  TrackingContext context_;
  std::unique_ptr<YoloDetectorAdapter> detector_;
  std::unique_ptr<ReacquireHrStrategy> reacquire_hr_;
  std::unique_ptr<TrackHrStrategy> track_hr_;
  std::unique_ptr<TrackLrStrategy> track_lr_;
  struct ReceivedPredictionSequence {
    ros::Time received;
    swarm_msgs::BoundingBoxSequence::ConstPtr message;
  };
  std::deque<ReceivedPredictionSequence> ekf_prediction_history_;
  bool use_ekf_prediction_{true};
  double ekf_prediction_timeout_s_{0.15};
  double ekf_prediction_history_s_{0.50};
  int ekf_prediction_max_sequences_{32};
  double debug_image_rate_hz_{10.0};
  ros::WallTime next_debug_image_time_;
};

}  // namespace target_detection_yolo

// ============================================================================
// 3. Program Entry（程序入口）
// ============================================================================
// 初始化 ROS 节点并进入单线程回调循环。构造或运行初始化失败时记录致命错误
// 并以非零状态退出，便于 roslaunch 识别节点启动失败。
int main(int argc, char** argv) {
  ros::init(argc, argv, "target_detection_yolo_node");
  try {
    target_detection_yolo::TargetDetectionNode node;
    ros::spin();
  } catch (const std::exception& error) {
    // TargetDetectionNode owns the last NodeHandles in this process. During
    // stack unwinding they are destroyed before control reaches this catch,
    // so ROS logging may already be shut down and silently drop ROS_FATAL.
    std::fprintf(stderr, "target_detection_yolo_node failed: %s\n",
                 error.what());
    std::fflush(stderr);
    ROS_FATAL("target_detection_yolo_node failed: %s", error.what());
    return 1;
  }
  return 0;
}
