#include <algorithm>
#include <string>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <swarm_msgs/BoundingBox.h>
#include <swarm_msgs/BoundingBoxes.h>

namespace {

// ============================================================================
//  Node State（节点状态）
// ============================================================================
bool show_rgb = false;
bool show_hsv = false;
ros::Publisher detections_pub;

// ============================================================================
//  Image Conversion（图像转换）
// ============================================================================
// Convert a ROS Image into an owned BGR matrix.（将 ROS 图像转换为独立 BGR 矩阵。）
// Validate the layout to prevent malformed messages from causing an overrun.
//（校验图像布局以防畸形消息导致越界。）
bool imageMsgToBgrMat(const sensor_msgs::ImageConstPtr& msg, cv::Mat& bgr_img) {
  const int channels =
      msg->encoding == sensor_msgs::image_encodings::MONO8 ? 1 : 3;
  const size_t min_step = static_cast<size_t>(msg->width) * channels;
  if (msg->height == 0 || msg->width == 0 || msg->step < min_step ||
      msg->data.size() < msg->step * msg->height) {
    ROS_WARN_THROTTLE(
        1.0,
        "Invalid image: encoding=%s width=%u height=%u step=%u data=%zu",
        msg->encoding.c_str(), msg->width, msg->height, msg->step,
        msg->data.size());
    return false;
  }

  const int cv_type = channels == 1 ? CV_8UC1 : CV_8UC3;
  cv::Mat src(msg->height, msg->width, cv_type,
              const_cast<uint8_t*>(msg->data.data()), msg->step);
  if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
    bgr_img = src.clone();
  } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
    cv::cvtColor(src, bgr_img, cv::COLOR_RGB2BGR);
  } else if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
    cv::cvtColor(src, bgr_img, cv::COLOR_GRAY2BGR);
  } else {
    ROS_WARN_THROTTLE(1.0, "Unsupported image encoding: %s",
                      msg->encoding.c_str());
    return false;
  }
  return true;
}

// ============================================================================
//  HSV Balloon Detection（HSV 气球检测）
// ============================================================================
// Detect red candidates and publish deterministic left-to-right bounding boxes.
//（检测红色候选区域，并按从左到右的确定顺序发布检测框。）
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  swarm_msgs::BoundingBoxes detections;
  // Preserve capture time for downstream delay compensation.（保留采集时间供下游时延补偿。）
  detections.header = msg->header;

  cv::Mat image;
  if (!imageMsgToBgrMat(msg, image)) {
    return;
  }

  cv::Mat hsv;
  cv::Mat red_high;
  cv::Mat red_low;
  cv::Mat mask;

  // ========================================================================
  //  Red HSV Segmentation（红色 HSV 分割）
  // ========================================================================
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  // Red wraps around OpenCV's [0, 180] hue range, so merge two intervals.
  //（红色跨越 OpenCV 的 H 范围首尾，因此合并两段阈值。）
  // S/V lower bounds reject gray and dark regions.（S/V 下限用于抑制灰暗区域。）
  cv::inRange(hsv, cv::Scalar(171, 80, 40), cv::Scalar(180, 255, 255),
              red_high);
  cv::inRange(hsv, cv::Scalar(0, 80, 40), cv::Scalar(10, 255, 255),
              red_low);
  cv::bitwise_or(red_high, red_low, mask);

  // ========================================================================
  //  Connected Components（连通域提取）
  // ========================================================================
  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int num_labels =
      cv::connectedComponentsWithStats(mask, labels, stats, centroids);

  int id = 0;
  for (int i = 1; i < num_labels; ++i) {
    const int left = stats.at<int>(i, cv::CC_STAT_LEFT);
    const int top = stats.at<int>(i, cv::CC_STAT_TOP);
    const int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
    const int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
    const int area = stats.at<int>(i, cv::CC_STAT_AREA);

    // Reject isolated color noise below the historical 25-pixel threshold.
    //（使用历史 25 像素阈值剔除孤立颜色噪声。）
    if (area <= 25) {
      continue;
    }

    swarm_msgs::BoundingBox bbox;
    bbox.Class = "Ballon";
    bbox.probability = 1.0;
    bbox.xmin = left;
    bbox.ymin = top;
    bbox.xmax = left + width;
    bbox.ymax = top + height;
    bbox.id = id++;
    bbox.a = width;
    bbox.b = height;
    detections.bounding_boxes.push_back(bbox);

    cv::rectangle(image, cv::Point(bbox.xmin, bbox.ymin),
                  cv::Point(bbox.xmax, bbox.ymax), cv::Scalar(0, 255, 0), 2);
  }

  // ========================================================================
  //  Detection Ordering（检测结果排序）
  // ========================================================================
  // Reassign IDs after sorting to keep each frame deterministic.
  //（排序后重排 ID，确保单帧结果具有确定性。）
  // Omit the common divisor when comparing center coordinates.
  //（比较中心坐标时省略共同除数。）
  std::sort(detections.bounding_boxes.begin(), detections.bounding_boxes.end(),
            [](const swarm_msgs::BoundingBox& lhs,
               const swarm_msgs::BoundingBox& rhs) {
              return lhs.xmin + lhs.xmax < rhs.xmin + rhs.xmax;
            });
  id = 0;
  for (auto& bbox : detections.bounding_boxes) {
    bbox.id = id++;
  }

  detections_pub.publish(detections);

  // Keep GUI output optional for headless onboard deployment.（机载无显示器部署时默认关闭 GUI。）
  if (show_rgb) {
    cv::imshow("target_detection_hsv/rgb", image);
    cv::waitKey(1);
  }
  if (show_hsv) {
    cv::imshow("target_detection_hsv/mask", mask);
    cv::waitKey(1);
  }
}

}  // namespace（匿名命名空间）

// ============================================================================
//  ROS Node Entry（ROS 节点入口）
// ============================================================================
int main(int argc, char** argv) {
  ros::init(argc, argv, "target_detection_hsv");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // ========================================================================
  //  Parameters（参数）
  // ========================================================================
  std::string image_topic;
  std::string detection_topic;
  private_nh.param<std::string>("image_topic", image_topic,
                                "/camera/image_raw");
  private_nh.param<std::string>("detection_topic", detection_topic,
                                "/tracker/p_image_hsv");
  private_nh.param("show_rgb", show_rgb, false);
  private_nh.param("show_hsv", show_hsv, false);

  // ========================================================================
  //  ROS Interfaces（ROS 接口）
  // ========================================================================
  // A one-frame input queue favors fresh images when detection falls behind.
  //（检测滞后时使用单帧队列优先处理最新图像。）
  ros::Subscriber image_sub = nh.subscribe(image_topic, 1, imageCallback);
  detections_pub =
      nh.advertise<swarm_msgs::BoundingBoxes>(detection_topic, 10);

  ROS_INFO("HSV detector: image=%s detections=%s", image_topic.c_str(),
           detection_topic.c_str());
  ros::spin();
  return 0;
}
